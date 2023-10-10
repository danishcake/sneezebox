#![no_std]
#![no_main]

///! Baseed on the USB twitchy mouse example and the 'rusty-keys' firmware
///!https://github.com/KOBA789/rusty-keys/blob/7194a0ec3bf5656610075125e239bd3b941fbad4/firmware/keyboard/src/bin/simple.rs#L24
///!https://docs.rs/crate/rp-pico/latest/source/examples/pico_usb_twitchy_mouse.rs
// The entry macro, defining the entrypoint of the application
use rp_pico::entry;

// Clocks
use rp_pico::hal::Clock;

// Pin traits
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_probe as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;
use usbd_hid::hid_class::HidClassSettings;
use usbd_hid::hid_class::HidCountryCode;
use usbd_hid::hid_class::HidProtocol;
use usbd_hid::hid_class::HidSubClass;
use usbd_hid::hid_class::ProtocolModeConfig;

// Debug printing
use defmt::*;
use defmt_rtt as _;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    debug!("SB:init start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Obtain LED pin, button LED pin and button state pin
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let mut onboard_led_pin = pins.led.into_push_pull_output();
    let mut button_led_pin = pins.gpio10.into_push_pull_output();
    let button_in_pin = pins.gpio9.into_pull_up_input();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    // Move the USB driver to a global location, as it is required to be alive during
    // interrupts. We are promising to the compiler not to take mutable access to this
    // global variable whilst this reference exists.
    let bus_ref = unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
        USB_BUS.as_ref().unwrap()
    };

    // Set up the USB HID Class Device driver, providing keyboard Reports at 100Hz
    let usb_hid = HIDClass::new_ep_in_with_settings(
        bus_ref,
        KeyboardReport::desc(),
        10,
        HidClassSettings {
            subclass: HidSubClass::NoSubClass,
            protocol: HidProtocol::Keyboard,
            config: ProtocolModeConfig::ForceReport,
            locale: HidCountryCode::NotSupported,
        },
    );
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x8008, 0x5000))
        .manufacturer("BAE Systems Digital Intelligence")
        .product("SNEEZE BUTTON 2001")
        .serial_number("00000000001")
        .device_class(0x00)
        .device_sub_class(0x00)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    debug!("SB:init complete");

    let mut debounce_count = 0u8;
    let debounce_time_ms = 10u8;
    let mut debounce_state = false;

    loop {
        let last_debounce_state = debounce_state;
        match button_in_pin.is_low() {
            Ok(true) => {
                if debounce_count < debounce_time_ms {
                    debounce_count += 1;
                }
                if debounce_count == debounce_time_ms {
                    debounce_state = true
                }
            }
            Ok(false) | Err(_) => {
                if debounce_count > 0 {
                    debounce_count -= 1;
                }
                if debounce_count == 0 {
                    debounce_state = false;
                }
            }
        };

        match (debounce_state, last_debounce_state) {
            (true, false) => {
                debug!("SB:Transition high");
                onboard_led_pin.set_high().unwrap_or_default();
                button_led_pin.set_high().unwrap_or_default();

                // Indicate alt-control-NUM3
                let rep_down = KeyboardReport {
                    modifier: 0x05u8,                          // 0x01 => left control, 0x04 => left alt
                    reserved: 0u8,                             // Reserved
                    leds: 0u8,                                 // No LEDs lit
                    keycodes: [91u8, 0u8, 0u8, 0u8, 0u8, 0u8], // KP3
                };
                push_keyboard_report(rep_down).unwrap_or_default();
            }
            (false, true) => {
                debug!("SB:Transition low");
                onboard_led_pin.set_low().unwrap_or_default();
                button_led_pin.set_low().unwrap_or_default();

                // Indicate all keys are up
                let rep_up = KeyboardReport {
                    modifier: 0u8, // 0x01 => left control
                    reserved: 0u8, // Reserved
                    leds: 0u8,     // No LEDs lit
                    keycodes: [0u8, 0u8, 0u8, 0u8, 0u8, 0u8],
                };
                push_keyboard_report(rep_up).unwrap_or(0);
            }
            _ => { /* No change */ }
        }

        delay.delay_ms(1);
    }
}

/// Send a USB keyboard report
/// https://docs.rs/usbd-hid/latest/usbd_hid/descriptor/struct.KeyboardReport.html
/// https://files.microscan.com/helpfiles/ms4_help_file/ms-4_help-02-46.html
/// https://www.win.tue.nl/~aeb/linux/kbd/scancodes-14.html
fn push_keyboard_report(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    debug!("SB:usb int"); // Not a good idea to log from interrupt handler...
                          // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
