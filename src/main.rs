#![no_std]
#![no_main]

//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Pointing device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and license details.
//!
use embedded_hal::digital::v2::OutputPin;
///!https://docs.rs/crate/rp-pico/latest/source/examples/pico_usb_twitchy_mouse.rs
// The entry macro, defining the entrypoint of the application
use rp_pico::entry;

use rp_pico::hal::gpio::FunctionSio;
// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

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

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

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

    // Workaround for erata 5
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led_pin: hal::gpio::Pin<
        hal::gpio::bank0::Gpio25,
        FunctionSio<hal::gpio::SioOutput>,
        hal::gpio::PullDown,
    > = pins.led.into_push_pull_output();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports at 100Hz
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 10);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x8008, 0x5000))
        .manufacturer("BAE Systems Digital Intelligence")
        .product("SNEEZE BUTTON 2000")
        .serial_number("00000000001")
        .device_class(0x03)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);

        // https://docs.rs/usbd-hid/latest/usbd_hid/descriptor/struct.KeyboardReport.html
        // https://files.microscan.com/helpfiles/ms4_help_file/ms-4_help-02-46.html
        // https://www.win.tue.nl/~aeb/linux/kbd/scancodes-14.html
        // Indicate A button down
        let rep_down = KeyboardReport {
            modifier: 0u8, // 0x01 => left control
            reserved: 0u8, // Reserved
            leds: 0u8,     // No LEDs lit
            keycodes: [0x04u8, 0u8, 0u8, 0u8, 0u8, 0u8],
        };
        push_keyboard_report(rep_down).unwrap_or(0);

        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        // Indicate all keys are up
        let rep_up = KeyboardReport {
            modifier: 0u8, // 0x01 => left control
            reserved: 0u8, // Reserved
            leds: 0u8,     // No LEDs lit
            keycodes: [0u8, 0u8, 0u8, 0u8, 0u8, 0u8],
        };
        push_keyboard_report(rep_up).unwrap_or(0);
    }
}

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
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}
