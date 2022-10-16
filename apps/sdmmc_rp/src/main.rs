//#![feature(is_some_with)]
#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
extern crate alloc;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use embedded_sdmmc::SdMmcSpi;
use rp_pico::hal::gpio;
use rp_pico::hal::spi;
use rp_pico::hal::Clock;

//use cortex_m::delay::Delay;

// Embed the `Hz` function/trait:
use fugit::RateExtU32;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

mod dump;
use arrayvec::ArrayString;

// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;
use rp_pico::hal::rom_data::reset_to_usb_boot;
//use rp_pico::hal::rom_data::rom_version_number;

// Import the GPIO abstraction:
//use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use core::fmt::Write;
use rp_pico::hal;

use embedded_hal::digital::v2::OutputPin;
//use embedded_hal::digital::v2::InputPin;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceBuilder;
use usb_device::device::UsbVidPid;
use usbd_serial::SerialPort;

//static mut screen_buffer: [u16;240 * 240 ] = [0;240 * 240 ];
use core::mem::MaybeUninit;
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 15 * 1024;

fn stopwatch_start(timer: &hal::Timer) -> u64 {
    timer.get_counter()
}

fn stopwatch_delta(start: u64, timer: &hal::Timer) -> u64 {
    timer.get_counter() - start
}

// Setup some blinking codes:
const BLINK_OK_LONG: [u8; 1] = [8u8];
const BLINK_OK_SHORT_LONG: [u8; 4] = [1u8, 0u8, 6u8, 0u8];
const BLINK_OK_SHORT_SHORT_LONG: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 6u8, 0u8];
const BLINK_ERR_2_SHORT: [u8; 4] = [1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_3_SHORT: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_4_SHORT: [u8; 8] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_5_SHORT: [u8; 10] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_6_SHORT: [u8; 12] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];

fn blink_signals(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) {
    for bit in sig {
        if *bit != 0 {
            pin.set_high().unwrap();
        } else {
            pin.set_low().unwrap();
        }

        let length = if *bit > 0 { *bit } else { 1 };

        for _ in 0..length {
            delay.delay_ms(100);
        }
    }

    pin.set_low().unwrap();

    delay.delay_ms(500);
}

fn blink_signals_loop(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) -> ! {
    loop {
        blink_signals(pin, delay, sig);
        delay.delay_ms(1000);
    }
}

#[entry]
fn main() -> ! {
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_high().unwrap();

    // Setup a delay for the LED blink signals:
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("EmbeddedRust")
        .product("SDMMCTEST")
        .serial_number("0.3")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // These are implicitly used by the spi driver if they are in the correct mode
    // let _spi_sclk = pins.gpio18.into_mode::<gpio::FunctionSpi>();  // SPICLK
    // let _spi_mosi = pins.gpio19.into_mode::<gpio::FunctionSpi>();  // TS MOSI
    // let _spi_miso = pins.gpio16.into_mode::<gpio::FunctionSpi>();  // RX MISO
    // let spi_cs = pins.gpio17.into_push_pull_output();              // CS

    // Create an 8 bit SPI driver instance for the SPI0 device
    // let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

    // // Exchange the uninitialised SPI driver for an initialised one
    // let spi = spi.init(
    //     &mut pac.RESETS,
    //     clocks.peripheral_clock.freq(),
    //     1_000_000u32.Hz(),
    //     &embedded_hal::spi::MODE_0,
    // );

    // //info!("Aquire SPI SD/MMC BlockDevice...");
    // let mut sdspi = SdMmcSpi::new(spi, spi_cs);
    // blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut strbuf = ArrayString::<4096>::new();
    writeln!(&mut strbuf, "Version = {} !\n", "0.01").unwrap();

    // Next we need to aquire the block device and initialize the
    // communication with the SD card.
    // let block = match sdspi.acquire() {
    //     Ok(block) => {
    // // Create a fixed buffer to store screen contents
    //         block
    //     },
    //     Err(e) => {
    //         writeln!(&mut strbuf, "Err = {:?} !\n", e).unwrap();
    //  //       blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_2_SHORT);
    //         //error!("Error retrieving card size: {}", defmt::Debug2Format(&e));
    //         //blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_2_SHORT);
    //     }
    // };

    //    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2000_000_000 {
            said_hello = true;
            drop(serial.write(b"\r\n *** Welcome to PICO Rustzeighty v0.1d *** \r\n"));
            drop(serial.write(strbuf.as_bytes()));
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });

                    match buf[0] as char {
                        'A' => {
                            writeln!(strbuf, "RESET {}\r\n", false).unwrap();
                            drop(serial.write(strbuf.as_bytes()));
                        }

                        'F' => {
                            drop(serial.write(b"Boot to Flash mode!\r\n"));
                            reset_to_usb_boot(0, 0);
                        }
                        _ => {
                            drop(serial.write(b"Unkown command \r\n"));
                            strbuf.clear();
                            unsafe {
                                let ptr = HEAP.as_ptr();
                                writeln!(
                                    strbuf,
                                    "HEAP Used: {} bytes, free: {} bytes HEAP {:?}",
                                    ALLOCATOR.used(),
                                    ALLOCATOR.free(),
                                    ptr
                                )
                                .unwrap();
                            }
                            drop(serial.write(strbuf.as_bytes()));
                        }
                    }

                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}
