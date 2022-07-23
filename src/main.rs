#![no_std]
#![no_main]

#![feature(alloc_error_handler)]
extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use alloc_cortex_m::CortexMHeap;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::primitives::Rectangle;
use core::alloc::Layout;



#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}


mod dump;

use arrayvec::ArrayString;
use embedded_graphics::Drawable;
use embedded_graphics::image::Image;
use embedded_graphics::image::ImageRawLE;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::Point;
use embedded_graphics::geometry::Size;

use embedded_graphics::prelude::RgbColor;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// Embed the `Hz` function/trait:
use embedded_time::rate::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

use rp_pico::hal::rom_data::reset_to_usb_boot;
use rp_pico::hal::rom_data::rom_version_number;
// Import the SPI abstraction:
use rp_pico::hal::spi;

// Import the GPIO abstraction:
use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use core::{fmt::Write, slice};

use embedded_hal::digital::v2::OutputPin;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceBuilder;
use usb_device::device::UsbVidPid;
use usbd_serial::SerialPort;

const FLASH_BASE : u32= 0x1000_0000;
const COUNTER_OFFSET : u32= 0x0010_0000;
const COUNTER_ADDRESS : u32= FLASH_BASE + COUNTER_OFFSET; 


//static mut screen_buffer: [u16;240 * 240 ] = [0;240 * 240 ];
use core::mem::MaybeUninit;
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 145 * 1024;

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

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);

    delay.delay_ms(1);

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
        .product("Eyes")
        .serial_number("0.1")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // experiment with some bootrom functions and data
    let version_nb = rom_version_number();
    // Create a fixed buffer to store screen contents
    let mut strbuf = ArrayString::<500>::new();
    writeln!(&mut strbuf, "Version = {} !\n", version_nb).unwrap();
    

    //flash_experiment();

    // SETUP SCREEN

    // SCL		GP10 (pin 14)
    // SDA		GP11 (pin 15) (MOSI)
    // RES		GP12 (pin 16)
    // DC		GP13 (pin 17)

    // Configure pins
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();

    let dc = pins.gpio13.into_push_pull_output();
    let res = pins.gpio12.into_push_pull_output();

    // Chip select
    let cs = pins.gpio15.into_push_pull_output();
    // Backlight
    // let bl = pins.gpio14.into_push_pull_output();

    // Setup and init the SPI device (SPI1 !)
    let spi = rp_pico::hal::Spi::<_, _, 8>::new(pac.SPI1);

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let display_width = 240;
    let display_height = 240;

    let display_interface = display_interface_spi::SPIInterface::new(spi, dc, cs);

    let mut display = st7789::ST7789::new(
        display_interface,
        res,
        display_width as _,
        display_height as _,
    );

    // initialize
    display.init(&mut delay).unwrap();

    // set default orientation
    display
        .set_orientation(st7789::Orientation::Landscape)
        .unwrap();
    display
        .set_tearing_effect(st7789::TearingEffect::HorizontalAndVertical)
        .unwrap();

    drop( display.clear(Rgb565::GREEN));



    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 86);
    let ferris = Image::new(&raw_image_data, Point::new(34, 8));

    let mut screen_buffer : Vec<u16> = vec![55u16; display_width * display_height ];

   // let raw_image_vec = ImageRawLE::new( &screen_buffer, display_width as u32);
   // let vec_img = Image::new(&raw_image_vec, Point::new(0, 0));

    let mut idx = 0;
    for yp in 0..display_height {
           for xp in 0..display_width {
                if xp < 100 && yp < 150 {
                    screen_buffer[idx] = 0x0FF0;
                } else {
                    screen_buffer[idx] = 0xFFFF;
                }
                idx += 1;
            }
    }

    //drop( display.set_pixels(0,0,display_width,display_height, screen_buffer.iter().map(|x| *x)));
    ferris.draw(&mut display).unwrap();

    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;

            drop( serial.write(b"Welcome to PICO ALTEST v0.2b \r\n") );
            drop( serial.write(strbuf.as_bytes()));
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

                    // check for reset R
                    if buf[0] == 82 {
                        drop( serial.write(b"Going down!!!!\r\n") );
                        reset_to_usb_boot(0,0);
                        //watchdog.enable_tick_generation((rp_pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);
                    }
                    // check for S
                    if buf[0] == 83 {
                        strbuf.clear();
                        let v1 = vec![2i32;10];
  


                        unsafe { 
                           let ptr =  HEAP.as_ptr();


                            writeln!(strbuf, "HEAP Used: {} bytes, free: {} bytes HEAP {:?} v1 {} v2 {} ",
                        ALLOCATOR.used(),
                        ALLOCATOR.free() ,
                        ptr,
                        v1[1],
                        screen_buffer[1]

                    ).unwrap();
                        }
                        drop( serial.write(strbuf.as_bytes()));

                        // unsafe {
                        //     let counter = COUNTER_ADDRESS as *mut u32;
                        //     let counter_val = *counter;    

                        //     strbuf.clear();
                        //     writeln!(strbuf,"Counter value {}", counter_val).unwrap();
                        //     //dump::dump( slice::from_raw_parts(COUNTER_ADDRESS as *const u8 , 4), 0, &mut strbuf);
                        // }
                        // drop( serial.write(b"FLASH experiment\r\n") );
                        // drop( serial.write(strbuf.as_bytes()));
                        // //flash_experiment();
                        // drop( serial.write(b"FLASH experiment finished \r\n") );
                    }

                    // dump memory T
                    if buf[0] == 84 {
                        serial.write(b"Memory dump flash at COUNTER_ADDRESS\r\n").unwrap();
                        strbuf.clear();
                        unsafe {
                            dump::dump( slice::from_raw_parts(COUNTER_ADDRESS as *const u8 , 32), 0, &mut strbuf);
                        }

                        drop( serial.write(strbuf.as_bytes()));
                    }                    
                    // Send back to the host
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
