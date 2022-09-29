#![cfg_attr(not(test), no_std)]
#![no_main]

#![feature(alloc_error_handler)]
extern crate alloc;

mod peripherals;

//use alloc::vec;
//use alloc::vec::Vec;
use alloc_cortex_m::CortexMHeap;
use rz80::Memory;
use core::alloc::Layout;

use peripherals::Peripherals;


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

// Pull in any important traits
//use rp_pico::hal::prelude::*;

// Embed the `Hz` function/trait:
//use embedded_time::rate::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

use rp_pico::hal::rom_data::reset_to_usb_boot;
use rp_pico::hal::rom_data::rom_version_number;

// Import the GPIO abstraction:
//use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use core::{fmt::Write};

use embedded_hal::digital::v2::OutputPin;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceBuilder;
use usb_device::device::UsbVidPid;
use usbd_serial::SerialPort;


//static mut screen_buffer: [u16;240 * 240 ] = [0;240 * 240 ];
use core::mem::MaybeUninit;
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 145 * 1024;


fn stopwatch_start(timer : &hal::Timer) -> u64 {
    timer.get_counter()
}

fn stopwatch_delta(start : u64, timer : &hal::Timer) -> u64 {
    timer.get_counter() - start
}


#[entry]
fn main() -> ! {

    let mut mem = Memory::new();
    let mut cpu = rz80::CPU::new();
    let mut per = Peripherals::new();



    // 1:     -	0000          	    org 0x0000
    // 2:    0+7	0000  3E0A    	    ld  a,10
    // 3:     -	0002          	.loop:
    // 4:    7+4	0002  3C      	    inc a
    // 5:   11+10	0003  C30200  	    jp .loop


    mem.heap[0x0000] = 0x3E;
    mem.heap[0x0001] = 0x0A;
    mem.heap[0x0002] = 0x3C;
    mem.heap[0x0003] = 0xC3;
    mem.heap[0x0004] = 0x02;
    mem.heap[0x0005] = 0x00;

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

    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;

            drop( serial.write(b"Welcome to PICO Rustzeighty v0.1a \r\n") );
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

                    match buf[0] as char {
                        'D' => {
                            drop( serial.write(b"Delay checl!\r\n") );
                            let start = stopwatch_start( &timer );
                            delay.delay_ms(100);
                            let stop = timer.get_counter();
                            let delta  = stopwatch_delta(start, &timer);
                            strbuf.clear();
                            writeln!(strbuf, "Time check :c start {} stop  {} delta {:?}", start, stop ,delta/1000).unwrap();
                            drop( serial.write(strbuf.as_bytes()));

                        } 
                        'E' => {
                            let start = stopwatch_start( &timer );
                            strbuf.clear();

                            cpu.step(&mut per, &mut mem);

                            
                            let delta  = stopwatch_delta(start, &timer);
                            writeln!(strbuf, "Time: {} After cycle : pc: {:04X} A: {:02X}", delta, cpu.reg.pc(), cpu.reg.a()).unwrap();
                            //drop( writeln!(strbuf, "After cycle :c start {} delta {} delta/loop {}", start,delta, delta/60));
                            drop( serial.write(strbuf.as_bytes()));

                        }
                        'F' => {
                            drop( serial.write(b"Boot to Flash mode!\r\n") );
                            reset_to_usb_boot(0,0);    
                        } 
                        'H' => {
                            drop( serial.write(b"Check heap!\r\n") );
                            reset_to_usb_boot(0,0);    
                        } 
                        'R' => {
                            drop( serial.write(b"Restart \r\n") );
                            reset_to_usb_boot(0,2);    
                        } 
                        _ => {
                            drop( serial.write(b"Unkown command \r\n") );
                            strbuf.clear();
                            unsafe { 
                                let ptr =  HEAP.as_ptr();
                                writeln!(strbuf, "HEAP Used: {} bytes, free: {} bytes HEAP {:?}",
                                ALLOCATOR.used(),
                                ALLOCATOR.free() ,
                                            ptr ).unwrap();
                                            
                            }
                            drop( serial.write(strbuf.as_bytes()));
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
                    
                    // // dump memory T
                    // if buf[0] == 84 {
                    //         for lp in 100..160 {
                    //             let mut idx = 0;
                    //             for _yp in 0..display_height {
                    //             for xp in 0..display_width {
                    //                     if xp < lp {
                    //                         screen_buffer[idx] = 0x0FF0;
                    //                     } else {
                    //                         screen_buffer[idx] = 0xF00F;
                    //                     }
                    //                     idx += 1;
                    //                 }
                    //         }
                    //         drop( display.set_pixels(0,0,(display_width -1) as u16 ,(display_height -1 ) as u16 , screen_buffer.iter().map(|x| *x)));
                    //     }
                    //     serial.write(b"LNE DONE\r\n").unwrap();
                    
                        
                    //     // strbuf.clear();
                    //     // unsafe {
                    //     //     dump::dump( slice::from_raw_parts(COUNTER_ADDRESS as *const u8 , 32), 0, &mut strbuf);
                    //     // }

                    //     // drop( serial.write(strbuf.as_bytes()));
                    // }                    
                    // Send back to the host
                }
            }
        }
    }
}
