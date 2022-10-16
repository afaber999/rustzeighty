//#![feature(is_some_with)]
#![cfg_attr(not(test), no_std)]
#![no_main]

#![feature(alloc_error_handler)]
extern crate alloc;

mod peripherals;

//use alloc::vec;
//use alloc::vec::Vec;
use alloc_cortex_m::CortexMHeap;
//use cortex_m::prelude::_embedded_hal_blocking_delay_DelayUs;
use peripherals::iochannel::IOChannel;
use peripherals::sd_spi;
use peripherals::spi_interface::SpiInterface;
use rz80::Memory;
use core::alloc::Layout;

use peripherals::Peripherals;
//use cortex_m::delay::Delay;

use peripherals::sw_spi::SwSpi;


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
//use embedded_hal::digital::v2::InputPin;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDeviceBuilder;
use usb_device::device::UsbVidPid;
use usbd_serial::SerialPort;


//static mut screen_buffer: [u16;240 * 240 ] = [0;240 * 240 ];
use core::mem::MaybeUninit;
static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 15 * 1024;


fn stopwatch_start(timer : &hal::Timer) -> u64 {
    timer.get_counter()
}

fn stopwatch_delta(start : u64, timer : &hal::Timer) -> u64 {
    timer.get_counter() - start
}

// Target CPU frequency in kHZ
const FREQ_KHZ: i64 = 2458;


pub fn step_frame(  
    micro_seconds: i64, 
    cpu : &mut rz80::CPU, 
    peripherals : &mut peripherals::Peripherals, 
    mem : &mut Memory) -> Option<usize>{

    let num_cycles = (FREQ_KHZ * micro_seconds) / 1000;
    let mut cur_cycles = 0;
    while cur_cycles < num_cycles {
    
    // writeln!( trace_file, " PC:0x{:04X} f:{:08b} a:0x{:02X} bc:0x{:04X} de:0x{:04X} hl:0x{:04X} sp:0x{:04X}", 
    //             cpu.reg.pc() as u16, 
    //             cpu.reg.f()  as u8,
    //             cpu.reg.a() as u8,
    //             cpu.reg.bc() as u16,
    //             cpu.reg.de() as u16,
    //             cpu.reg.hl() as u16,
    //             cpu.reg.sp()  as u16).expect("TRACEERR");
    // let dbg_trace = peripherals.get_dbg_trace();
    // if dbg_trace>0 {
    //     println!(" PC:0x{:04X} f:{:08b} a:0x{:02X} bc:0x{:04X} de:0x{:04X} hl:0x{:04X} sp:0x{:04X}", 
    //     cpu.reg.pc() as u16, 
    //     cpu.reg.f()  as u8,
    //     cpu.reg.a() as u8,
    //     cpu.reg.bc() as u16,
    //     cpu.reg.de() as u16,
    //     cpu.reg.hl() as u16,
    //     cpu.reg.sp()  as u16);

    // }

        let op_cycles = cpu.step(peripherals, mem);
        cur_cycles += op_cycles;
        match cpu.reg.pc() {
            //0x0005 => { cpm_bdos(&mut cpu); },  // emulated CP/M BDOS call
            0xFFFF => { return None },
            _ => { },
        }
    }
    Some(num_cycles as usize)
}

#[entry]
fn main() -> ! {

    let mut mem = Memory::new();
    let mut cpu = rz80::CPU::new();
    let mut num_cycles = 0;        


    let mut mm : u64 = 0;
    let mut tt : u64 = 0;

    static ROMCODE: &'static [u8] = include_bytes!("..\\..\\..\\asm\\firmware\\firmware.bin");
    //static ROMCODE: &'static [u8] = include_bytes!("..\\..\\..\\asm\\tests\\sio_echo.bin");

    // 1:     -	0000          	    org 0x0000
    // 2:    0+7	0000  3E0A    	    ld  a,10
    // 3:     -	0002          	.loop:
    // 4:    7+4	0002  3C      	    inc a
    // 5:   11+10	0003  C30200  	    jp .loop


    mem.write(0x000,ROMCODE);

    // mem.heap[0x0000] = 0x3E;
    // mem.heap[0x0001] = 0x0A;
    // mem.heap[0x0002] = 0x3C;
    // mem.heap[0x0003] = 0xC3;
    // mem.heap[0x0004] = 0x02;
    // mem.heap[0x0005] = 0x00;

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
    ).ok().unwrap();

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

    // Setup SW spi pins
    let spi_rx  = pins.gpio16.into_pull_up_input();
    let spi_cs  = pins.gpio17.into_push_pull_output();
    let spi_clk = pins.gpio18.into_push_pull_output();
    let spi_tx  = pins.gpio19.into_push_pull_output();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);
    let mut per = Peripherals::new();


    let mut sw_spi = SwSpi::new(spi_cs, spi_clk, spi_tx, spi_rx);
    let sd_spi = peripherals::sd_spi::SdSpi::new();
 

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
    let mut strbuf = ArrayString::<4096>::new();
    writeln!(&mut strbuf, "Version = {} !\n", version_nb).unwrap();

    let mut said_hello = false;
    let mut redirect = false;

    let micro_seconds_per_frame: i64 = 1000;

    // 10 reads with each 8 clks
    // for _q in 0..10 {
    //     drop( sw_spi.read() );
    // }
    // delay.delay_us(15);

    // sw_spi.start();
    // delay.delay_us(20);
    // sw_spi.stop();
    


    //vec![ 0x]
    // for _q in 0..10000 {

    //     let mut blk_slice : sd_spi::BlkSlice = [0;512];
    //     let block = 0_u32;

    //     sd_spi.read_block(&mut sw_spi, block, &mut blk_slice);
    //     delay.delay_ms(100);
    // }

    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter() >= 2000_000_000 {
            said_hello = true;

            drop( serial.write(b"\r\n *** Welcome to PICO Rustzeighty v0.1d *** \r\n") );
            drop( serial.write(strbuf.as_bytes()));
        }

        while let Some(c) = per.get_out_char() {
            let b = [c];
            drop( serial.write(&b));
        }

        if  said_hello {
            mm += 1;
            
            let start = stopwatch_start( &timer );
            // run emulator for interval
            if let Some(cycles) = step_frame(micro_seconds_per_frame, &mut cpu, &mut per, &mut mem) {
                num_cycles +=  cycles;
            } else {
                //break;
            }     
            tt += stopwatch_delta(start, &timer);
           
            if mm > 500000 {
                strbuf.clear();
                writeln!(strbuf, "\r\n *** Time check :mm {} tt {} cycles {} \r\n", mm, tt, num_cycles).unwrap();
                drop( serial.write(strbuf.as_bytes()));

                mm = 0;
                tt = 0;
                num_cycles = 0;
            }
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
                    if redirect {
                        buf[0..count].into_iter().for_each(|c| {
                            per.add_in_char(*c);
                        });
                    } 
                    else {
                        // Convert to upper case
                        buf.iter_mut().take(count).for_each(|b| {
                            b.make_ascii_uppercase();
                        });

                        match buf[0] as char {
                            'A' => {
                                strbuf.clear();
                                writeln!(strbuf, "RESET!!\r\n").unwrap();
                                let res = sd_spi.reset(&mut sw_spi);
                                writeln!(strbuf, "RESET done {}\r\n", res).unwrap();
                                drop( serial.write(strbuf.as_bytes()));
                            }

                            'B' => {
                                strbuf.clear();
                                writeln!(strbuf, "START READ BLOCK !!\r\n").unwrap();

                                let mut blk_slice : sd_spi::BlkSlice = [0;512];
                                let block = 0_u32;

                                sd_spi.read_block(&mut sw_spi, block, &mut blk_slice);

                                unsafe {
                                    dump::dump( &blk_slice, 0, &mut strbuf);
                                }
                                drop( serial.write(strbuf.as_bytes()));

                                // let mut val = 0;
                                // sw_spi.start();

                                // for _j in 0..10 {
                                //     val =  sw_spi.read() ;
                                //     delay.delay_us(20);
                                // }

                                // sw_spi.stop();

                                // writeln!(strbuf, "10 reads 20 u interval, latest read value  {}\r\n", val).unwrap();
                                // drop( serial.write(strbuf.as_bytes()));
                            }
                            'C' => {
                                sw_spi.start();

                                delay.delay_us(200);

                                sw_spi.write(0x00) ;
                                delay.delay_us(100);

                                sw_spi.write(0x55) ;
                                delay.delay_us(100);

                                sw_spi.write(0xAA) ;
                                delay.delay_us(100);

                                sw_spi.write(0xFF) ;
                                delay.delay_us(100);

                                //     sw_spi.write(10 * j) ;
                                //     delay.delay_us(20);


                                // for j in 0..10 {
                                //     sw_spi.write(10 * j) ;
                                //     delay.delay_us(20);
                                // }
                                sw_spi.stop();

                                writeln!(strbuf, "10 write20 u interval\r\n").unwrap();
                                drop( serial.write(strbuf.as_bytes()));
                            }                            
                            'R' => {
                                drop( serial.write(b"Redirect input !!!!\r\n") );
                                redirect = true;
                            }
                            'D' => {
                                drop( serial.write(b"Delay check!\r\n") );
                                let start = stopwatch_start( &timer );
                                //per.delay_ms(100);
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

                            // 'R' => {
                            //     drop( serial.write(b"Restart \r\n") );
                            //     reset_to_usb_boot(0,2);    
                            // } 
                            _ => {
                                drop( serial.write(b"Unkown command \r\n") );
                                // strbuf.clear();
                                // unsafe { 
                                //     let ptr =  HEAP.as_ptr();
                                //     writeln!(strbuf, "HEAP Used: {} bytes, free: {} bytes HEAP {:?}",
                                //     ALLOCATOR.used(),
                                //     ALLOCATOR.free() ,
                                //                 ptr ).unwrap();
                                                
                                // }
                                // drop( serial.write(strbuf.as_bytes()));
                            }
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
