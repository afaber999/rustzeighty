#![no_std]
#![no_main]

mod dump;

use rp_pico::{entry, hal::rom_data::{rom_version_number, reset_to_usb_boot}};
use panic_halt as _;
use rp_pico::hal::pac;
use rp_pico::hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use core::{fmt::Write, slice};
use arrayvec::ArrayString;

use embedded_hal::digital::v2::OutputPin;

const FLASH_BASE : u32= 0x1000_0000;
const COUNTER_OFFSET : u32= 0x0010_0000;
const COUNTER_ADDRESS : u32= FLASH_BASE + COUNTER_OFFSET; 


#[entry]
fn main() -> ! {

    let mut pac = pac::Peripherals::take().unwrap();

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
    

    flash_experiment();

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
                        unsafe {
                            let counter = COUNTER_ADDRESS as *mut u32;
                            let counter_val = *counter;    

                            strbuf.clear();
                            writeln!(strbuf,"Counter value {}", counter_val).unwrap();
                            //dump::dump( slice::from_raw_parts(COUNTER_ADDRESS as *const u8 , 4), 0, &mut strbuf);
                        }
                        drop( serial.write(b"FLASH experiment\r\n") );
                        drop( serial.write(strbuf.as_bytes()));
                        flash_experiment();
                        drop( serial.write(b"FLASH experiment finished \r\n") );
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


#[inline(never)]
//#[link_section = ".data.code"]
#[link_section = ".data.ram_func"]
fn flash_experiment( ) {

    unsafe {
        let mut data  = [0;256];
        let counter = COUNTER_ADDRESS as *mut u32;
        let counter_val = *counter;    
    
        *(data.as_mut_ptr() as *mut u32) = counter_val + 1;

        let connect_internal_flash : extern "C" fn() = rom_table_lookup( *b"IF" );
        let flash_exit_cmd_xip : extern "C" fn() = rom_table_lookup( *b"EX" );
        let flash_range_erase : extern "C" fn(u32, usize, u32, u8) = rom_table_lookup( *b"RE");
        let flash_range_program : extern "C" fn(u32, *const u8, usize ) = rom_table_lookup( *b"RP" );
        let flash_flush_cache : extern "C" fn() = rom_table_lookup( *b"FC" );
        let flash_enter_cmd_xip : extern "C" fn() = rom_table_lookup( *b"CX" );

        connect_internal_flash();
        flash_exit_cmd_xip();
        flash_range_erase( COUNTER_OFFSET, 1<<12,1<<16,0xD8);
        flash_range_program( COUNTER_OFFSET, data.as_ptr(), data.len());
        flash_flush_cache();
        flash_enter_cmd_xip();    
    }
}


/// Pointer to helper functions lookup table.
const FUNC_TABLE: *const u16 = 0x0000_0014 as _;
/// The following addresses are described at `2.8.2. Bootrom Contents`
/// Pointer to the lookup table function supplied by the rom.
const ROM_TABLE_LOOKUP_PTR: *const u16 = 0x0000_0018 as _;


/// This function searches for (table)
type RomTableLookupFn<T> = unsafe extern "C" fn(*const u16, u32) -> T;

/// A bootrom function table code.
pub type RomFnTableCode = [u8; 2];


/// Retrive rom content from a table using a code.
#[inline(always)]
fn rom_table_lookup<T>(tag: RomFnTableCode) -> T {
    unsafe {
        let rom_table_lookup_ptr: *const u32 = rom_hword_as_ptr(ROM_TABLE_LOOKUP_PTR);
        let rom_table_lookup: RomTableLookupFn<T> = core::mem::transmute(rom_table_lookup_ptr);
        rom_table_lookup(
            rom_hword_as_ptr(FUNC_TABLE) as *const u16,
            u16::from_le_bytes(tag) as u32,
        )
    }
}

/// To save space, the ROM likes to store memory pointers (which are 32-bit on
/// the Cortex-M0+) using only the bottom 16-bits. The assumption is that the
/// values they point at live in the first 64 KiB of ROM, and the ROM is mapped
/// to address `0x0000_0000` and so 16-bits are always sufficient.
///
/// This functions grabs a 16-bit value from ROM and expands it out to a full 32-bit pointer.
#[inline(always)]
unsafe fn rom_hword_as_ptr(rom_address: *const u16) -> *const u32 {
    let ptr: u16 = *rom_address;
    ptr as *const u32
}
