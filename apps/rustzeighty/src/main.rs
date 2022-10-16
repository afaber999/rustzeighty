extern crate rz80;
extern crate getch;
extern crate time;

mod peripherals;
mod stdio_channel;
mod telnet;


use std::fs::File;
use std::io::Write;

use rz80::Memory;
use time::PreciseTime;
use stdio_channel::StdioChannel;

#[allow(unused_imports)]
use crate::telnet::TelnetChannel;
//use telnet::create_telnet_channel;

// Target CPU frequency in kHZ
const FREQ_KHZ: i64 = 2458;



pub fn step_frame(  
        micro_seconds: i64, 
        cpu : &mut rz80::CPU, 
        peripherals : &mut peripherals::Peripherals, 
        mem : &mut Memory, 
        trace_file:&mut File) -> Option<usize>{

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
        let dbg_trace = peripherals.get_dbg_trace();
        if dbg_trace>0 {
            println!(" PC:0x{:04X} f:{:08b} a:0x{:02X} bc:0x{:04X} de:0x{:04X} hl:0x{:04X} sp:0x{:04X}", 
            cpu.reg.pc() as u16, 
            cpu.reg.f()  as u8,
            cpu.reg.a() as u8,
            cpu.reg.bc() as u16,
            cpu.reg.de() as u16,
            cpu.reg.hl() as u16,
            cpu.reg.sp()  as u16);

        }

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

fn main() -> std::io::Result<()> {
    //static ROMCODE: &'static [u8] = include_bytes!("..\\..\\asm\\asm.bin");
    static ROMCODE: &'static [u8] = include_bytes!("..\\..\\..\\asm\\firmware\\firmware.bin");
    
    let mut num_cycles = 0;        

    // accept connections and process them serially
    println!("**************** WAITING FOR IO CHANNEL ");
    let mut channel = StdioChannel::new();
    //let mut channel = TelnetChannel::new();
    println!("**************** GOT CONNECTION ");

    let mut cpu = rz80::CPU::new();
    let mut mem = rz80::Memory::new();
    let mut peripherals = peripherals::Peripherals::new();    
    
    mem.write(0x0000, ROMCODE);
    cpu.reg.set_sp(0xF000);
    cpu.reg.set_pc(0x0000);
    
    let mut micro_seconds_per_frame: i64 = 0;
    
  
    let mut trace_file = File::create("trace.txt")?;
    writeln!( trace_file, "Starting tracer" ).expect("tracerr");

    loop {
        let start = PreciseTime::now();

        // match stdio_channel_in.try_recv() {
        //     Ok(key) => {
        //         if key == 3 {
        //             break;
        //         }
        //         peripherals.add_in_key(key);
        //     },
        //     Err(TryRecvError::Empty) =>{} ,
        //     Err(TryRecvError::Disconnected) => panic!("Stdio channel disconnected"),
        // }

        // match telnet_channel_in.try_recv() {
        //     Ok(key) => {
        //         if key == 3 {
        //             break;
        //         }
        //         peripherals.add_in_key(key);
        //     },
        //     Err(TryRecvError::Empty) =>{} ,
        //     Err(TryRecvError::Disconnected) => panic!("Telnet channel disconnected"),
        // }
        channel.process_input(&mut peripherals);


        
        // run emulator for interval
        if let Some(cycles) = step_frame(micro_seconds_per_frame, &mut cpu, &mut peripherals, &mut mem, &mut trace_file) {
            num_cycles +=  cycles;
        } else {
            break;
        }

        channel.process_output(&mut peripherals);

        // for now 60 times per second
        std::thread::sleep(std::time::Duration::from_millis(16));

        let frame_time = start.to(PreciseTime::now());
        micro_seconds_per_frame = frame_time.num_microseconds().unwrap();
    }

    println!("Done {}", num_cycles);

    Ok(())
}

