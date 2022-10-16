
pub mod iochannel;
pub mod spi_interface;
pub mod sw_spi;
pub mod sd_spi;
pub mod rom_block_device;
use self::iochannel::IOChannel;

use rom_block_device::RomBlockDevice;
use z80_sys::ringbuffer::RingBuffer;
//use cortex_m::delay::Delay;

pub struct Peripherals<'a> {
    //delay           : Delay,
    last_serial_a   : u8,
    last_serial_b   : u8,    
    pub in_keys         : RingBuffer,
    pub out_chars       : RingBuffer,
    dbg_trace       : u8,
    rom_blk_device  : RomBlockDevice<'a>,
}

impl Peripherals<'_> {
//    pub fn new( delay: Delay) -> Self {
    pub fn new() -> Self {
            static ROMCODE: &'static [u8] = include_bytes!("../../../filesystem/cpm_all.img");

        Self {
            //delay,
            last_serial_a : 0,
            last_serial_b : 0,            
            in_keys : RingBuffer::new(),
            out_chars : RingBuffer::new(),
            dbg_trace : 0,
            rom_blk_device : RomBlockDevice::new(ROMCODE, 0x800),
        }
    }

    // pub fn delay_ms(&mut self, ms : u32) {
    //     self.delay.delay_ms(ms);
    // }
    pub fn get_dbg_trace(&self) -> u8 {
        self.dbg_trace
    }
}

impl IOChannel for Peripherals<'_> {
    fn add_in_char(&mut self, val: u8) {
        self.in_keys.push_back( val );        
    }

    fn get_out_char(&mut self) -> Option<u8> {

        let front = self.out_chars.pop_front();
        // if front.is_some() {
        //     println!(" D:{} {:?}", self.out_chars.len(), front);     
        // }
        front
    }
}

impl<'a> rz80::Bus for Peripherals<'a> {
    fn cpu_inp(&mut self, port: rz80::RegT) -> rz80::RegT {
        //println!("Read port 0x{:04X}   ", port);
        let mp = port & 0xFF;
        match mp {
            0x0030 => { 
                if let Some(c) = self.in_keys.pop_front() {
                    //println!("AFTER POP 0x0030 GOT LEN {} ", self.in_keys.len()); 
                    self.last_serial_a = c;
                }
                let ret = self.last_serial_a  as rz80::RegT;
                //println!("0x0030 GOT LEN {} {}", self.in_keys.len(), ret); 
                ret 
            },
            0x0031 => { 
                self.last_serial_b as rz80::RegT
            },
            0x0032 => {
                if self.in_keys.len() > 0 {
//                    println!("GOT LEN {} ", self.in_keys.len()); 
                    0b00000101
                } else {
                    0b00000100
                }
            },
            0x0033 => { 
                0b00000100
            },

             // RD BLK BYTE 
             0x00F1 => {
                let res = self.rom_blk_device.get_next_byte() as rz80::RegT;
                res
             },

            _ => {panic!("invalid read port 0x{:02X}  0x{:04X}", mp, port)},
        }
    }

    fn cpu_outp(&mut self, port: rz80::RegT, val: rz80::RegT) {
        let hi_port = (port >> 8 ) as u8; 
         // println!("Write port 0x{:04X} val 0x{:04X} ", port, val);
        match port & 0xFF {
            0x0000 => {
                self.dbg_trace = hi_port;                
                //println!("\r\n\r\n%%%%%%%%%%%%% TRACING SET TO {} ", self.dbg_trace);
                //std::thread::sleep(std::time::Duration::from_secs(3));
                self.dbg_trace = hi_port;
            }
            0x0030 => { 
                // println!( " C:{}", val as u8);
                self.out_chars.push_back(val as u8);
            },
            0x0031 => { 
            },
            0x0032 => { 
            },
            0x0033 => { 
            },

            // SD BLK DEVICE
            0x00F0 => {
                drop( self.rom_blk_device.init() );
            },

            // READ BLK DEVICE
            0x00F1 => {
                drop( self.rom_blk_device.get_blk());
            },
            // SET NEXT BYtE
            0x00F2 => {
                // CALL EMPTY FOR FOR ROM
                self.rom_blk_device.put_next_byte(hi_port);
            },
            // WRITEBLK DEVICE
            0x00F3 => {
                // CALL EMPTY FOR FOR ROM
                self.rom_blk_device.commit_blk();
            },

            // SD BLK B31..B24
            0x00FC => {
                self.rom_blk_device.set_blk3(hi_port);
            },

            // SD BLK B16..B23
            0x00FD => {
                self.rom_blk_device.set_blk2(hi_port);
            },

            // SD BLK B8..B15
            0x00FE => {
                self.rom_blk_device.set_blk1(hi_port);
            },

            // SD BLK B0..B7
            0x00FF => {
                self.rom_blk_device.set_blk0(hi_port);
            },

            _ => {
                //std::thread::sleep(std::time::Duration::from_secs(3));
                panic!("invalid write port 0x{:04X}", port)},
            }        
    }    

    fn irq(&self, _ctrl_id: usize, _vec: u8) {}

    fn irq_cpu(&self) {}

    fn irq_ack(&self) -> rz80::RegT {
        0
    }

    fn irq_reti(&self) {

    }
}
