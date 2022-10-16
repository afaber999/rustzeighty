use std::{io::{Read, Write,Seek, SeekFrom}, fs::File, borrow::BorrowMut};

#[derive(Debug)]
pub struct FileBlockDevice {
    active_blk : u32,
    blk    : [u8;512],
    rd_pos : usize,
    wr_pos : usize,
    file   : Option<File>,
}

impl FileBlockDevice {
    pub fn new() -> Self {
        //println!("NEW FILEBLOCKDEVICE ####################");
        Self {
            active_blk: 0,
            blk: [0;512],
            rd_pos: 0,
            wr_pos: 0,
            file  : None,
        }
    }

    pub fn init(&mut self) -> bool {
        self.rd_pos = 0;
        self.wr_pos = 0;
        self.active_blk = 0;

        self.file = Some( std::fs::OpenOptions::new()
//            .create(true)
            .write(true)
            .read(true)
            .open("../../filesystem/sda.img")
            .unwrap() );

        if let Some(file) = self.file.borrow_mut() {

            file.seek(SeekFrom::Start(0)).expect("seek");
            file.read(&mut self.blk).expect("read");

            //file.seek(SeekFrom::Start(0)).expect("seek");
            //file.write(&mut self.blk).expect("write");
        } else {
            panic!("DISK FILE NOT MOUNTED! ");
        }
        true
    }
    pub fn get_blk(&mut self) -> bool {
        self.rd_pos = 0;
        let seek_pos = 512 * (self.active_blk as u64 as u64);
        //println!("Seek to blk {:04X} seek_pos {:04X} ", self.active_blk as u32, seek_pos as u32);
        if let Some(file) = self.file.borrow_mut() {
            file.seek(SeekFrom::Start(seek_pos)).expect("seek");
            file.read(&mut self.blk).expect("read");
        } else {
            panic!("DISK FILE NOT MOUNTED! ");
        }
        true
    }

    pub fn commit_blk(&mut self) -> bool {
        self.wr_pos = 0;
        let seek_pos = 512 * (self.active_blk as u64 as u64);
        //println!("commit_blk Seek to blk 0x{:04X} seek_pos 0x{:04X} ", self.active_blk as u32, seek_pos as u32);
        if let Some(file) = self.file.borrow_mut() {
            file.seek(SeekFrom::Start(seek_pos)).expect("seek");
            file.write(&mut self.blk).expect("read");
            file.sync_all().expect("sync");
        } else {
            panic!("DISK FILE NOT MOUNTED! ");
        }
        false
    }

    pub fn set_blk3( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0x00FFFFFF) + ((val as u32) << 24); 
        //println!("BLOCK ADDRESS SET TO {:04X}", self.active_blk);
    }
    pub fn set_blk2( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0xFF00FFFF) + ((val as u32) << 16); 
        //println!("BLOCK ADDRESS SET TO {:04X}", self.active_blk);
    }
    pub fn set_blk1( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0xFFFF00FF) + ((val as u32) << 8); 
        //println!("BLOCK ADDRESS SET TO {:04X}", self.active_blk);
    }
    pub fn set_blk0( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0x00FFFF00) + ((val as u32)); 
        //println!("BLOCK ADDRESS SET TO {:04X}", self.active_blk);
    }

    pub fn get_next_byte(&mut self) -> u8 {
        let val = self.blk[ self.rd_pos];
        self.rd_pos = (self.rd_pos + 1 ) % self.blk.len();
        val
    }
    pub fn put_next_byte(&mut self, val : u8) {
        self.blk[ self.wr_pos] = val;
        //println!("put_next_byte {:02X} {:02X} ", self.wr_pos, self.blk[ self.wr_pos]);
        self.wr_pos = (self.wr_pos + 1 ) % self.blk.len();
    }

    pub fn shutdown(&mut self) -> bool {
        false
    }
    pub fn dump_buffer(&self) {
        print!("------------- DUMP BLK DEVICE BUFFER -------------");
        for addr in 0..self.blk.len() {
            if addr % 16 == 0 {
                print!( "\n{:04X}:", addr );
            } else if addr % 8 == 0 {
                print!( " " );
            }
            print!( " {:02X}", self.blk[addr] );
        }
        println!("\n------------- END DUMP BLK DEVICE BUFFER -------------");
    }

}