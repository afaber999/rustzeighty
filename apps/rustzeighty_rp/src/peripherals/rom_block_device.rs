//use std::{io::{Read, Write,Seek, SeekFrom}, fs::File, borrow::BorrowMut};


const ROM_BLK_SIZE : usize = 512;

#[derive(Debug)]
pub struct RomBlockDevice<'a> {
    active_blk : u32,
    rom        : &'a[u8],
    blk_ofs    : usize,
    rd_pos     : usize,
    wr_pos     : usize,
}

impl<'a> RomBlockDevice<'a> {
    pub fn new(rom : &'a[u8], blk_ofs : usize) -> Self {

        //println!("NEW ROM READONLY BLOCK DEVICE ####################");
        Self {
            active_blk: 0,
            rom,
            blk_ofs,
            rd_pos: 0,
            wr_pos: 0,
        }
    }

    pub fn init(&mut self) -> bool {
        self.rd_pos = 0;
        self.wr_pos = 0;
        self.active_blk = 0;
        true
    }
    pub fn get_blk(&mut self) -> bool {
        self.rd_pos = 0;
        true
    }
    
    pub fn set_blk3( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0x00FFFFFF) + ((val as u32) << 24); 
    }
    pub fn set_blk2( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0xFF00FFFF) + ((val as u32) << 16); 
    }
    pub fn set_blk1( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0xFFFF00FF) + ((val as u32) << 8); 
    }
    pub fn set_blk0( &mut self, val : u8 ) {
        self.active_blk = (self.active_blk & 0x00FFFF00) + ((val as u32)); 
    }
    
    pub fn get_next_byte(&mut self) -> u8 {
        let read_pos = self.rd_pos + ROM_BLK_SIZE * ( self.active_blk  as usize - self.blk_ofs);
        let val = self.rom[ read_pos];
        self.rd_pos = (self.rd_pos + 1 ) % self.rom.len();
        val
    }
    
    pub fn commit_blk(&mut self) -> bool {
        // ignore, ROM
        false
    }
    pub fn put_next_byte(&mut self, _val : u8) {
        // ignore, ROM
    }

    pub fn shutdown(&mut self) -> bool {
        false
    }
}