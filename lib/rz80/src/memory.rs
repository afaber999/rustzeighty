//use std::mem;
use crate::RegT;

const HEAP_SIZE: usize = 64*1024;


/// Very simple 64K memory
pub struct Memory {

    pub heap: [u8; HEAP_SIZE],
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}


impl Memory {
    /// return new, unmapped memory object
    pub fn new() -> Memory {
        Memory {
            heap: [0; HEAP_SIZE],
        }
    }

    /// read unsigned byte from 16-bit address
    #[inline(always)]
    pub fn r8(&self, addr: RegT) -> RegT {
        let uaddr = (addr & 0xFFFF) as usize;
        self.heap[uaddr]as RegT
    }

    /// read signed byte from 16-bit address
    #[inline(always)]
    pub fn rs8(&self, addr: RegT) -> RegT {
        let uaddr = (addr & 0xFFFF) as usize;
        self.heap[uaddr] as i8 as RegT
    }

    /// write unsigned byte to 16-bit address
    #[inline(always)]
    pub fn w8(&mut self, addr: RegT, val: RegT) {
        let uaddr = (addr & 0xFFFF) as usize;
        self.heap[uaddr] = val as u8;
    }

    /// write unsigned byte, ignore write-protection flag
    pub fn w8f(&mut self, addr: RegT, val: RegT) {
        let uaddr = (addr & 0xFFFF) as usize;
        self.heap[uaddr] = val as u8;
    }

    /// read unsigned word from 16-bit address
    #[inline(always)]
    pub fn r16(&self, addr: RegT) -> RegT {
        let l = self.r8(addr);
        let h = self.r8(addr + 1);
        h << 8 | l
    }

    /// write unsigned word to 16-bit address
    #[inline(always)]
    pub fn w16(&mut self, addr: RegT, val: RegT) {
        let l = val & 0xff;
        let h = (val >> 8) & 0xff;
        self.w8(addr, l);
        self.w8(addr + 1, h);
    }

    /// write a whole chunk of memory, ignore write-protection
    pub fn write(&mut self, addr: RegT, data: &[u8]) {
        let mut offset = 0;
        for b in data {
            self.w8f(addr + offset, *b as RegT);
            offset += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mem_readwrite() {
        let mut mem = Memory::new();
        mem.w8(0x1234, 0x12);
        assert_eq!(mem.r8(0x1234), 0x12);

        mem.w8(0x2345, 0x32);
        assert_eq!(mem.r8(0x2345), 0x32);

        mem.w16(0x1000, 0x1234);
        assert_eq!(mem.r16(0x1000), 0x1234);
        assert_eq!(mem.r8(0x1000), 0x34);
        assert_eq!(mem.r8(0x1001), 0x12);

        mem.w16(0xFFFF, 0x2233);
        assert_eq!(mem.r16(0xFFFF), 0x2233);
        assert_eq!(mem.r8(0xFFFF), 0x33);
        assert_eq!(mem.r8(0x0000), 0x22);
    }
}
