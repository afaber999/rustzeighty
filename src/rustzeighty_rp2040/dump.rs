#![allow(unused)]

use arrayvec::ArrayString;
use core::fmt::Write;


/// Simple hex dump of slice
///
/// # Panics
///
/// Panics if .
pub fn dump(data : &[u8], base: u32, str : &mut ArrayString::<500>) {

    for (i, val) in data.iter().enumerate() {
        if (i %  16) == 0 {
            write!(str, "{:08X} ", base + i as u32 ).unwrap();
        }
        if (i % 8 ) == 0 {
            write!(str, " " ).unwrap()
        }
        
        write!(str, " {:02X}", val).unwrap();

        if (i %  16) == 15 {        
            writeln!(str).unwrap();
        }
    }
    writeln!(str).unwrap();
}