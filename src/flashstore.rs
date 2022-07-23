

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
