use super::spi_interface::SpiInterface;
//use alloc::vec;
use core::arch::asm;

pub struct SdSpi {

}

type ResponseR1 = u8;
type ResponseR7 = [u8;5];
pub type BlkSlice   = [u8;512];

impl SdSpi {
    pub fn new() -> Self {
        Self {
            
        }
    }

    pub fn is_valid_r1_response(response: ResponseR1 ) -> bool {
        (response & 0b_1000_0000) == 0
    }

    pub fn is_valid_r7_response(response: ResponseR7 ) -> bool {
        SdSpi::is_valid_r1_response(response[0])
    }


    // ############################################################################
    // NOTE: Response message formats in SPI mode are different than in SD mode.
    //
    // Read bytes until we find one with MSB = 0 or bail out retrying 
    // after 1000 times, then return 0xFF
    // Return last read byte otherwise 0xFF
    // ############################################################################     
    pub fn wait_for_r1_response(&self, spi: &mut dyn SpiInterface ) -> ResponseR1 {
        for _ in 0..64 {
            let response = spi.read() ;
            if SdSpi::is_valid_r1_response(response) {
                return response;
            }
        }
        0xFF
    }

    // ############################################################################
    // Read bytes until we find one with MSB = 0, then read the next 4 bytes
    // (so 5 bytes total), or bail out retrying  after 1000 times, then return 0xFF
    // Returns a u8 slice  with 5 bytes
    // ############################################################################     
    pub fn wait_for_r7_response(&self, spi: &mut dyn SpiInterface ) -> ResponseR7 { 
        [
            self.wait_for_r1_response(spi),
            spi.read(),
            spi.read(),
            spi.read(),
            spi.read(),
    
        ]
    }

    // writes a slice of bytes to the SPI device
    pub fn write_slice(&self, spi: &mut dyn SpiInterface, slice : &[u8]  ) {
        for b in slice {
            spi.write(*b);
        }
    }
     
    // ############################################################################
    // Send a CMD0 (GO_IDLE) message and read an R1 response.
    //
    // CMD0 will 
    // 1) Establish the card protocol as SPI (if has just powered up.)
    // 2) Tell the card the voltage at which we are running it.
    // 3) Enter the IDLE state.
    //
    // returns true if we received the correct response
    // ############################################################################
    pub fn cmd_0( &self, spi: &mut dyn SpiInterface ) -> bool{
        let mut result = false;
        spi.start();

        self.write_slice(spi, &[ 0 | 0x40, 0x00, 0x00, 0x00, 0x00, 0x94 | 0x01 ]);

        if self.wait_for_r1_response(spi) != 0xFF {
            result = true;
        }

        spi.stop();
        result
    }

    // ############################################################################
    // Send a CMD8 (SEND_IF_COND) message and read an R7 response.
    //
    // Establish that we are squawking V2.0 of spec & tell the SD 
    // card the operating voltage is 3.3V.  The reply to CMD8 should 
    // be to confirm that 3.3V is OK and must echo the 0xAA back as 
    // an extra confirm that the command has been processed properly. 
    // The 0x01 in the byte before the 0xAA in the command buffer 
    // below is the flag for 2.7-3.6V operation.
    //
    // Establishing V2.0 of the SD spec enables the HCS bit in 
    // ACMD41 and CCS bit in CMD58.
    //
    // Return the 5-byte response in u8 slice
    // The response should be: 0x01 0x00 0x00 0x01 0xAA.
    // ############################################################################
    pub fn cmd_8( &self, spi: &mut dyn SpiInterface ) -> bool {
        let mut result = false;
        spi.start();

        self.write_slice(spi, &[ 8 | 0x40, 0x00, 0x00, 0x01, 0xAA, 0x86 | 0x01 ]);

        let resp = self.wait_for_r7_response(spi); 
        if  resp[0] != 0x00 {
            result = true;
        }

        spi.stop();
        result
    }


    // ############################################################################
    //  Send a CMD58 message and read an R3 response.
    //  CMD58 is used to ask the card what voltages it supports and
    //  if it is an SDHC/SDXC card or not.
    //  Returns the 5-byte response
    // ############################################################################
    pub fn cmd_58( &self, spi: &mut dyn SpiInterface ) -> ResponseR7 {
        spi.start();
        self.write_slice(spi, &[ 58 | 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 | 0x01 ]);
        let response = self.wait_for_r7_response(spi);
        spi.stop();
        response
    }


    // ############################################################################
    // Send a CMD55 (APP_CMD) message and read an R1 response.
    // CMD55 is used to notify the card that the following message is an ACMD 
    // (as opposed to a regular CMD.)
    // Returns the 1-byte response
    // ############################################################################
    pub fn cmd_55( &self, spi: &mut dyn SpiInterface ) -> ResponseR1 {
        spi.start();
        self.write_slice(spi, &[ 55 | 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 | 0x01 ]);
        let response = self.wait_for_r1_response(spi);
        spi.stop();
        response
    }

    // ############################################################################
    //  Send a ACMD41 (SD_SEND_OP_COND) message and return an R1 response byte in A.
    // 
    //  The main purpose of ACMD41 to set the SD card state to READY so
    //  that data blocks may be read and written.  It can fail if the card
    //  is not happy with the operating voltage.
    // 
    //  Note that A-commands are prefixed with a CMD55.
    // ############################################################################    
    pub fn cmd_acmd41( &self, spi: &mut dyn SpiInterface ) -> ResponseR1 {

        let mut response = 0xFF;

        if SdSpi::is_valid_r1_response(self.cmd_55(spi)) {
            spi.start();

            // SD spec p263 Fig 7.1 footnote 1 says we want to set the HCS bit here for HC/XC cards.
            // Notes on Internet about setting the supply voltage in ACMD41. But not in SPI mode?
            // The folowing works on my Samsung SDHC cards:
            self.write_slice(spi, &[ 41 | 0x40, 0x40, 0x00, 0x00, 0x00, 0x00 | 0x01 ]);
            response = self.wait_for_r1_response(spi);
            spi.stop();
        }
        response
    }

    pub fn reset(&self, spi: &mut dyn SpiInterface ) -> bool{

        // wake up SD card by sending 80 clks
        for _ in 0..10 {
            drop( spi.read());
        }

        drop( self.cmd_0( spi));
        drop( self.cmd_8( spi));

        for __q in 0..80 {
            let val = self.cmd_acmd41( spi);
            if val == 0 {
                drop( self.cmd_58( spi));
                return true;
            }
            self.pulse_wait();
        }

        false
    }


    // ############################################################################
    // CMD17 (READ_SINGLE_BLOCK)
    //
    //  Read one block given by the 32-bit (little endian) number at 
    //  the top of the stack into the buffer given by address
    // 
    //  - set SSEL = true
    //  - send command 17 with block address
    //  - read for CMD ACK (R1 response)
    //  - wait for 'data token'
    //  - read data block
    //  - read data CRC
    //  - set SSEL = false
    // 
    // ############################################################################

    fn pulse_wait(&self) {
        unsafe {
            asm!(
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                "nop","nop","nop","nop","nop","nop","nop","nop","nop",
            );
        }        
    }

    
    pub fn read_block(&self, spi: &mut dyn SpiInterface, block : u32, blk_slice : &mut BlkSlice ) -> bool{
        
        let mut result = false;

        spi.start();

        // issue cmd 17, address in big endian
        self.write_slice(   spi, 
                            &[  17 | 0x40,
                                (block >> 24) as u8, 
                                (block >> 16) as u8,
                                (block >> 8 ) as u8,
                                (block      ) as u8,
                                0x00 | 0x01 ] );

        let response = self.wait_for_r1_response(spi);

        if response == 0x00 {

            // wait for data read
            for _q in 0..1000 {
                let val = spi.read();

                if val != 0xFF {
                    // check if this is the data token marker
                    if val == 0xFE {
                        for idx in 0..blk_slice.len() {
                            blk_slice[idx] = spi.read();
                        }
                        // drop CRC
                        let _crc = spi.read();
                        result = true;
                    }
                    break;
                }
            }
        }

        spi.stop();

        result
    }
}
