use core::arch::asm;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;
use super::spi_interface::SpiInterface;

// SD cards operate on SPI mode 0
// Data changes on falling CLK edge & sampled on rising CLK edge
#[derive(Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SwSpi<A,B,C,D>
{
    pin_cs  : A,
    pin_clk : B,
    pin_tx  : C,
    pin_rx  : D,
}

impl<A,B,C,D> SwSpi<A,B,C,D>
where   A : OutputPin,
        B : OutputPin, 
        C : OutputPin, 
        D : InputPin, 
 {
    pub fn new( pin_cs : A, pin_clk : B, pin_tx : C, pin_rx : D) -> Self {

        let mut bb = Self {
            pin_cs,
            pin_clk,
            pin_tx,
            pin_rx,
        };

        drop( bb.pin_cs.set_high() );
        drop( bb.pin_clk.set_high() );
        drop( bb.pin_tx.set_high() );
        bb
    }

    fn pulse_wait(&self) {
        for _ in 0..100 {
            unsafe {
                asm!(
                    "nop","nop","nop","nop","nop","nop","nop","nop","nop",
                );
            }        
        }
    }
 }


 impl<A,B,C,D> SpiInterface for SwSpi<A,B,C,D>
    where   A : OutputPin,
            B : OutputPin, 
            C : OutputPin, 
            D : InputPin, 
{
    // Start SPI session with a device
    fn start(&mut self) {

        // wake up, so send 8 clock pulses
        drop(self.read());
        self.pulse_wait();

        // MOSI = 1, CLK = 0
        drop( self.pin_tx.set_high() );
        drop( self.pin_clk.set_low() );
        self.pulse_wait();

        // SEL = 0
        drop( self.pin_cs.set_low() );
        self.pulse_wait();
        
        // trigger another 8 clock pulses after selection
        drop(self.read());
    }

    // Stop the SPI session with a device
    fn stop(&mut self) {
        // wake up, so send 8 clock pulses
        drop(self.read());
        self.pulse_wait();

        // MOSI = 1, CLK = 0
        drop( self.pin_tx.set_high() );
        drop( self.pin_clk.set_low() );
        self.pulse_wait();

        // SEL = 1 (DISABLE)
        drop( self.pin_cs.set_high() );
        self.pulse_wait();

        // trigger another 8 clock pulses after selection
        drop(self.read());
        drop(self.read());
    }

    // write a byte value to the device
    // clock 8 bits, MSB is first, LSB is last
    fn write(&mut self, val : u8) {

        let mut shift_val = val;

        for _ in 0..8 {
            drop( self.pin_clk.set_low() );

            if shift_val & 0b10000000 == 0 {
                drop( self.pin_tx.set_low() );
            } else {
                drop( self.pin_tx.set_high() );
            }

            // shift for next value
            shift_val = shift_val << 1;

            self.pulse_wait();
            drop( self.pin_clk.set_high() );
            self.pulse_wait();
        }

        // Make sure we are setting TX to high after write
        drop( self.pin_tx.set_high() ); 
    }

    // Read 8 bits from the SPI & return value
    // pin_tx will be set to 1 during all bit transfers.
    // On exit : pin_clk =1, pin_tx = 1
    // MSB is first
    // LST is last
    fn read(&mut self) -> u8 {

        let mut val = 0;
        drop( self.pin_tx.set_high() );

        for _ in 0..8 {

            drop( self.pin_clk.set_low() );
            self.pulse_wait();
            drop( self.pin_clk.set_high() );
            self.pulse_wait();

            val = val << 1;

            if let Ok(v) = self.pin_rx.is_high() {
                if v {
                    val |= 0b00000001u8;
                }
            //if self.pin_rx.is_high().is_ok_and(|&x: &bool| x) {
            }
        }
        val
    }
}

