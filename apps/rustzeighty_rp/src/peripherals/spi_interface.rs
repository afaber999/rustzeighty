

pub trait SpiInterface {
    fn start(&mut self);
    fn stop(&mut self);
    fn write(&mut self, val : u8 );
    fn read(&mut self ) -> u8;
}

