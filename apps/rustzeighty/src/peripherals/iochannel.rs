

#[allow(unused_variables)]
pub trait IOChannel {
    fn add_in_char(&mut self, val: u8);
    fn get_out_char(&mut self) -> Option<u8>;
}
