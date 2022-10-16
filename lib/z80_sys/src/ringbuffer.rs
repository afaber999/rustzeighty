

pub struct RingBuffer  {
    buf    : [u8;128],
    rd_idx : usize,
    wr_idx : usize,
    len    : usize,
}


impl RingBuffer {

    pub fn new() -> Self {
        Self {
            buf : [ 0; 128],
            rd_idx: 0,
            wr_idx: 0,
            len   : 0,
        }
    }

    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn free(&self) -> usize {
        self.capacity() - self.len
    }

    pub fn can_push(&self) -> bool {
        self.free() > 0
    }

    pub fn can_pop(&self) -> bool {
        self.len > 0
    }

    pub fn push_back(&mut self, c: u8) {
        if self.can_push() {
            self.len += 1;
            self.buf[self.wr_idx] = c;
            self.wr_idx = (self.wr_idx + 1 ) % self.buf.len();
        }
    }

    pub fn pop_front(&mut self) -> Option<u8>{
        if self.can_pop() {
            let c = self.buf[self.rd_idx];
            self.rd_idx = (self.rd_idx + 1 ) % self.buf.len();
            self.len -= 1;
            return Some(c)
        }
        None
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new() {
        let mut rb = RingBuffer::new();
        assert_eq!(rb.len(), 0);
        assert_eq!(rb.free(), rb.capacity()  );
        assert_eq!(rb.can_pop(), false);
        assert_eq!(rb.can_push(), true);
        assert_eq!(rb.pop_front(), None);
    }

    #[test]
    fn push_pop_one() {
        let mut rb = RingBuffer::new();
        rb.push_back(0xAA);
        assert_eq!(rb.free(), rb.capacity()-1);
        assert_eq!(rb.can_pop(), true);
        assert_eq!(rb.can_push(), true);
        assert_eq!(rb.len(), 1);

        assert_eq!(rb.pop_front(), Some(0xAA));
        assert_eq!(rb.free(), rb.capacity());
        assert_eq!(rb.can_pop(), false);
        assert_eq!(rb.can_push(), true);
        assert_eq!(rb.len(), 0);
    }


    fn fill_and_empty(rb : &mut RingBuffer) {
        let mut cnt = 0;
        while rb.can_push() {
            rb.push_back( (cnt & 0xFF ) as u8);
            cnt += 1;
        }
        assert_eq!(rb.free(), 0);
        assert_eq!(rb.len(), cnt);
        assert_eq!(cnt , rb.capacity());

        // for i in 0..rb.capacity() {
        //     if i % 16 == 0 {
        //         println!();
        //     }
        //     print!("{:02X} ", rb.buf[i]);
        // }

        cnt = 0;
        while rb.can_pop() {           
            assert_eq!( rb.pop_front(), Some( (cnt & 0xFF) as u8));
            assert_eq!( rb.can_push(), true);           
            cnt += 1;
        }

        assert_eq!( cnt, rb.capacity() );

    }

    #[test]
    fn push_max() {
        let mut rb = RingBuffer::new();
        fill_and_empty(&mut rb);
    }

    #[test]
    fn push_all_pos() {
        let mut rb = RingBuffer::new();

        let loops = rb.capacity() * 3 / 2;
        for _ofs in 0..loops {
            rb.push_back(0x00);
            drop( rb.pop_front() );
            assert_eq!(rb.free(), rb.capacity());            
            fill_and_empty(&mut rb);
        }
    }

}
