// use alloc::rc::Rc;


// #[derive()]
// pub struct RingBuffer {
//     mem : [u8;1024],
//     head: usize,
//     tail: usize,
// }

// #[derive()]
// pub struct RingBufferReader {
//     buffer: Rc<RingBuffer>
// }

// pub struct RingBufferWriter {
//     buffer: Rc<RingBuffer>
// }

// impl RingBuffer {
//     pub fn split() -> (RingBufferReader, RingBufferWriter) {
//         let buffer = Rc::new(self);
//         let reader = RingBufferReader {
//             buffer : buffer.clone(),
//         };
//         let writer = RingBufferWriter {
//             buffer : buffer.clone(),
//         }
//         (reader,writer)
//     }
// }