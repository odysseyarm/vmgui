//! Convert a UDP socket to a Read/Write stream

use std::{io::{Read, Write}, net::UdpSocket};

/// Adapter for using a UDP socket as a byte stream.
///
/// Naively converts a UDP socket to a byte stream. The socket must be connected before using this
/// adapter. There is no logic to handle backpressure, packet reordering, dropped packets, etc.
pub struct UdpStream {
    socket: UdpSocket,
    read_buf: Box<[u8]>,
    write_buf: Box<[u8]>,
    read_pos: usize,
    read_len: usize,
    write_pos: usize,
    read_prefix: usize,
    write_prefix: usize,
}

impl UdpStream {
    /// Create the adapter with the specified read and write capacities. The max datagram size that
    /// can be received and sent is dictated by the read and write capacities respectively.
    ///
    /// `read_prefix` can be used to ignore some number of bytes at the start of each datagram.
    pub fn with_capacity(
        socket: UdpSocket,
        read_capacity: usize,
        write_capacity: usize,
        read_prefix: usize,
        write_prefix: &[u8],
    ) -> Self {
        let mut vec = vec![0; write_capacity];
        vec[..write_prefix.len()].copy_from_slice(write_prefix);
        UdpStream {
            socket,
            read_buf: vec![0; read_capacity].into(),
            write_buf: vec.into(),
            read_pos: 0,
            read_len: 0,
            write_pos: write_prefix.len(),
            read_prefix,
            write_prefix: write_prefix.len(),
        }
    }
}

impl Read for UdpStream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.read_len == 0 {
            self.read_len = self.socket.recv(&mut self.read_buf)?;
            self.read_pos = self.read_prefix;
        };

        let consume = std::cmp::min(buf.len(), self.read_len - self.read_pos);
        buf[..consume].copy_from_slice(&self.read_buf[self.read_pos..][..consume]);
        self.read_pos += consume;
        if self.read_pos == self.read_len {
            self.read_len = 0;
        }
        Ok(consume)
    }
}

impl Write for UdpStream {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let mut consume = std::cmp::min(buf.len(), self.write_buf.len() - self.write_pos);
        if consume == 0 {
            self.flush()?;
            consume = std::cmp::min(buf.len(), self.write_buf.len());
        }
        self.write_buf[self.write_pos..][..consume].copy_from_slice(&buf[..consume]);
        self.write_pos += consume;
        Ok(consume)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.socket.send(&self.write_buf[..self.write_pos])?;
        self.write_pos = self.write_prefix;
        Ok(())
    }
}
