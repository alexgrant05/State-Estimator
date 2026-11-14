use std::fmt;
use std::fs::File;
#[cfg(target_os = "linux")]
use std::fs::OpenOptions;
use std::io;
use std::path::Path;
use std::sync::atomic::{compiler_fence, Ordering};

use se_abi::{
    ABI_HASH_HI, ABI_HASH_LO, ABI_MAJOR, CORE_MAGIC, REG_ABI_HASH_HI, REG_ABI_HASH_LO,
    REG_ABI_VERSION, REG_CORE_MAGIC,
};

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    Identity(&'static str),
    Bounds,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::Identity(s) => write!(f, "{s}"),
            Self::Bounds => write!(f, "register offset is out of range"),
        }
    }
}
impl std::error::Error for Error {}
impl From<io::Error> for Error {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

pub trait RegisterIo {
    fn read32(&self, offset: usize) -> Result<u32, Error>;
    fn write32(&mut self, offset: usize, value: u32) -> Result<(), Error>;
}

#[derive(Clone, Debug)]
pub struct MemoryRegisters {
    words: Vec<u32>,
}
impl MemoryRegisters {
    #[must_use]
    pub fn new(bytes: usize) -> Self {
        Self {
            words: vec![0; bytes / 4],
        }
    }
    pub fn set(&mut self, offset: usize, value: u32) -> Result<(), Error> {
        self.write32(offset, value)
    }
}
impl RegisterIo for MemoryRegisters {
    fn read32(&self, offset: usize) -> Result<u32, Error> {
        self.words
            .get(offset / 4)
            .copied()
            .filter(|_| offset.is_multiple_of(4))
            .ok_or(Error::Bounds)
    }
    fn write32(&mut self, offset: usize, value: u32) -> Result<(), Error> {
        let word = self
            .words
            .get_mut(offset / 4)
            .filter(|_| offset.is_multiple_of(4))
            .ok_or(Error::Bounds)?;
        *word = value;
        Ok(())
    }
}

pub struct UioRegisters {
    _file: File,
    pointer: *mut u32,
    bytes: usize,
}
unsafe impl Send for UioRegisters {}
impl UioRegisters {
    #[cfg(target_os = "linux")]
    pub fn open(path: impl AsRef<Path>, bytes: usize) -> Result<Self, Error> {
        use std::ffi::c_void;
        unsafe extern "C" {
            fn mmap(
                addr: *mut c_void,
                length: usize,
                prot: i32,
                flags: i32,
                fd: i32,
                offset: isize,
            ) -> *mut c_void;
        }
        use std::os::fd::AsRawFd;
        const PROT_READ: i32 = 1;
        const PROT_WRITE: i32 = 2;
        const MAP_SHARED: i32 = 1;
        let file = OpenOptions::new().read(true).write(true).open(path)?;
        let mapped = unsafe {
            mmap(
                std::ptr::null_mut(),
                bytes,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                file.as_raw_fd(),
                0,
            )
        };
        if mapped as isize == -1 {
            return Err(Error::Io(io::Error::last_os_error()));
        }
        Ok(Self {
            _file: file,
            pointer: mapped.cast(),
            bytes,
        })
    }
    #[cfg(not(target_os = "linux"))]
    pub fn open(_path: impl AsRef<Path>, _bytes: usize) -> Result<Self, Error> {
        Err(Error::Identity("UIO is available only on Linux"))
    }
}
impl Drop for UioRegisters {
    fn drop(&mut self) {
        #[cfg(target_os = "linux")]
        unsafe {
            unsafe extern "C" {
                fn munmap(addr: *mut std::ffi::c_void, length: usize) -> i32;
            }
            let _ = munmap(self.pointer.cast(), self.bytes);
        }
    }
}
impl RegisterIo for UioRegisters {
    fn read32(&self, offset: usize) -> Result<u32, Error> {
        if !offset.is_multiple_of(4) || offset + 4 > self.bytes {
            return Err(Error::Bounds);
        }
        compiler_fence(Ordering::SeqCst);
        let value = unsafe { std::ptr::read_volatile(self.pointer.add(offset / 4)) };
        compiler_fence(Ordering::SeqCst);
        Ok(value)
    }
    fn write32(&mut self, offset: usize, value: u32) -> Result<(), Error> {
        if !offset.is_multiple_of(4) || offset + 4 > self.bytes {
            return Err(Error::Bounds);
        }
        compiler_fence(Ordering::SeqCst);
        unsafe {
            std::ptr::write_volatile(self.pointer.add(offset / 4), value);
        }
        compiler_fence(Ordering::SeqCst);
        Ok(())
    }
}

pub struct Device<R: RegisterIo> {
    registers: R,
}
impl<R: RegisterIo> Device<R> {
    pub fn new(registers: R) -> Self {
        Self { registers }
    }
    pub fn identify(&self) -> Result<Identity, Error> {
        if self.registers.read32(REG_CORE_MAGIC)? != CORE_MAGIC as u32 {
            return Err(Error::Identity("CORE_MAGIC mismatch; writes refused"));
        }
        let version = self.registers.read32(REG_ABI_VERSION)?;
        if (version >> 16) as u16 != ABI_MAJOR {
            return Err(Error::Identity("ABI major mismatch; writes refused"));
        }
        if self.registers.read32(REG_ABI_HASH_LO)? != ABI_HASH_LO
            || self.registers.read32(REG_ABI_HASH_HI)? != ABI_HASH_HI
        {
            return Err(Error::Identity("ABI hash mismatch; writes refused"));
        }
        Ok(Identity {
            abi_major: (version >> 16) as u16,
            abi_minor: version as u16,
            hash_lo: ABI_HASH_LO,
            hash_hi: ABI_HASH_HI,
        })
    }
    pub fn registers(&self) -> &R {
        &self.registers
    }
    pub fn registers_mut(&mut self) -> &mut R {
        &mut self.registers
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Identity {
    pub abi_major: u16,
    pub abi_minor: u16,
    pub hash_lo: u32,
    pub hash_hi: u32,
}

#[cfg(target_os = "linux")]
pub mod dma {
    use super::Error;
    use std::ffi::{c_int, c_ulong, c_void};
    use std::fs::{File, OpenOptions};
    use std::os::fd::AsRawFd;
    use std::path::Path;
    use std::slice;

    const IOC_WRITE: u64 = 1;
    const IOC_READ: u64 = 2;
    const TYPE: u64 = 0x53;
    const fn ioctl_number(direction: u64, number: u64, bytes: u64) -> u64 {
        (direction << 30) | (bytes << 16) | (TYPE << 8) | number
    }
    const GET_RING_INFO: u64 = ioctl_number(IOC_READ, 0x00, 32);
    const SET_CONSUMER: u64 = ioctl_number(IOC_WRITE, 0x01, 4);
    const START: u64 = ioctl_number(0, 0x02, 0);
    const STOP: u64 = ioctl_number(0, 0x03, 0);
    const FLUSH: u64 = ioctl_number(0, 0x04, 0);
    const SOFT_RESET: u64 = ioctl_number(0, 0x05, 0);

    unsafe extern "C" {
        fn ioctl(fd: c_int, request: c_ulong, ...) -> c_int;
        fn mmap(
            address: *mut c_void,
            length: usize,
            protection: c_int,
            flags: c_int,
            fd: c_int,
            offset: isize,
        ) -> *mut c_void;
        fn munmap(address: *mut c_void, length: usize) -> c_int;
    }

    #[repr(C)]
    #[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
    pub struct RingInfo {
        pub block_bytes: u32,
        pub block_count: u32,
        pub producer_block: u32,
        pub consumer_block: u32,
        pub host_overruns: u64,
        pub reset_generation: u32,
        pub reserved: u32,
    }

    pub struct DmaDevice {
        file: File,
    }

    impl DmaDevice {
        pub fn open(path: impl AsRef<Path>) -> Result<Self, Error> {
            Ok(Self {
                file: OpenOptions::new().read(true).write(true).open(path)?,
            })
        }

        fn command(&self, request: u64) -> Result<(), Error> {
            let result = unsafe { ioctl(self.file.as_raw_fd(), request as c_ulong) };
            if result < 0 {
                return Err(Error::Io(std::io::Error::last_os_error()));
            }
            Ok(())
        }

        pub fn ring_info(&self) -> Result<RingInfo, Error> {
            let mut info = RingInfo::default();
            let result = unsafe {
                ioctl(
                    self.file.as_raw_fd(),
                    GET_RING_INFO as c_ulong,
                    &mut info as *mut RingInfo,
                )
            };
            if result < 0 {
                return Err(Error::Io(std::io::Error::last_os_error()));
            }
            if info.reserved != 0 {
                return Err(Error::Identity(
                    "kernel ring metadata reserved field is nonzero",
                ));
            }
            Ok(info)
        }

        pub fn set_consumer(&self, block: u32) -> Result<(), Error> {
            let result = unsafe {
                ioctl(
                    self.file.as_raw_fd(),
                    SET_CONSUMER as c_ulong,
                    &block as *const u32,
                )
            };
            if result < 0 {
                return Err(Error::Io(std::io::Error::last_os_error()));
            }
            Ok(())
        }

        pub fn start(&self) -> Result<(), Error> {
            self.command(START)
        }
        pub fn stop(&self) -> Result<(), Error> {
            self.command(STOP)
        }
        pub fn flush(&self) -> Result<(), Error> {
            self.command(FLUSH)
        }
        pub fn soft_reset(&self) -> Result<(), Error> {
            self.command(SOFT_RESET)
        }

        pub fn map_ring(&self, bytes: usize) -> Result<ReadOnlyRing, Error> {
            let pointer =
                unsafe { mmap(std::ptr::null_mut(), bytes, 1, 1, self.file.as_raw_fd(), 0) };
            if pointer as isize == -1 {
                return Err(Error::Io(std::io::Error::last_os_error()));
            }
            Ok(ReadOnlyRing { pointer, bytes })
        }
    }

    pub struct ReadOnlyRing {
        pointer: *mut c_void,
        bytes: usize,
    }
    unsafe impl Send for ReadOnlyRing {}
    impl ReadOnlyRing {
        pub fn bytes(&self) -> &[u8] {
            unsafe { slice::from_raw_parts(self.pointer.cast(), self.bytes) }
        }
    }
    impl Drop for ReadOnlyRing {
        fn drop(&mut self) {
            unsafe {
                let _ = munmap(self.pointer, self.bytes);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use se_abi::*;
    #[test]
    fn identify_accepts_matching_core() {
        let mut r = MemoryRegisters::new(65536);
        r.set(REG_CORE_MAGIC, CORE_MAGIC as u32).unwrap();
        r.set(
            REG_ABI_VERSION,
            u32::from(ABI_MAJOR) << 16 | u32::from(ABI_MINOR),
        )
        .unwrap();
        r.set(REG_ABI_HASH_LO, ABI_HASH_LO).unwrap();
        r.set(REG_ABI_HASH_HI, ABI_HASH_HI).unwrap();
        assert!(Device::new(r).identify().is_ok());
    }
    #[test]
    fn mismatch_refuses_identity() {
        assert!(Device::new(MemoryRegisters::new(65536)).identify().is_err());
    }
}
