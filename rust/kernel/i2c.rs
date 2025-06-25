use core::ptr::NonNull;

use crate::{
    device::property::FwNode,
    error,
    str::CStr,
    types::{ARef, AlwaysRefCounted, Opaque},
};

pub const I2C_CLIENT_SLAVE: u16 = bindings::I2C_CLIENT_SLAVE as u16;

#[repr(transparent)]
pub struct I2cBoardInfo(Opaque<bindings::i2c_board_info>);

impl I2cBoardInfo {
    pub fn new(board_type: &CStr, addr: u16, flags: u16) -> Self {
        let temp: Opaque<bindings::i2c_board_info> = Opaque::zeroed();
        let type_len = core::cmp::min(board_type.len(), unsafe { (*temp.get()).type_.len() });

        unsafe {
            (*temp.get()).addr = addr;
            (*temp.get()).type_[..type_len].copy_from_slice(&board_type.as_bytes()[..type_len]);
            (*temp.get()).flags = flags;
        }

        Self(temp)
    }

    pub const fn as_raw(&self) -> *mut bindings::i2c_board_info {
        self.0.get()
    }
}

#[repr(transparent)]
pub struct I2cAdapter(NonNull<bindings::i2c_adapter>);

impl I2cAdapter {
    pub fn i2c_get_adapter_by_fwnode(node: &FwNode) -> error::Result<Self> {
        let ptr = unsafe { bindings::i2c_get_adapter_by_fwnode(node.as_raw()) };
        match NonNull::new(ptr) {
            Some(x) => Ok(Self(x)),
            None => Err(error::code::ENODEV),
        }
    }

    pub const fn as_raw(&self) -> *mut bindings::i2c_adapter {
        self.0.as_ptr()
    }

    pub fn new_client_device(&self, board_info: &I2cBoardInfo) -> error::Result<I2cClient> {
        let client = error::from_err_ptr(unsafe {
            bindings::i2c_new_client_device(self.as_raw(), board_info.as_raw())
        })?;

        Ok(I2cClient(NonNull::new(client).unwrap()))
    }
}

impl Drop for I2cAdapter {
    fn drop(&mut self) {
        unsafe { bindings::i2c_put_adapter(self.as_raw()) }
    }
}

#[repr(transparent)]
pub struct I2cClient(NonNull<bindings::i2c_client>);

impl I2cClient {
    pub const fn as_raw(&self) -> *mut bindings::i2c_client {
        self.0.as_ptr()
    }

    pub fn device(&self) -> &kernel::device::Device {
        unsafe { kernel::device::Device::from_raw(&mut (*self.as_raw()).dev) }
    }
}

impl Drop for I2cClient {
    fn drop(&mut self) {
        unsafe {
            bindings::i2c_unregister_device(self.as_raw());
        }
    }
}
