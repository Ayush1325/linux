use core::ptr::NonNull;

use crate::{error, str::CStr, types::Opaque};

#[repr(transparent)]
pub struct NvmemDevice(NonNull<bindings::nvmem_device>);

impl NvmemDevice {
    unsafe extern "C" fn nvmem_device_find_cb(
        dev: *mut bindings::device,
        data: *const kernel::ffi::c_void,
    ) -> kernel::ffi::c_int {
        let parent_dev = data as *const bindings::device;

        if unsafe { (*dev).parent.cast_const() == parent_dev } {
            1
        } else {
            0
        }
    }

    /// FIXME: Implement proper callback support at some point
    pub fn nvmem_device_find_by_parent(data: &kernel::device::Device) -> error::Result<Self> {
        let ptr = error::from_err_ptr(unsafe {
            bindings::nvmem_device_find(data.as_raw() as *mut _, Some(Self::nvmem_device_find_cb))
        })?;

        Ok(Self(NonNull::new(ptr).unwrap()))
    }

    fn as_raw(&self) -> *mut bindings::nvmem_device {
        self.0.as_ptr()
    }

    pub fn nvmem_device_read(&self, offset: u32, buf: &mut [u8]) -> error::Result<u32> {
        let t = unsafe {
            bindings::nvmem_device_read(self.as_raw(), offset, buf.len(), buf.as_mut_ptr().cast())
        };

        if t < 0 {
            Err(error::Error::from_errno(t))
        } else {
            Ok(t as u32)
        }
    }
}

impl Drop for NvmemDevice {
    fn drop(&mut self) {
        unsafe { bindings::nvmem_device_put(self.as_raw()) }
    }
}
