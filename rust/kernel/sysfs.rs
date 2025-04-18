use macros::vtable;

use crate::{device, str::CStr};
use core::{fmt, marker::PhantomData, mem::MaybeUninit};

pub const MODE_RO: u16 = 0o444;
pub const MODE_WO: u16 = 0o200;
pub const MODE_RW: u16 = 0o644;

pub struct SysfsWBuf(*mut ffi::c_char);

impl SysfsWBuf {
    pub fn emit(&self, msg: fmt::Arguments<'_>) -> usize {
        unsafe {
            bindings::sysfs_emit(
                self.0,
                crate::c_str!("%pA").as_char_ptr(),
                &msg as *const _ as *const crate::ffi::c_void,
            ) as usize
        }
    }
}

#[macro_export]
macro_rules! sysfs_emit {
    ($buf:expr, $($f:tt)*) => {
        {
            ($buf).emit(core::format_args!($($f)*))
        }
    }
}

#[repr(transparent)]
pub struct DeviceAttribute<T: DeviceAttributeOps> {
    inner: bindings::device_attribute,
    phantom: PhantomData<T>,
}

unsafe impl<T: DeviceAttributeOps> Sync for DeviceAttribute<T> {}

#[vtable]
pub trait DeviceAttributeOps {
    fn show(_dev: &device::Device<device::Bound>, _buf: SysfsWBuf) -> crate::error::Result<usize> {
        crate::build_error!(crate::error::VTABLE_DEFAULT_ERROR)
    }

    fn store(_dev: &device::Device<device::Bound>, _buf: &[u8]) -> crate::error::Result<usize> {
        crate::build_error!(crate::error::VTABLE_DEFAULT_ERROR)
    }
}

impl<T: DeviceAttributeOps> DeviceAttribute<T> {
    pub const fn new(name: &'static CStr, mode: u16) -> Self {
        let show = if T::HAS_SHOW {
            Some(Self::show_callback as _)
        } else {
            None
        };

        let store = if T::HAS_STORE {
            Some(Self::store_callback as _)
        } else {
            None
        };

        let temp = bindings::device_attribute {
            attr: bindings::attribute {
                name: name.as_char_ptr(),
                mode,
            },
            show,
            store,
        };
        Self {
            inner: temp,
            phantom: PhantomData,
        }
    }

    pub const fn attr(&self) -> &bindings::attribute {
        &self.inner.attr
    }

    extern "C" fn show_callback(
        device: *mut bindings::device,
        attr: *mut bindings::device_attribute,
        buf: *mut ffi::c_char,
    ) -> isize {
        let dev = unsafe { device::Device::as_ref(device) };
        match T::show(dev, SysfsWBuf(buf)) {
            Ok(x) => x as isize,
            Err(e) => e.to_errno() as isize,
        }
    }

    extern "C" fn store_callback(
        device: *mut bindings::device,
        attr: *mut bindings::device_attribute,
        buf: *const ffi::c_char,
        count: usize,
    ) -> isize {
        let dev = unsafe { device::Device::as_ref(device) };
        let b = unsafe { core::slice::from_raw_parts(buf, count) };

        match T::store(dev, b) {
            Ok(x) => x as isize,
            Err(e) => e.to_errno() as isize,
        }
    }
}

#[repr(C)]
pub struct AttributeArray<const N: usize> {
    attributes: [*const bindings::attribute; N],
    sentinel: MaybeUninit<*const bindings::attribute>,
}

unsafe impl<const N: usize> Sync for AttributeArray<N> {}

impl<const N: usize> AttributeArray<N> {
    pub const fn new(attrs: [&'static bindings::attribute; N]) -> Self {
        let mut attributes: [*const bindings::attribute; N] = [core::ptr::null(); N];
        let mut i = 0;

        while i < N {
            attributes[i] = attrs[i];
            i += 1;
        }

        Self {
            attributes,
            sentinel: MaybeUninit::zeroed(),
        }
    }

    pub const fn as_ptr(&self) -> *mut *mut bindings::attribute {
        (self as *const _ as *mut Self).cast()
    }
}

#[repr(transparent)]
pub struct AttributeGroup(bindings::attribute_group);

unsafe impl Sync for AttributeGroup {}

impl AttributeGroup {
    pub const fn new<const N: usize>(attr_arr: &'static AttributeArray<N>) -> Self {
        let temp = unsafe {
            bindings::attribute_group {
                attrs: attr_arr.as_ptr(),
                ..core::mem::zeroed()
            }
        };
        Self(temp)
    }

    pub const fn attribute_group(&self) -> &bindings::attribute_group {
        &self.0
    }
}

#[repr(C)]
pub struct AttributeGroupsArray<const N: usize> {
    attribute_groups: [*const bindings::attribute_group; N],
    sentinel: MaybeUninit<*const bindings::attribute_group>,
}

unsafe impl<const N: usize> Sync for AttributeGroupsArray<N> {}

impl<const N: usize> AttributeGroupsArray<N> {
    pub const fn new(attr_groups: [&'static AttributeGroup; N]) -> Self {
        let mut attributes: [*const bindings::attribute_group; N] = [core::ptr::null(); N];
        let mut i = 0;

        while i < N {
            attributes[i] = attr_groups[i].attribute_group();
            i += 1;
        }

        Self {
            attribute_groups: attributes,
            sentinel: MaybeUninit::zeroed(),
        }
    }
}

pub type AttributeGroupTable = &'static dyn AttributeGroupsTable;

pub trait AttributeGroupsTable {
    fn as_ptr(&self) -> *mut *const bindings::attribute_group;
}

impl<const N: usize> AttributeGroupsTable for AttributeGroupsArray<N> {
    fn as_ptr(&self) -> *mut *const bindings::attribute_group {
        (self as *const _ as *mut Self).cast()
    }
}
