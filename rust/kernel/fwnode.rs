use crate::{
    error,
    str::CStr,
    types::{ARef, AlwaysRefCounted, Opaque},
};

#[repr(transparent)]
pub struct FwnodeHandle(Opaque<bindings::fwnode_handle>);

impl FwnodeHandle {
    /// Convert a raw C `struct fwnode_handle` pointer to a `&'a FwnodeHandle`.
    ///
    /// # Safety
    ///
    /// Callers must ensure that `ptr` is valid, non-null. for the duration of this function call
    /// and the entire duration when the returned reference exists.
    pub unsafe fn as_ref<'a>(ptr: *mut bindings::fwnode_handle) -> &'a Self {
        // SAFETY: Guaranteed by the safety requirements of the function.
        unsafe { &*ptr.cast() }
    }

    /// Obtain the raw `struct fwnode_handle *`.
    pub fn as_raw(&self) -> *mut bindings::fwnode_handle {
        self.0.get()
    }

    pub fn fwnode_find_reference(&self, name: &CStr, idx: u32) -> error::Result<ARef<Self>> {
        let ptr = error::from_err_ptr(unsafe {
            bindings::fwnode_find_reference(self.as_raw(), name.as_ptr(), idx)
        })?;
        Ok(unsafe { Self::as_ref(ptr) }.into())
    }
}

unsafe impl AlwaysRefCounted for FwnodeHandle {
    fn inc_ref(&self) {
        unsafe { bindings::fwnode_handle_get(self.as_raw()) };
    }

    unsafe fn dec_ref(obj: core::ptr::NonNull<Self>) {
        unsafe { bindings::fwnode_handle_put(obj.cast().as_ptr()) };
    }
}
