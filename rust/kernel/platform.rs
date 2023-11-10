// SPDX-License-Identifier: GPL-2.0

//! Platform devices and drivers.
//!
//! Also called `platformdev`, `pdev`.
//!
//! C header: [`include/linux/platform_device.h`](../../../../include/linux/platform_device.h)

use core::{marker::PhantomData, pin::Pin, ptr::addr_of_mut};

use macros::vtable;

use crate::{
    bindings, device,
    error::{from_result, Result},
    str::CStr,
    types::Opaque,
};

/// A platform device.
///
/// # Invariants
///
/// The field `ptr` is non-null and valid for the lifetime of the object.
#[repr(transparent)]
pub struct Device(Opaque<bindings::platform_device>);

impl Device {
    /// Creates a new [`Device`] instance from a raw pointer.
    ///
    /// # Safety
    ///
    /// For the duration of `'a`,
    /// - the pointer must point at a valid `platform_device`, and the caller
    ///   must be in a context where all methods defined on this struct
    ///   are safe to call.
    unsafe fn from_raw<'a>(ptr: *mut bindings::platform_device) -> &'a mut Self {
        // CAST: `Self` is a `repr(transparent)` wrapper around `bindings::platform_device`.
        let ptr = ptr.cast::<Self>();
        // SAFETY: by the function requirements the pointer is valid and we have unique access for
        // the duration of `'a`.
        unsafe { &mut *ptr }
    }

    /// Returns id of the platform device.
    pub fn id(&self) -> i32 {
        let platformdev = self.0.get();
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { (*platformdev).id }
    }
}

impl AsRef<device::Device> for Device {
    fn as_ref(&self) -> &device::Device {
        let platformdev = self.0.get();
        // SAFETY: By the type invariants, we know that `self.ptr` is non-null and valid.
        unsafe { device::Device::as_ref(addr_of_mut!((*platformdev).dev)) }
    }
}

/// An adapter for the registration of a Platform driver.
struct Adapter<T: Driver> {
    _p: PhantomData<T>,
}

impl<T: Driver> Adapter<T> {
    /// # Safety
    ///
    /// `pdev` must be passed by the corresponding callback in `platform_driver`.
    unsafe extern "C" fn probe_callback(pdev: *mut bindings::platform_device) -> core::ffi::c_int {
        from_result(|| {
            // SAFETY: This callback is called only in contexts
            // where we can exclusively access `platform_device` because
            // it's not published yet, so the accessors on `Device` are okay
            // to call.
            let dev = unsafe { Device::from_raw(pdev) };
            T::probe(dev)?;
            Ok(0)
        })
    }

    /// # Safety
    ///
    /// `pdev` must be passed by the corresponding callback in `platform_driver`.
    unsafe extern "C" fn remove_callback(pdev: *mut bindings::platform_device) {
        // SAFETY: This callback is called only in contexts
        // where we can exclusively access `platform_device` because
        // it's not published yet, so the accessors on `Device` are okay
        // to call.
        let dev = unsafe { Device::from_raw(pdev) };
        T::remove(dev);
    }
}

/// Driver structure for a particular Platform driver.
///
/// Wraps the kernel's [`struct platform_driver`].
/// This is used to register a driver for a particular PHY type with the kernel.
///
/// # Invariants
///
/// `self.0` is always in a valid state.
///
/// [`struct platform_driver`]: srctree/include/linux/platform.h
#[repr(transparent)]
pub struct DriverVTable(Opaque<bindings::platform_driver>);

// SAFETY: `DriverVTable` doesn't expose any &self method to access internal data, so it's safe to
// share `&DriverVTable` across execution context boundaries.
unsafe impl Sync for DriverVTable {}

impl DriverVTable {
    /// Creates a [`DriverVTable`] instance from [`Driver`].
    ///
    /// This is used by [`module_platform_driver`] macro to create a static array of `phy_driver`.
    ///
    /// [`module_platform_driver`]: crate::module_platform_driver
    pub const fn new<T: Driver, const C: usize>(match_tbl: &'static DeviceIdTable<C>) -> Self {
        let drv = Opaque::new(bindings::platform_driver {
            probe: if T::HAS_PROBE {
                Some(Adapter::<T>::probe_callback)
            } else {
                None
            },
            __bindgen_anon_1: bindings::platform_driver__bindgen_ty_1 {
                remove: if T::HAS_REMOVE {
                    Some(Adapter::<T>::remove_callback)
                } else {
                    None
                },
            },
            driver: create_device_driver::<T, C>(match_tbl),
            // SAFETY: The rest is zeroed out to initialize `struct platform_driver`.
            ..unsafe { core::mem::MaybeUninit::<bindings::platform_driver>::zeroed().assume_init() }
        });

        DriverVTable(drv)
    }
}

const fn create_device_driver<T: Driver, const C: usize>(
    match_tbl: &'static DeviceIdTable<C>,
) -> bindings::device_driver {
    bindings::device_driver {
        name: T::NAME.as_char_ptr(),
        of_match_table: match_tbl.get(),
        // SAFETY: The rest is zeroed out to initialize `struct device_driver`.
        ..unsafe { core::mem::MaybeUninit::<bindings::device_driver>::zeroed().assume_init() }
    }
}

/// A platform driver.
#[vtable]
pub trait Driver {
    /// The friendly name
    const NAME: &'static CStr;

    /// Sets up device-specific structures during discovery.
    fn probe(_dev: &mut Device) -> Result;

    /// Clean up device-specific structures during removal.
    fn remove(_dev: &mut Device);
}

/// Registration structure for Platform driver.
///
/// Registers [`DriverVTable`] instance with the kernel. It will be unregistered when dropped.
///
/// # Invariants
///
/// The `driver` is currently registered to the kernel via `__platform_driver_register`.
pub struct Registration(Pin<&'static DriverVTable>);

// SAFETY: The only action allowed in a `Registration` instance is dropping it, which is safe to do
// from any thread because `platform_drivers_unregister` can be called from any thread context.
unsafe impl Send for Registration {}

impl Registration {
    /// Registers a Platform driver.
    pub fn new(drv: Pin<&'static DriverVTable>, m: &'static crate::ThisModule) -> Registration {
        unsafe {
            bindings::__platform_driver_register(drv.0.get(), m.0);
        }

        Self(drv)
    }
}

impl Drop for Registration {
    fn drop(&mut self) {
        unsafe { bindings::platform_driver_unregister(self.0 .0.get()) }
    }
}

/// An identifier for Platform devices.
///
/// Represents the kernel's [`struct of_device_id`]. This is used to find an appropriate
/// Platform driver.
///
/// [`struct of_device_id`]: srctree/include/linux/mod_devicetable.h
pub struct DeviceId(&'static CStr);

impl DeviceId {
    /// A zeroed [`struct of_device_id`] used to signify end of of_device_id array.
    ///
    /// [`struct of_device_id`]: srctree/include/linux/mod_devicetable.h
    pub const ZERO: bindings::of_device_id = bindings::of_device_id {
        // SAFETY: The rest is zeroed out to initialize `struct of_device_id`.
        ..unsafe { core::mem::MaybeUninit::<bindings::of_device_id>::zeroed().assume_init() }
    };

    /// Create new instance
    pub const fn new(s: &'static CStr) -> Self {
        Self(s)
    }

    const fn compatible(&self) -> [i8; 128] {
        let compatible = self.0.as_bytes_with_nul();
        let mut comp = [0i8; 128];
        let mut i = 0;

        while i < compatible.len() {
            comp[i] = compatible[i] as _;
            i += 1;
        }

        comp
    }

    // macro use only
    #[doc(hidden)]
    pub const fn to_rawid(&self) -> bindings::of_device_id {
        let comp = self.compatible();

        bindings::of_device_id {
            compatible: comp,
            // SAFETY: The rest is zeroed out to initialize `struct of_device_id`.
            ..unsafe { core::mem::MaybeUninit::<bindings::of_device_id>::zeroed().assume_init() }
        }
    }
}

/// An array of identifiers for platform driver
#[repr(transparent)]
pub struct DeviceIdTable<const C: usize>([bindings::of_device_id; C]);

impl<const C: usize> DeviceIdTable<C> {
    /// Create a new instance
    pub const fn new(ids: [bindings::of_device_id; C]) -> Self {
        Self(ids)
    }

    /// Returns a raw pointer to static table.
    pub const fn get(&'static self) -> *const bindings::of_device_id {
        self.0.as_ptr()
    }
}

// SAFETY: `DeviceIdTable` is only used in C side behind a *const pointer, and thus remains
// immutable and thus can be shared across execution context boundaries.
unsafe impl<const C: usize> Sync for DeviceIdTable<C> {}

/// Declares a kernel module for Platform drivers.
///
/// This creates a static [`struct platform_driver`] and registers it. It also creates an array of
/// [`struct of_device_id`] for matching the driver to devicetree device.
///
/// [`struct platform_driver`]: srctree/include/linux/platform.h
/// [`struct of_device_id`]: srctree/include/linux/mod_devicetable.h
///
/// # Examples
///
/// ```
/// # mod module_platform_driver_sample {
/// use kernel::c_str;
/// use kernel::platform::{self, DeviceId};
/// use kernel::prelude::*;
///
/// kernel::module_platform_driver! {
///     driver: PlatformSimple,
///     of_table: [DeviceId::new(c_str!("platform-simple"))],
///     name: "rust_sample_platform",
///     author: "Rust for Linux Contributors",
///     description: "Rust sample Platform driver",
///     license: "GPL",
/// }
///
/// struct PlatformSimple;
///
/// #[vtable]
/// impl platform::Driver for PlatformSimple {
///     const NAME: &'static CStr = c_str!("PlatformSimple");
/// }
/// # }
/// ```
///
/// This expands to the following code:
///
/// ```ignore
/// use kernel::c_str;
/// use kernel::platform::{self, DeviceId};
/// use kernel::prelude::*;
///
///
/// struct Module {
///     _reg: $crate::platform::Registration,
/// }
///
/// module! {
///     type: Module,
///     name: "rust_sample_platform",
///     author: "Rust for Linux Contributors",
///     description: "Rust sample Platform driver",
///     license: "GPL",
/// }
///
/// const _: () = {
///     static OF_TABLE: $crate::platform::DeviceIdTable = $crate::platform::DeviceIdTable<2>([
///         (DeviceId::new(c_str!("platform-simple"))).to_rawid(),
///         $crate::platform::DeviceId::ZERO,
///     ]);
///     static DRIVER: $crate::platform::DriverVTable =
///         $crate::platform::DriverVTable::new::<MikrobusDriver, 2>(&OF_TABLE);
///     impl $crate::Module for Module {
///         fn init(module: &'static ThisModule) -> Result<Self> {
///             let reg =
///                 $crate::platform::Registration::new(
///                     ::core::pin::Pin::static_ref(&DRIVER), module);
///             Ok(Module { _reg: reg })
///         }
///     }
/// }
///
/// struct PlatformSimple;
///
/// #[vtable]
/// impl platform::Driver for PlatformSimple {
///     const NAME: &'static CStr = c_str!("PlatformSimple");
/// }
/// ```
#[macro_export]
macro_rules! module_platform_driver {
    (@replace_expr $_t:tt $sub:expr) => {$sub};

    (@count_devices $($x:expr),*) => {
        0usize $(+ $crate::module_platform_driver!(@replace_expr $x 1usize))*
    };

    (driver: $driver:ident, of_table: [$($of_id:expr),+ $(,)?], $($f:tt)*) => {
        struct Module {
            _reg: $crate::platform::Registration,
        }

        $crate::prelude::module! {
            type: Module,
            $($f)*
        }

        const _: () = {
            // SAFETY: C will not read off the end of this constant since the last element is zero.
            static OF_TABLE: $crate::platform::DeviceIdTable<
                {$crate::module_platform_driver!(@count_devices $($of_id),+) + 1} > =
                $crate::platform::DeviceIdTable::new(
                    [$($of_id.to_rawid()),*, $crate::platform::DeviceId::ZERO]);

            static DRIVER: $crate::platform::DriverVTable =
                $crate::platform::DriverVTable::new::<
                    $driver, {$crate::module_platform_driver!(@count_devices $($of_id),+) + 1}
                >(&OF_TABLE);

            impl $crate::Module for Module {
                fn init(module: &'static ThisModule) -> Result<Self> {
                    let reg = $crate::platform::Registration::new(
                        ::core::pin::Pin::static_ref(&DRIVER), module);
                    Ok(Module { _reg: reg })
                }
            }
        };
    };
}
