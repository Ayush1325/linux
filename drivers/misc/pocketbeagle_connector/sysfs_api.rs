// SPDX-License-Identifier: GPL-2.0

use kernel::{dev_info, device, error, fmt, macros::vtable, platform, str::CString, sysfs};

use crate::{add_cape, remove_cape, PocketBeagleConnector};

pub(crate) struct CapeAttribute;

#[vtable]
impl sysfs::DeviceAttributeOps for CapeAttribute {
    fn show(dev: &device::Device<device::Bound>, buf: sysfs::SysfsWBuf) -> error::Result<usize> {
        let data = platform::driver_data_mut::<PocketBeagleConnector>(dev);
        if let Some((x, _)) = &data.overlay {
            Ok(kernel::sysfs_emit!(buf, "{}\n", x.as_ref()))
        } else {
            Ok(0)
        }
    }

    fn store(dev: &device::Device<device::Bound>, buf: &[u8]) -> error::Result<usize> {
        let data = platform::driver_data_mut::<PocketBeagleConnector>(dev);

        dev_info!(dev, "Data: {:?}", buf);

        if data.overlay.is_some() {
            return Err(error::code::EBUSY);
        }

        let parsed = core::str::from_utf8(buf).map_err(|_| error::code::EINVAL)?;
        let cape_name = CString::try_from_fmt(fmt!("{}", parsed))?;

        dev_info!(dev, "Adding Cape");

        let id = add_cape(dev, &cape_name)?;
        data.overlay = Some((cape_name, id));

        Ok(buf.len())
    }
}

pub(crate) struct RemoveCapeAttribute;

#[vtable]
impl sysfs::DeviceAttributeOps for RemoveCapeAttribute {
    fn store(dev: &device::Device<device::Bound>, buf: &[u8]) -> error::Result<usize> {
        dev_info!(dev, "Removing Cape");
        remove_cape(dev)?;
        Ok(buf.len())
    }
}
