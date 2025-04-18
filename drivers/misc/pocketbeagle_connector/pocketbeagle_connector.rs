// SPDX-License-Identifier: GPL-2.0

//! PocketBeagle Connector driver

use kernel::{
    c_str,
    device::{self, Core},
    error, of, platform,
    prelude::*,
    str::CString,
    sysfs,
    types::ARef,
};

struct NewCapeAttribute;

#[vtable]
impl sysfs::DeviceAttributeOps for NewCapeAttribute {
    fn store(dev: &device::Device<device::Bound>, buf: &[u8]) -> error::Result<usize> {
        let data = platform::driver_data_mut::<PocketBeagleConnector>(dev);

        if data.overlay.is_some() {
            return Err(error::code::EBUSY);
        }

        let parsed = core::str::from_utf8(buf).map_err(|_| error::code::EINVAL)?;
        let dtbo = CString::try_from_fmt(kernel::fmt!("{}.dtbo", parsed.trim()))?;
        dev_info!(dev, "Adding cape: {}", dtbo.as_ref());

        let firmware = kernel::firmware::Firmware::request(&dtbo, dev)?;
        dev_info!(dev, "Found firmware of size: {}", firmware.size());

        let ovcd_id = kernel::of::OvcsId::of_overlay_fdt_apply(firmware.data(), dev.device_node())?;

        dev.devm_of_platform_populate()?;

        data.overlay = Some((dtbo, ovcd_id));

        Ok(buf.len())
    }
}

static NEW_CAPE_ATTRIBUTE: sysfs::DeviceAttribute<NewCapeAttribute> =
    sysfs::DeviceAttribute::new(c_str!("new_cape"), sysfs::MODE_WO);

struct CurrentCapeAttribute;

#[vtable]
impl sysfs::DeviceAttributeOps for CurrentCapeAttribute {
    fn show(dev: &device::Device<device::Bound>, buf: sysfs::SysfsWBuf) -> error::Result<usize> {
        let data = platform::driver_data_mut::<PocketBeagleConnector>(dev);
        if let Some((x, _)) = &data.overlay {
            Ok(kernel::sysfs_emit!(buf, "{}\n", x.as_ref()))
        } else {
            Ok(0)
        }
    }
}

static HAS_CAPE_ATTRIBUTE: sysfs::DeviceAttribute<CurrentCapeAttribute> =
    sysfs::DeviceAttribute::new(c_str!("current_cape"), sysfs::MODE_RO);

struct RemoveCapeAttribute;

#[vtable]
impl sysfs::DeviceAttributeOps for RemoveCapeAttribute {
    fn store(dev: &device::Device<device::Bound>, buf: &[u8]) -> error::Result<usize> {
        let data = platform::driver_data_mut::<PocketBeagleConnector>(dev);

        if let Some((x, _overlay)) = data.overlay.take() {
            dev.devm_of_platform_depopulate();
            dev_info!(dev, "Removing cape: {}", x.as_ref());
            Ok(buf.len())
        } else {
            Err(error::code::ENODEV)
        }
    }
}

static REMOVE_CAPE_ATTRIBUTE: sysfs::DeviceAttribute<RemoveCapeAttribute> =
    sysfs::DeviceAttribute::new(c_str!("remove_cape"), sysfs::MODE_WO);

static ATTRIBUTES: sysfs::AttributeArray<3> = sysfs::AttributeArray::new([
    NEW_CAPE_ATTRIBUTE.attr(),
    HAS_CAPE_ATTRIBUTE.attr(),
    REMOVE_CAPE_ATTRIBUTE.attr(),
]);

static ATTRIBUTE_GROUP: sysfs::AttributeGroup = sysfs::AttributeGroup::new(&ATTRIBUTES);

static ATTRIBUTE_GROUPS: sysfs::AttributeGroupsArray<1> =
    sysfs::AttributeGroupsArray::new([&ATTRIBUTE_GROUP]);

struct PocketBeagleConnector {
    pdev: ARef<platform::Device>,
    overlay: Option<(CString, kernel::of::OvcsId)>,
}

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <PocketBeagleConnector as platform::Driver>::IdInfo,
    [(of::DeviceId::new(c_str!("pocketbeagle2-connector")), ())]
);

impl platform::Driver for PocketBeagleConnector {
    type IdInfo = ();
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);
    const DRIVER_DEV_GROUPS: Option<kernel::sysfs::AttributeGroupTable> = Some(&ATTRIBUTE_GROUPS);

    fn probe(
        pdev: &platform::Device<Core>,
        _id_info: Option<&Self::IdInfo>,
    ) -> Result<Pin<KBox<Self>>> {
        dev_info!(pdev.as_ref(), "PocketBeagleConnector probe");

        let drvdata = KBox::new(
            Self {
                pdev: pdev.into(),
                overlay: None,
            },
            GFP_KERNEL,
        )?;

        Ok(drvdata.into())
    }
}

impl Drop for PocketBeagleConnector {
    fn drop(&mut self) {
        dev_info!(self.pdev.as_ref(), "PocketBeagleConnector drop");
    }
}

kernel::module_platform_driver! {
    type: PocketBeagleConnector,
    name: "pocktbeagle_connector",
    authors: ["Ayush Singh"],
    description: "Driver for PocketBeagle Connector",
    license: "GPL v2",
}
