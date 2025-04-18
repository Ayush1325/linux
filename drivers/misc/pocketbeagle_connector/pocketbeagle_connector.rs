// SPDX-License-Identifier: GPL-2.0

//! PocketBeagle Connector driver

mod eeprom;
mod sysfs_api;

use kernel::{
    c_str,
    device::{self, Core},
    error, of, platform,
    prelude::*,
    str::CString,
    sysfs,
    types::ARef,
};

use eeprom::EEPROMData;
use sysfs_api::{CapeAttribute, RemoveCapeAttribute};

/// Find and add cape overlay. Thus function is also used in probe. Hence cannot modify device
/// data.
fn add_cape(dev: &device::Device<device::Bound>, cape_name: &CStr) -> Result<kernel::of::OvcsId> {
    let dtbo = CString::try_from_fmt(fmt!("pocketbeagle2_connector/{}.dtbo", cape_name))?;
    let firmware = kernel::firmware::Firmware::request(&dtbo, dev)?;
    let ovcs_id = kernel::of::OvcsId::of_overlay_fdt_apply(firmware.data(), dev.device_node())?;
    dev.devm_of_platform_populate()?;

    Ok(ovcs_id)
}

/// Remove any existing cape
fn remove_cape(dev: &device::Device<device::Bound>) -> Result<()> {
    let data = platform::driver_data_mut::<PocketBeagleConnector>(dev);

    match data.overlay.take() {
        Some((_, _)) => Ok(dev.devm_of_platform_depopulate()),
        None => Err(error::code::ENODEV),
    }
}

static CAPE_ATTRIBUTE: sysfs::DeviceAttribute<CapeAttribute> =
    sysfs::DeviceAttribute::new(c_str!("cape"), sysfs::MODE_RW);
static REMOVE_CAPE_ATTRIBUTE: sysfs::DeviceAttribute<RemoveCapeAttribute> =
    sysfs::DeviceAttribute::new(c_str!("remove_cape"), sysfs::MODE_WO);

static ATTRIBUTES: sysfs::AttributeArray<2> =
    sysfs::AttributeArray::new([CAPE_ATTRIBUTE.attr(), REMOVE_CAPE_ATTRIBUTE.attr()]);
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

fn read_eeprom(pdev: &platform::Device<Core>) -> Result<CString> {
    let fwnode = pdev.fwnode();

    let eeprom_board_info =
        kernel::i2c::I2cBoardInfo::new(c_str!("24c32"), 0x57, kernel::i2c::I2C_CLIENT_SLAVE);

    let i2c_adapter = fwnode
        .fwnode_find_reference(c_str!("i2c-eeprom"), 0)
        .map(|x| kernel::i2c::I2cAdapter::i2c_get_adapter_by_fwnode(&x))??;

    let eeprom = i2c_adapter.new_client_device(&eeprom_board_info)?;
    let nvmem_dev = kernel::nvmem::NvmemDevice::nvmem_device_find_by_parent(eeprom.device())?;

    let mut data: KBox<[u8; size_of::<EEPROMData>()]> =
        KBox::new([0u8; size_of::<EEPROMData>()], GFP_KERNEL).unwrap();

    let count = nvmem_dev.nvmem_device_read(0, data.as_mut_slice())?;
    if count as usize != size_of::<EEPROMData>() {
        return Err(error::code::EINVAL);
    }
    let Some(eeprom_data) = EEPROMData::from_bytes(&data) else {
        return Err(error::code::EINVAL);
    };

    CString::try_from_fmt(fmt!(
        "{}-{}",
        eeprom_data.part_number(),
        eeprom_data.version()
    ))
}

impl platform::Driver for PocketBeagleConnector {
    type IdInfo = ();
    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);
    const DRIVER_DEV_GROUPS: Option<kernel::sysfs::AttributeGroupTable> = Some(&ATTRIBUTE_GROUPS);

    fn probe(
        pdev: &platform::Device<Core>,
        _id_info: Option<&Self::IdInfo>,
    ) -> Result<Pin<KBox<Self>>> {
        dev_info!(pdev.as_ref(), "PocketBeagleConnector probe");

        let overlay = match read_eeprom(pdev) {
            Ok(x) => {
                dev_info!(pdev.as_ref(), "Found cape {}", x.as_ref());
                let ovcs_id = add_cape(pdev.as_ref(), &x)?;
                Some((x, ovcs_id))
            }
            Err(e) => {
                dev_warn!(pdev.as_ref(), "Failed to read Cape EEPROM: {:?}", e);
                None
            }
        };

        let drvdata = KBox::new(
            Self {
                pdev: pdev.into(),
                overlay,
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
