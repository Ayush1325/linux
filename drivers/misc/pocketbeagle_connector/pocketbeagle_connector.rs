// SPDX-License-Identifier: GPL-2.0

//! PocketBeagle Connector driver

mod configfs_api;
mod eeprom;

use configfs_api::Configuration;
use eeprom::read_eeprom;

use kernel::{
    c_str, configfs, configfs_attrs,
    device::{self, Core},
    of, platform,
    prelude::*,
    str::CString,
    types::ARef,
};

/// Find and add cape overlay. Thus function is also used in probe. Hence cannot modify device
/// data.
fn add_cape(dev: &device::Device<device::Bound>, cape_name: &CStr) -> Result<kernel::of::OvcsId> {
    let dtbo = CString::try_from_fmt(fmt!("pocketbeagle2_connector/{}.dtbo", cape_name))?;
    let firmware = kernel::firmware::Firmware::request(&dtbo, dev)?;
    let ovcs_id = kernel::of::OvcsId::of_overlay_fdt_apply(firmware.data(), dev.device_node())?;
    dev.devm_of_platform_populate()?;

    Ok(ovcs_id)
}

struct PocketBeagleConnector {
    pdev: ARef<platform::Device>,
    _config: Pin<KBox<configfs::Subsystem<Configuration>>>,
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

        let item_type = configfs_attrs! {
            container: configfs::Subsystem<Configuration>,
            data: Configuration,
            attributes: [
                cape: 0,
            ],
        };
        let pdev: ARef<platform::Device> = pdev.into();

        let config = KBox::pin_init(
            configfs::Subsystem::new(
                c_str!("pocketbeagle2_connector"),
                item_type,
                Configuration::new(pdev.clone(), overlay),
            ),
            GFP_KERNEL,
        )?;

        let drvdata = KBox::new(
            Self {
                pdev,
                _config: config,
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
