// SPDX-License-Identifier: GPL-2.0

//! MikroBUS driver

use kernel::c_str;
use kernel::platform::{self, DeviceId};
use kernel::prelude::*;

kernel::module_platform_driver! {
    driver: MikrobusDriver,
    of_table: [DeviceId::new(c_str!("mikrobus-connector"))],
    name: "mikrobus",
    author: "Ayush Singh <ayush@beagleboard.org>",
    description: "MikroBUS connector Driver",
    license: "GPL",
}

struct MikrobusDriver;

#[vtable]
impl platform::Driver for MikrobusDriver {
    const NAME: &'static CStr = c_str!("MikroBUS");

    fn probe(_dev: &mut platform::Device) -> Result {
        pr_debug!("Mikrobus Driver (probe)\n");
        Ok(())
    }

    fn remove(_dev: &mut platform::Device) {
        pr_debug!("Mikrobus Driver (remove)\n");
    }
}
