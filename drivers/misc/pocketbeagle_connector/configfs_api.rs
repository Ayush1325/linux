// SPDX-License-Identifier: GPL-2.0

use kernel::{
    configfs, error, of::OvcsId, page::PAGE_SIZE, platform, prelude::*, str::CString, sync::Mutex,
    types::ARef,
};

use crate::add_cape;

#[pin_data]
pub(crate) struct Configuration {
    pdev: ARef<platform::Device>,
    #[pin]
    overlay: Mutex<Option<(CString, OvcsId)>>,
}

impl Configuration {
    pub(crate) fn new(
        pdev: ARef<platform::Device>,
        overlay: Option<(CString, OvcsId)>,
    ) -> impl PinInit<Self, Error> {
        try_pin_init!(Self { pdev, overlay <- kernel::new_mutex!(overlay) })
    }
}

#[vtable]
impl configfs::AttributeOperations<0> for Configuration {
    type Data = Configuration;

    fn show(container: &Configuration, page: &mut [u8; PAGE_SIZE]) -> Result<usize> {
        match container.overlay.lock().as_ref() {
            Some((x, _)) => {
                page[..x.len()].copy_from_slice(x);
                Ok(x.len())
            }
            None => Err(error::code::ENODEV),
        }
    }

    fn store(container: &Configuration, page: &[u8]) -> Result {
        let dev = unsafe { container.pdev.as_ref().as_bound() };
        let mut overlay = container.overlay.lock();

        let parsed = core::str::from_utf8(page)
            .map_err(|_| error::code::EINVAL)?
            .trim();

        if parsed.is_empty() {
            dev_info!(container.pdev.as_ref(), "Removing cape");

            match overlay.take() {
                Some((_, _)) => Ok(dev.devm_of_platform_depopulate()),
                None => Err(error::code::ENODEV),
            }
        } else {
            dev_info!(container.pdev.as_ref(), "Adding cape: {}", parsed);

            let cape_name = CString::try_from_fmt(fmt!("{}", parsed))?;
            let id = add_cape(dev, &cape_name)?;
            *overlay = Some((cape_name, id));

            Ok(())
        }
    }
}
