// SPDX-License-Identifier: GPL-2.0

use kernel::{c_str, device::Core, error, platform, prelude::*, str::CString};

#[repr(packed)]
struct EEPROMData {
    _header: [u8; 4],
    _eeprom_revision: [u8; 2],
    _board_name: [u8; 32],
    version: [u8; 4],
    _manufacturer: [u8; 16],
    part_number: [u8; 16],
    _number_of_pins: [u8; 2],
    _serial_number: [u8; 12],
    _pin_usage: [u8; 148],
    _vdd_3v3b_current: [u8; 2],
    _vdd_5v_current: [u8; 2],
    _sys_5v_current: [u8; 2],
    _dc_supplied: [u8; 2],
}

impl EEPROMData {
    fn from_bytes(data: &[u8; size_of::<Self>()]) -> Option<&Self> {
        // Check header
        if data[..4] == [0xaa, 0x55, 0x33, 0xee] {
            let t = unsafe { core::mem::transmute(data) };
            Some(t)
        } else {
            None
        }
    }

    fn version(&self) -> &str {
        str::from_utf8(&self.version).unwrap()
    }

    fn part_number(&self) -> &str {
        let count = self.part_number.iter().take_while(|x| **x != 0).count();
        str::from_utf8(&self.part_number[..count]).unwrap()
    }
}

pub(crate) fn read_eeprom(pdev: &platform::Device<Core>) -> Result<CString> {
    let fwnode = pdev.as_ref().fwnode().unwrap();

    let eeprom_board_info =
        kernel::i2c::I2cBoardInfo::new(c_str!("24c32"), 0x57, kernel::i2c::I2C_CLIENT_SLAVE);

    let i2c_adapter = fwnode
        .find_reference(c_str!("i2c-eeprom"), 0)
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
