// SPDX-License-Identifier: GPL-2.0

#[repr(packed)]
pub(crate) struct EEPROMData {
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
    pub(crate) fn from_bytes(data: &[u8; size_of::<Self>()]) -> Option<&Self> {
        // Check header
        if data[..4] == [0xaa, 0x55, 0x33, 0xee] {
            let t = unsafe { core::mem::transmute(data) };
            Some(t)
        } else {
            None
        }
    }

    pub(crate) fn version(&self) -> &str {
        str::from_utf8(&self.version).unwrap()
    }

    pub(crate) fn part_number(&self) -> &str {
        let count = self.part_number.iter().take_while(|x| **x != 0).count();
        str::from_utf8(&self.part_number[..count]).unwrap()
    }
}
