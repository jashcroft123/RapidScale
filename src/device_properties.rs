use embassy_rp::flash::{Blocking, Flash, Instance};

const FLASH_SIZE: usize = 2 * 1024 * 1024;
const ERASE_SIZE: u32 = 4096;
const RECORD_SIZE: usize = 128;
const SLOT_OFFSETS: [u32; 2] = [
    FLASH_SIZE as u32 - 2 * ERASE_SIZE,
    FLASH_SIZE as u32 - ERASE_SIZE,
];
const MAGIC: &[u8; 4] = b"RSP2";
const CRC_OFFSET: usize = 76;

#[derive(Clone, Copy)]
pub struct DeviceProperties {
    pub scale_factor_g_per_count: f32,
    pub tare_offset_counts: i32,
    pub tare_noise_variance_counts2: i64,
    pub tare_valid: bool,
    pub calibration_last_reference_mass_g: f32,
    pub calibration_last_raw_delta_counts: f32,
    pub characterisation_reference_mass_g: f32,
    pub characterisation_cycles: u8,
    pub characterisation_convergence_g: f32,
    pub characterisation_window_samples: u16,
    pub characterisation_valid: bool,
    pub characterisation_background_noise_sd_g: f32,
    pub characterisation_loaded_mean_g: f32,
    pub characterisation_loaded_noise_sd_g: f32,
    pub characterisation_average_settle_ms: u32,
    pub characterisation_maximum_settle_ms: u32,
}

impl Default for DeviceProperties {
    fn default() -> Self {
        Self {
            scale_factor_g_per_count: 0.0002627,
            tare_offset_counts: 0,
            tare_noise_variance_counts2: 0,
            tare_valid: false,
            calibration_last_reference_mass_g: 0.0,
            calibration_last_raw_delta_counts: 0.0,
            characterisation_reference_mass_g: 500.0,
            characterisation_cycles: 5,
            characterisation_convergence_g: 0.01,
            characterisation_window_samples: 160,
            characterisation_valid: false,
            characterisation_background_noise_sd_g: 0.0,
            characterisation_loaded_mean_g: 0.0,
            characterisation_loaded_noise_sd_g: 0.0,
            characterisation_average_settle_ms: 0,
            characterisation_maximum_settle_ms: 0,
        }
    }
}

pub fn assign_property(
    p: &mut DeviceProperties,
    key: &str,
    text: &str,
) -> Result<(), &'static str> {
    match key {
        "calibration.scale_factor_g_per_count" => {
            let value = parse_f32(text)?;
            if value == 0.0 || value.abs() > 1.0 {
                return Err("BAD_VALUE");
            }
            p.scale_factor_g_per_count = value;
        }
        "tare.offset_counts" => {
            p.tare_offset_counts = text.parse().map_err(|_| "BAD_VALUE")?;
            p.tare_valid = true;
        }
        "tare.noise_variance_counts2" => {
            let value: i64 = text.parse().map_err(|_| "BAD_VALUE")?;
            if value < 0 {
                return Err("BAD_VALUE");
            }
            p.tare_noise_variance_counts2 = value;
        }
        "characterisation.reference_mass_g" => {
            let value = parse_f32(text)?;
            if value <= 0.0 {
                return Err("BAD_VALUE");
            }
            p.characterisation_reference_mass_g = value;
        }
        "characterisation.cycles" => {
            let value: u8 = text.parse().map_err(|_| "BAD_VALUE")?;
            if !(1..=10).contains(&value) {
                return Err("BAD_VALUE");
            }
            p.characterisation_cycles = value;
        }
        "characterisation.convergence_g" => {
            let value = parse_f32(text)?;
            if !(0.001..=1.0).contains(&value) {
                return Err("BAD_VALUE");
            }
            p.characterisation_convergence_g = value;
        }
        "characterisation.window_samples" => {
            let value: u16 = text.parse().map_err(|_| "BAD_VALUE")?;
            if !(32..=640).contains(&value) {
                return Err("BAD_VALUE");
            }
            p.characterisation_window_samples = value;
        }
        "tare.valid"
        | "calibration.last_reference_mass_g"
        | "calibration.last_raw_delta_counts"
        | "characterisation.valid"
        | "characterisation.background_noise_sd_g"
        | "characterisation.loaded_mean_g"
        | "characterisation.loaded_noise_sd_g"
        | "characterisation.average_settle_ms"
        | "characterisation.maximum_settle_ms" => return Err("READ_ONLY"),
        _ => return Err("BAD_PROPERTY"),
    }
    p.characterisation_valid = false;
    Ok(())
}

fn parse_f32(text: &str) -> Result<f32, &'static str> {
    let value: f32 = text.parse().map_err(|_| "BAD_VALUE")?;
    if !value.is_finite() {
        return Err("BAD_VALUE");
    }
    Ok(value)
}

pub struct PersistentProperties {
    pub values: DeviceProperties,
    generation: u32,
    active_slot: Option<usize>,
}

impl PersistentProperties {
    pub fn load<T: Instance>(flash: &mut Flash<'_, T, Blocking, FLASH_SIZE>) -> Self {
        let record0 = read_record(flash, SLOT_OFFSETS[0]);
        let record1 = read_record(flash, SLOT_OFFSETS[1]);
        let selected = match (record0, record1) {
            (Some(a), Some(b)) => {
                if generation_is_newer(b.0, a.0) {
                    Some((b, 1))
                } else {
                    Some((a, 0))
                }
            }
            (Some(a), None) => Some((a, 0)),
            (None, Some(b)) => Some((b, 1)),
            (None, None) => None,
        };

        match selected {
            Some(((generation, values), active_slot)) => Self {
                values,
                generation,
                active_slot: Some(active_slot),
            },
            None => Self {
                values: DeviceProperties::default(),
                generation: 0,
                active_slot: None,
            },
        }
    }

    pub fn save<T: Instance>(
        &mut self,
        flash: &mut Flash<'_, T, Blocking, FLASH_SIZE>,
    ) -> Result<(), ()> {
        let target_slot = match self.active_slot {
            Some(0) => 1,
            _ => 0,
        };
        let generation = self.generation.wrapping_add(1);
        let bytes = encode_record(generation, &self.values);
        let offset = SLOT_OFFSETS[target_slot];

        flash
            .blocking_erase(offset, offset + ERASE_SIZE)
            .map_err(|_| ())?;
        flash.blocking_write(offset, &bytes).map_err(|_| ())?;
        let Some((written_generation, written_values)) = read_record(flash, offset) else {
            return Err(());
        };
        if written_generation != generation || !same_values(&written_values, &self.values) {
            return Err(());
        }

        self.generation = generation;
        self.active_slot = Some(target_slot);
        Ok(())
    }
}

fn read_record<T: Instance>(
    flash: &mut Flash<'_, T, Blocking, FLASH_SIZE>,
    offset: u32,
) -> Option<(u32, DeviceProperties)> {
    let mut bytes = [0u8; RECORD_SIZE];
    flash.blocking_read(offset, &mut bytes).ok()?;
    decode_record(&bytes)
}

fn encode_record(generation: u32, p: &DeviceProperties) -> [u8; RECORD_SIZE] {
    let mut bytes = [0xFF; RECORD_SIZE];
    bytes[0..4].copy_from_slice(MAGIC);
    bytes[4..8].copy_from_slice(&generation.to_le_bytes());
    bytes[8..10].copy_from_slice(&2u16.to_le_bytes());
    bytes[12..16].copy_from_slice(&p.scale_factor_g_per_count.to_bits().to_le_bytes());
    bytes[16..20].copy_from_slice(&p.tare_offset_counts.to_le_bytes());
    bytes[20..28].copy_from_slice(&p.tare_noise_variance_counts2.to_le_bytes());
    bytes[28] = u8::from(p.tare_valid);
    bytes[32..36].copy_from_slice(&p.calibration_last_reference_mass_g.to_bits().to_le_bytes());
    bytes[36..40].copy_from_slice(&p.calibration_last_raw_delta_counts.to_bits().to_le_bytes());
    bytes[40..44].copy_from_slice(&p.characterisation_reference_mass_g.to_bits().to_le_bytes());
    bytes[44] = p.characterisation_cycles;
    bytes[48..52].copy_from_slice(&p.characterisation_convergence_g.to_bits().to_le_bytes());
    bytes[52..54].copy_from_slice(&p.characterisation_window_samples.to_le_bytes());
    bytes[54] = u8::from(p.characterisation_valid);
    bytes[56..60].copy_from_slice(
        &p.characterisation_background_noise_sd_g
            .to_bits()
            .to_le_bytes(),
    );
    bytes[60..64].copy_from_slice(&p.characterisation_loaded_mean_g.to_bits().to_le_bytes());
    bytes[64..68].copy_from_slice(&p.characterisation_loaded_noise_sd_g.to_bits().to_le_bytes());
    bytes[68..72].copy_from_slice(&p.characterisation_average_settle_ms.to_le_bytes());
    bytes[72..76].copy_from_slice(&p.characterisation_maximum_settle_ms.to_le_bytes());
    let crc = crc32(&bytes[..CRC_OFFSET]);
    bytes[CRC_OFFSET..CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
    bytes
}

fn decode_record(bytes: &[u8; RECORD_SIZE]) -> Option<(u32, DeviceProperties)> {
    if &bytes[0..4] != MAGIC || u16::from_le_bytes(bytes[8..10].try_into().ok()?) != 2 {
        return None;
    }
    if u32::from_le_bytes(bytes[CRC_OFFSET..CRC_OFFSET + 4].try_into().ok()?)
        != crc32(&bytes[..CRC_OFFSET])
    {
        return None;
    }

    let read_f32 = |range: core::ops::Range<usize>| -> Option<f32> {
        Some(f32::from_bits(u32::from_le_bytes(
            bytes[range].try_into().ok()?,
        )))
    };
    let p = DeviceProperties {
        scale_factor_g_per_count: read_f32(12..16)?,
        tare_offset_counts: i32::from_le_bytes(bytes[16..20].try_into().ok()?),
        tare_noise_variance_counts2: i64::from_le_bytes(bytes[20..28].try_into().ok()?),
        tare_valid: bytes[28] == 1,
        calibration_last_reference_mass_g: read_f32(32..36)?,
        calibration_last_raw_delta_counts: read_f32(36..40)?,
        characterisation_reference_mass_g: read_f32(40..44)?,
        characterisation_cycles: bytes[44],
        characterisation_convergence_g: read_f32(48..52)?,
        characterisation_window_samples: u16::from_le_bytes(bytes[52..54].try_into().ok()?),
        characterisation_valid: bytes[54] == 1,
        characterisation_background_noise_sd_g: read_f32(56..60)?,
        characterisation_loaded_mean_g: read_f32(60..64)?,
        characterisation_loaded_noise_sd_g: read_f32(64..68)?,
        characterisation_average_settle_ms: u32::from_le_bytes(bytes[68..72].try_into().ok()?),
        characterisation_maximum_settle_ms: u32::from_le_bytes(bytes[72..76].try_into().ok()?),
    };
    if !p.scale_factor_g_per_count.is_finite() || p.scale_factor_g_per_count == 0.0 {
        return None;
    }
    Some((u32::from_le_bytes(bytes[4..8].try_into().ok()?), p))
}

fn same_values(a: &DeviceProperties, b: &DeviceProperties) -> bool {
    a.scale_factor_g_per_count.to_bits() == b.scale_factor_g_per_count.to_bits()
        && a.tare_offset_counts == b.tare_offset_counts
        && a.tare_noise_variance_counts2 == b.tare_noise_variance_counts2
        && a.tare_valid == b.tare_valid
        && a.calibration_last_reference_mass_g.to_bits()
            == b.calibration_last_reference_mass_g.to_bits()
        && a.calibration_last_raw_delta_counts.to_bits()
            == b.calibration_last_raw_delta_counts.to_bits()
        && a.characterisation_reference_mass_g.to_bits()
            == b.characterisation_reference_mass_g.to_bits()
        && a.characterisation_cycles == b.characterisation_cycles
        && a.characterisation_convergence_g.to_bits() == b.characterisation_convergence_g.to_bits()
        && a.characterisation_window_samples == b.characterisation_window_samples
        && a.characterisation_valid == b.characterisation_valid
        && a.characterisation_background_noise_sd_g.to_bits()
            == b.characterisation_background_noise_sd_g.to_bits()
        && a.characterisation_loaded_mean_g.to_bits() == b.characterisation_loaded_mean_g.to_bits()
        && a.characterisation_loaded_noise_sd_g.to_bits()
            == b.characterisation_loaded_noise_sd_g.to_bits()
        && a.characterisation_average_settle_ms == b.characterisation_average_settle_ms
        && a.characterisation_maximum_settle_ms == b.characterisation_maximum_settle_ms
}

fn generation_is_newer(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) > 0
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = !0u32;
    for byte in bytes {
        crc ^= *byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (0u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}
