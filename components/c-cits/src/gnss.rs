use alloc::{boxed::Box, slice};

use nmea::{ParseResult, sentences::{FixType, GgaData, VtgData}};

use crate::esp_stdout;


unsafe extern "C" {
    /// cbindgen:ignore
    fn gnss_handle_time(gnss_time_unix_seconds: i64);
}

pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32
}

pub struct GnssState {
    pub pos: Option<Position>,
    pub speed: Option<f32>,
    pub hdg: Option<f32>
}

impl GnssState {
    fn handle_gga(self: &mut Self, data: GgaData) {
        if matches!(data.fix_type, Some(FixType::Invalid) | None) {
            self.pos = None;
            return;
        }

        if let (Some(lat), Some(lon), Some(alt)) = (data.latitude, data.longitude, data.altitude) {
            self.pos = Some(Position {
                lat, lon, alt
            });
        } else {
            self.pos = None;
            return;
        }
    }

    fn handle_vtg(self: &mut Self, data: VtgData) {
        self.speed = data.speed_over_ground;
        self.hdg = data.true_course;
    }
}

impl Default for GnssState {
    fn default() -> Self {
        Self { pos: Default::default(), speed: Default::default(), hdg: Default::default() }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn gnss_new() -> *mut GnssState {
    Box::leak(Box::new(GnssState::default())) as _
}

#[unsafe(no_mangle)]
pub extern "C" fn gnss_handle_nmea(state: *mut GnssState, data: *const u8, length: usize) -> u32 {
    let state = unsafe { state.as_mut_unchecked() };
    let data_slice = unsafe { slice::from_raw_parts(data, length) };

    //let _ = writeln!(esp_stdout::stdout(), "{:?}", str::from_utf8(data_slice));
    let result = nmea::parse_bytes(data_slice);

    //let _ = writeln!(esp_stdout::stdout(), "{:?}", result);

    let parse_result = match result {
        Err(_) => {
            //let _ = writeln!(esp_stdout::stdout(), "Error parsing NMEA");
            return 1
        },
        Ok(parse_result) => parse_result
    };

    match parse_result {
        ParseResult::GGA(gga) => state.handle_gga(gga),
        ParseResult::VTG(vtg) => state.handle_vtg(vtg),
        ParseResult::ZDA(zda) => {
            match zda.utc_date_time() {
                Some(dt) => unsafe { gnss_handle_time(dt.and_utc().timestamp()); }
                None => ()
            }
        }
        _ => ()
    }

    0
}
