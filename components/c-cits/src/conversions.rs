use c_its_parser::standards::cdd_1_3_1_1::its_container::{AltitudeValue, HeadingValue, SpeedValue, TimestampIts};

// Shamelessly stolen from https://github.com/consider-it/C-ITS-Parser
pub fn its_offset_ms(unix_time_ms: i64) -> u16 {
    if unix_time_ms >= 1_483_228_800_000 {
        // leap second introduced at 2016-12-31, so +5 since 2017-01-01
        5000
    } else if unix_time_ms >= 1_435_708_800_000 {
        // leap second introduced at 2015-06-30, so +4 since 2015-07-01
        4000
    } else if unix_time_ms >= 1_341_100_800_000 {
        // leap second introduced at 2012-06-30, so +3 since 2012-07-01
        3000
    } else if unix_time_ms >= 1_199_145_600_000 {
        // leap second introduced at 2008-12-31, so +2 since 2009-01-01
        2000
    } else if unix_time_ms >= 1_136_073_600_000 {
        // leap second introduced at 2005-12-31, so +1 since 2006-01-01
        1000
    } else {
        0
    }
}

pub fn unix_millis_to_its_time(unix_millis: i64) -> i64 {
    const ITS_EPOCH_UNIX_MS: i64 = 1_072_915_200_000; // UNIX timestamp of ITS epoch begin

    let its_time = unix_millis - ITS_EPOCH_UNIX_MS + i64::from(its_offset_ms(unix_millis));

    its_time
}

pub fn unix_millis_to_etsi_timestamp_its(unix_millis: i64) -> TimestampIts {
    TimestampIts(unix_millis_to_its_time(unix_millis) as u64)
}

pub fn coordinate_to_i32(coord: f64) -> i32 {
    core::f64::math::round_ties_even(coord * 1e7) as i32
}

const KNOTS_TO_MPS_DIV_100: f32 = 51.44;

pub fn knots_to_gnw_speed(speed: Option<f32>) -> i16 {
    let speed = match speed {
        Some(speed) => speed,
        None => return 0
    };

    // 15-bit signed integer
    const I15_MIN: i16 = i16::MIN >> 1;
    const I15_MAX: i16 = i16::MAX >> 1;

    let mps_div_100 = core::f32::math::round_ties_even(speed * KNOTS_TO_MPS_DIV_100);

    if mps_div_100 < I15_MIN as f32 {
        return I15_MIN
    } else if mps_div_100 > I15_MAX as f32 {
        return I15_MAX
    }

    mps_div_100 as i16
}

pub fn knots_to_etsi_speed_value(speed: Option<f32>) -> SpeedValue {
    let speed = match speed {
        Some(speed) => speed,
        None => return SpeedValue::unavailable()
    };

    // shouldn't be the case?
    if speed < 0.0 {
        return SpeedValue::unavailable()
    }

    // 14-bit unsigned integer, max value is "unavailable"
    const ETSI_SPEED_MAX: u16 = (u16::MAX >> 2) - 1;

    let mps_div_100 = core::f32::math::round_ties_even(speed * KNOTS_TO_MPS_DIV_100);

    if mps_div_100 > ETSI_SPEED_MAX as f32 {
        return SpeedValue(ETSI_SPEED_MAX)
    }

    SpeedValue(mps_div_100 as u16)
}

/// heading: heading in degrees
/// speed: speed in knots from GNSS
/// set heading to zero as "unknown value" if we are not moving
pub fn degrees_to_gnw_heading(heading: Option<f32>, speed: Option<f32>) -> u16 {
    let heading = match (heading, speed) {
        (Some(_), Some(speed)) if speed <= 0.1 => return 0,
        (Some(heading), Some(_)) => heading,
        (_, None) | (None, _) => return 0
    };

    if heading < 0.0 || heading >= 360.0 {
        return 0
    }

    let mut etsi_heading = core::f32::math::round_ties_even(heading * 10.0) as u16;
    if etsi_heading == 3600 {
        etsi_heading = 0;
    }

    etsi_heading
}

pub fn degrees_to_etsi_heading_value(heading: Option<f32>, speed: Option<f32>) -> HeadingValue {
    let heading = match (heading, speed) {
        (Some(_), Some(speed)) if speed <= 0.1 => return HeadingValue::unavailable(),
        (Some(heading), Some(_)) => heading,
        (_, None) | (None, _) => return HeadingValue::unavailable()
    };

    if heading < 0.0 || heading >= 360.0 {
        return HeadingValue::unavailable()
    }

    let mut etsi_heading = core::f32::math::round_ties_even(heading * 10.0) as u16;
    if etsi_heading == 3600 {
        etsi_heading = 0;
    }

    HeadingValue(etsi_heading)
}

pub fn meters_to_etsi_altitude_value(altitude: Option<f32>) -> AltitudeValue {
    let altitude = match altitude {
        Some(altitude) => altitude,
        None => return AltitudeValue(800_001)
    };

    if altitude <= -1000.0 {
        return AltitudeValue(-100_000)
    } else if altitude >= 8000.0 {
        return AltitudeValue(800_000)
    }

    AltitudeValue(core::f32::math::round_ties_even(altitude * 100.0) as i32)
}
