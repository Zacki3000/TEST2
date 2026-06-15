use alloc::boxed::Box;
use core::fmt::Write;
use c_its_parser::{EncodingRules, ItsMessage, standards::{cam_1_4_1::cam_pdu_descriptions::{BasicContainer, BasicVehicleContainerHighFrequency, CAM, CamParameters, CoopAwareness, GenerationDeltaTime, HighFrequencyContainer}, cdd_1_3_1_1::its_container::{AccelerationConfidence, Altitude, AltitudeConfidence, Curvature, CurvatureCalculationMode, CurvatureConfidence, CurvatureValue, DriveDirection, Heading, HeadingConfidence, HeadingValue, ItsPduHeader, Latitude, Longitude, LongitudinalAcceleration, LongitudinalAccelerationValue, PosConfidenceEllipse, ReferencePosition, SemiAxisLength, Speed, SpeedConfidence, StationID, VehicleLength, VehicleLengthConfidenceIndication, VehicleLengthValue, VehicleWidth, YawRate, YawRateConfidence, YawRateValue}}, transport::{BasicTransportBHeader, TransportHeader}};
use geonetworking::{Lifetime, Timestamp};

use crate::{conversions, esp_stdout, gnss::GnssState, gnw::{GnwBuilder, SrcPosBuilder}};

const DEFAULT_MAC: [u8; 6] = [0x02, 0xca, 0xfe, 0x12, 0x34, 0x56];

pub struct ItsSState {
    mac: [u8; 6],
    station_type: u8,
    mobile: bool
}

impl Default for ItsSState {
    fn default() -> Self {
        Self {
            mac: DEFAULT_MAC,
            station_type: Default::default(),
            mobile: true
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn create_its_s(mac: *const [u8; 6], station_type: u8, mobile: bool) -> *mut ItsSState {
    let mut state = Box::new(ItsSState::default());
    state.mac = unsafe { mac.read() };
    state.station_type = station_type;
    state.mobile = mobile;

    Box::leak(state) as _
}

#[unsafe(no_mangle)]
pub extern "C" fn build_cam(its_s_state: *mut ItsSState, gnss_state: *mut GnssState, time: i64, size: *mut usize) -> *mut u8 {
    let etsi_timestamp = conversions::unix_millis_to_etsi_timestamp_its(time);
    let its_s_state = unsafe { its_s_state.as_ref_unchecked() };
    let gnss_state = unsafe { gnss_state.as_ref_unchecked() };

    let pos = match &gnss_state.pos {
        Some(pos) => pos,
        None => {
            let _ = writeln!(esp_stdout::stdout(), "Not generating CAM: position unavailable");
            return core::ptr::null_mut()
        }
    };

    let gnw = GnwBuilder::new()
        .shb()
            .address_manually_configured(false)
            // TODO: This is not nice... Patch the GNW library to allow arbitrary types?
            // The underlying issue here is that not all station types that are defined in ITS are defined in GNW.
            // Maybe the correct behaviour would be to set another station type here and only set the actual one in ITS?
            .station_type(unsafe { core::mem::transmute(its_s_state.station_type) })
            .address(its_s_state.mac)
            .timestamp(Timestamp(etsi_timestamp.0 as u32))
            .lat_lon(pos.lat, pos.lon)
            // TODO: PAI
            .pai(true)
            .speed_knots(gnss_state.speed)
            .heading(gnss_state.hdg, gnss_state.speed)
            .done()
        .mobile(its_s_state.mobile)
        .lifetime(Lifetime(0b1_01))
        .mhl(1)
        .rhl(1)
        .build();

    let basic_container = BasicContainer::new(
        c_its_parser::standards::cdd_1_3_1_1::its_container::StationType(its_s_state.station_type),
        ReferencePosition {
            latitude: Latitude(conversions::coordinate_to_i32(pos.lat)),
            longitude: Longitude(conversions::coordinate_to_i32(pos.lon)),
            position_confidence_ellipse: PosConfidenceEllipse {
                semi_major_confidence: SemiAxisLength(4095),
                semi_minor_confidence: SemiAxisLength(4095),
                semi_major_orientation: HeadingValue::unavailable()
            },
            altitude: Altitude::new(conversions::meters_to_etsi_altitude_value(Some(pos.alt)), AltitudeConfidence::unavailable)
    });

    let high_frequency_container = HighFrequencyContainer::basicVehicleContainerHighFrequency(
        BasicVehicleContainerHighFrequency::new(
            Heading::new(conversions::degrees_to_etsi_heading_value(gnss_state.hdg, gnss_state.speed), HeadingConfidence(127)),
            Speed::new(conversions::knots_to_etsi_speed_value(gnss_state.speed), SpeedConfidence(127)),
            DriveDirection::unavailable,
            VehicleLength::new(VehicleLengthValue::unavailable(), VehicleLengthConfidenceIndication::unavailable),
            VehicleWidth::unavailable(),
            LongitudinalAcceleration::new(LongitudinalAccelerationValue::unavailable(), AccelerationConfidence(102)),
            Curvature::new(CurvatureValue::unavailable(), CurvatureConfidence::unavailable),
            CurvatureCalculationMode::unavailable,
            YawRate::new(YawRateValue::unavailable(), YawRateConfidence::unavailable),
            None,
            None,
            None,
            None,
            None,
            None,
            None
    ));

    let msg = ItsMessage::Cam {
        geonetworking: Some(gnw),
        transport: Some(TransportHeader::BtpB(BasicTransportBHeader {
            destination_port: 2001,
            destination_port_info: 0
        }).into()),
        etsi: CAM {
            header: ItsPduHeader {
                protocol_version: 2,
                message_id: 2,
                station_id: StationID(12345678)
            },
            cam: CoopAwareness {
                generation_delta_time: GenerationDeltaTime(etsi_timestamp.0 as u16),
                cam_parameters: CamParameters::new(basic_container, high_frequency_container, None, None)
            },
        }.into()
    };

    let vec = msg.encode(EncodingRules::UPER).unwrap();

    unsafe { *size = vec.len() }
    vec.leak().as_mut_ptr()
}
