use c_its_parser::Packet;
use geonetworking::{Address, BasicHeader, Bits, BroadcastType, CommonHeader, ExtendedHeader, HeaderType, Lifetime, LongPositionVector, NextAfterBasic, NextAfterCommon, SingleHopBroadcast, StationType, Timestamp, TrafficClass, bits};

use crate::conversions;

const DUMMY_BUFFER: [u8; 0] = [];

trait HeaderAccess {
    fn basic_header_mut(self: &mut Self) -> &mut BasicHeader;
    fn common_header_mut(self: &mut Self) -> &mut CommonHeader;
    fn extended_header_mut(self: &mut Self) -> &mut Option<ExtendedHeader>;
}

impl HeaderAccess for Packet<'_> {
    fn basic_header_mut(self: &mut Self) -> &mut BasicHeader {
        match &mut *self {
            Packet::Unsecured { basic, .. } => basic,
            Packet::Secured { basic, .. } => basic
        }
    }

    fn common_header_mut(self: &mut Self) -> &mut CommonHeader {
        match &mut *self {
            Packet::Unsecured { common, .. } => common,
            Packet::Secured { common, .. } => common
        }
    }

    fn extended_header_mut(self: &mut Self) -> &mut Option<ExtendedHeader> {
        match &mut *self {
            Packet::Unsecured { extended, .. } => extended,
            Packet::Secured { extended, .. } => extended
        }
    }
}

pub trait SrcPosBuilder<'input> {
    fn address_manually_configured(self: Self, m: bool) -> Self;
    fn station_type(self: Self, station_type: StationType) -> Self;
    fn address(self: Self, address: [u8; 6]) -> Self;
    fn timestamp(self: Self, timestamp: Timestamp) -> Self;
    fn lat_lon(self: Self, lat: f64, lon: f64) -> Self;
    fn pai(self: Self, pai: bool) -> Self;
    fn speed_knots(self: Self, speed: Option<f32>) -> Self;
    fn heading(self: Self, heading: Option<f32>, speed: Option<f32>) -> Self;
    fn mdd(self: Self, mdd: [u8; 4]) -> Self;
    fn done(self: Self) -> GnwBuilder<'input>;
}

pub struct GnwBuilder<'input>(Packet<'input>);

impl<'input> GnwBuilder<'input> {
    pub fn new() -> Self {
        Self(Packet::Unsecured {
            basic: BasicHeader {
                version: 1,
                next_header: NextAfterBasic::CommonHeader,
                reserved: bits![0, 0, 0, 0, 0, 0, 0, 0],
                lifetime: Lifetime(0b1_01),
                remaining_hop_limit: 1
            },
            common: CommonHeader {
                next_header: NextAfterCommon::BTPB,
                reserved_1: bits![0, 0, 0, 0],
                header_type_and_subtype: HeaderType::TopologicallyScopedBroadcast(BroadcastType::SingleHop),
                traffic_class: TrafficClass {
                    store_carry_forward: false,
                    channel_offload: false,
                    traffic_class_id: 2
                },
                flags: bits![0, 0, 0, 0, 0, 0, 0, 0],
                payload_length: 0,
                maximum_hop_limit: 1,
                reserved_2: bits![0, 0, 0, 0, 0, 0, 0, 0]
            },
            extended: None,
            payload: &DUMMY_BUFFER
        })
    }

    pub fn lifetime(mut self: Self, lifetime: Lifetime) -> Self {
        self.0.basic_header_mut().lifetime = lifetime;

        self
    }

    pub fn rhl(mut self: Self, rhl: u8) -> Self {
        self.0.basic_header_mut().remaining_hop_limit = rhl;

        self
    }

    pub fn mhl(mut self: Self, mhl: u8) -> Self {
        self.0.common_header_mut().maximum_hop_limit = mhl;

        self
    }

    pub fn mobile(mut self: Self, mobile: bool) -> Self {
        self.0.common_header_mut().flags = bits![mobile as u8, 0, 0, 0, 0, 0, 0, 0];

        self
    }

    pub fn shb(mut self: Self) -> ShbBuilder<'input> {
        self.0.basic_header_mut().remaining_hop_limit = 1;
        self.0.common_header_mut().header_type_and_subtype = HeaderType::TopologicallyScopedBroadcast(BroadcastType::SingleHop);

        ShbBuilder::new(self.0)
    }

    pub fn build(self: Self) -> Packet<'input> {
        self.0
    }
}


pub struct ShbBuilder<'input>(Packet<'input>, SingleHopBroadcast);

impl ShbBuilder<'_> {
    fn new(packet: Packet<'_>) -> ShbBuilder<'_> {
        let shb_header = SingleHopBroadcast {
            source_position_vector: LongPositionVector {
                gn_address: Address {
                    manually_configured: false,
                    station_type: StationType::Unknown,
                    reserved: bits![0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    address: [0x02, 0xca, 0xfe, 0x12, 0x34, 0x56]
                },
                timestamp: Timestamp(0),
                latitude: 0,
                longitude: 0,
                position_accuracy: false,
                speed: 0,
                heading: 0
            },
            media_dependent_data: [0; 4]
        };

        ShbBuilder(packet, shb_header)
    }
}

impl<'input> SrcPosBuilder<'input> for ShbBuilder<'input> {
    fn address_manually_configured(mut self: Self, m: bool) -> Self {
        self.1.source_position_vector.gn_address.manually_configured = m;

        self
    }

    fn station_type(mut self: Self, station_type: StationType) -> Self {
        self.1.source_position_vector.gn_address.station_type = station_type;

        self
    }

    fn address(mut self: Self, address: [u8; 6]) -> Self {
        self.1.source_position_vector.gn_address.address = address;

        self
    }

    fn timestamp(mut self: Self, timestamp: Timestamp) -> Self {
        self.1.source_position_vector.timestamp = timestamp;

        self
    }

    fn lat_lon(mut self: Self, lat: f64, lon: f64) -> Self {
        self.1.source_position_vector.latitude = conversions::coordinate_to_i32(lat);
        self.1.source_position_vector.longitude = conversions::coordinate_to_i32(lon);

        self
    }

    fn pai(mut self: Self, pai: bool) -> Self {
        self.1.source_position_vector.position_accuracy = pai;

        self
    }

    fn speed_knots(mut self: Self, speed: Option<f32>) -> Self {
        self.1.source_position_vector.speed = conversions::knots_to_gnw_speed(speed);

        self
    }

    fn heading(mut self: Self, heading: Option<f32>, speed: Option<f32>) -> Self {
        self.1.source_position_vector.heading = conversions::degrees_to_gnw_heading(heading, speed);

        self
    }

    fn mdd(mut self: Self, mdd: [u8; 4]) -> Self {
        self.1.media_dependent_data = mdd;

        self
    }

    fn done(mut self: Self) -> GnwBuilder<'input> {
        match &mut self.0 {
            Packet::Unsecured { extended, .. } => {
                *extended = Some(ExtendedHeader::SHB(self.1));
            },
            _ => todo!()
        }

        GnwBuilder(self.0)
    }
}
