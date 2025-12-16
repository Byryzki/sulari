use chrono::{Datelike, TimeZone, Timelike, Utc};
use serde::{Deserialize, Serialize};

/// `ssmarshal` compatible DateTime
///
/// `ssmarshal` cannot serialize chrono DateTime, therefore we provide our own wrapper type.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SDateTime {
    year: i32,
    month: u32,
    day: u32,
    hour: u32,
    minute: u32,
    second: u32,
    nanoseconds: u32,
}

impl From<chrono::DateTime<Utc>> for SDateTime {
    fn from(dt: chrono::DateTime<Utc>) -> Self {
        Self {
            year: dt.year(),
            month: dt.month(),
            day: dt.day(),
            hour: dt.hour(),
            minute: dt.minute(),
            second: dt.second(),
            nanoseconds: dt.nanosecond(),
        }
    }
}

impl From<SDateTime> for chrono::DateTime<Utc> {
    fn from(value: SDateTime) -> Self {
        Utc.with_ymd_and_hms(
            value.year,
            value.month,
            value.day,
            value.hour,
            value.minute,
            value.second,
        )
        .unwrap()
    }
}

impl PartialOrd for SDateTime {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        match self.year.partial_cmp(&other.year) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.month.partial_cmp(&other.month) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.day.partial_cmp(&other.day) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.hour.partial_cmp(&other.hour) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.minute.partial_cmp(&other.minute) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        match self.second.partial_cmp(&other.second) {
            Some(core::cmp::Ordering::Equal) => {}
            ord => return ord,
        }
        self.nanoseconds.partial_cmp(&other.nanoseconds)
    }
}
