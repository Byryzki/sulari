use crate::SDateTime;
use serde::{Deserialize, Serialize};

/// Commands (possibly) supported by the device
///
/// Do not change the ABI (i.e., anything about the data structure), as the staff test code uses
/// exactly this binary layout to test the device.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[repr(C)]
pub enum Command {
    /// Reset application and hardware to initial state
    Reset,
    /// Return the device-internal counter
    Counter,
    /// Sets or clears the current time for the device
    SetDateTime(Option<SDateTime>),
    /// Actuates specified functionality immediately
    Immediate(Funct),
    /// Schedules specified functionality ([Funct]) to start at specified time
    ///
    /// Once something is scheduled, it cannot be removed from schedule.
    Schedule(Funct, SDateTime),
}

/// Functionality
///
/// Used as part of [Command] to specify what functionality should be triggered.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[repr(C)]
pub enum Funct {
    /// Increment device-internal counter
    Increment,
    /// Start blinking a led with specified period
    EnableBlink {
        /// Led should switch states at this interval
        period_ms: u64,
    },
    /// Stop the blinking and turn off the led
    DisableBlink,
    /// Turn on the RGB
    ///
    /// The led should show the time of day in UTC-0
    EnableRgb,
    /// Turn off the RGB led
    DisableRgb,
}
