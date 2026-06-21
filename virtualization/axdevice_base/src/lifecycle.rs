//! Device lifecycle operations.

use ax_errno::AxResult;

/// Optional lifecycle hooks implemented by stateful devices.
///
/// Devices register this capability only when they have meaningful lifecycle
/// state to reset, suspend, or resume.
pub trait DeviceLifecycle: Send + Sync {
    /// Resets the device to its power-on state.
    fn reset(&self) -> AxResult;

    /// Saves or quiesces device state before the VM is suspended.
    fn suspend(&self) -> AxResult;

    /// Restores device state after the VM is resumed.
    fn resume(&self) -> AxResult;
}
