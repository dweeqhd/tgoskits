//! VM interrupt router facade.

use ax_errno::AxResult;
use axdevice_base::{IrqLineId, MsiMessage};
use axvm_types::{InterruptVector, VCpuId};

use super::InterruptFabric;

/// Source that completed or acknowledged an interrupt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IrqSource {
    /// A VM-local line interrupt.
    Line(IrqLineId),
    /// A message-signaled interrupt.
    Msi(MsiMessage),
    /// A controller-originated vector without a stable device line.
    Vector(InterruptVector),
}

/// A routed MSI vector owned by one VM.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MsiRoute {
    /// Message value accepted by the interrupt router.
    pub message: MsiMessage,
    /// Target vCPU for the vector.
    pub target_vcpu: VCpuId,
    /// Guest interrupt vector.
    pub vector: InterruptVector,
}

/// Architecture-independent interrupt routing operations.
pub trait InterruptRouter {
    /// Raises a level-triggered line.
    fn raise(&self, line: IrqLineId) -> AxResult;

    /// Lowers a level-triggered line.
    fn lower(&self, line: IrqLineId) -> AxResult;

    /// Pulses an edge-triggered line.
    fn pulse_line(&self, line: IrqLineId) -> AxResult;

    /// Delivers an MSI/MSI-X message.
    fn msi(&self, message: MsiMessage) -> AxResult;

    /// Broadcasts end-of-interrupt state to the backend.
    fn eoi_source(&self, vcpu_id: VCpuId, source: IrqSource, vector: InterruptVector) -> AxResult;
}

impl InterruptRouter for InterruptFabric {
    fn raise(&self, line: IrqLineId) -> AxResult {
        self.set_level(line.0, true)
    }

    fn lower(&self, line: IrqLineId) -> AxResult {
        self.set_level(line.0, false)
    }

    fn pulse_line(&self, line: IrqLineId) -> AxResult {
        self.pulse(line.0)
    }

    fn msi(&self, message: MsiMessage) -> AxResult {
        self.deliver_msi(message)
    }

    fn eoi_source(&self, vcpu_id: VCpuId, _source: IrqSource, vector: InterruptVector) -> AxResult {
        self.eoi(vcpu_id, vector)
    }
}
