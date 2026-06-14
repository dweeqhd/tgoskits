// Copyright 2025 The Axvisor Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! VM-owned interrupt line routing.

use alloc::sync::Arc;

use ax_errno::{AxResult, ax_err};
use axdevice::IrqResolver;
use axdevice_base::{InterruptTriggerMode, IrqLine, IrqLineId, IrqSink};
use axvm_types::{InterruptVector, VCpuId, VMInterruptMode};

#[cfg(any(target_arch = "aarch64", test))]
pub(crate) mod aarch64;
#[cfg(target_arch = "riscv64")]
pub(crate) mod riscv;
#[cfg(target_arch = "x86_64")]
pub(crate) mod x86;

/// An interrupt routed by a VM interrupt controller and ready for vCPU delivery.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PendingInterrupt {
    /// Guest interrupt vector.
    pub vector: usize,
    /// Trigger mode selected by the interrupt controller.
    pub trigger: InterruptTriggerMode,
}

/// Controller operations that require a safe VM or vCPU runtime context.
pub trait InterruptControllerOps: Send + Sync {
    /// Process an EOI broadcast from one vCPU.
    fn eoi(&self, vcpu_id: VCpuId, vector: InterruptVector) -> AxResult;

    /// Map a host interrupt number to a VM-local interrupt line.
    fn forward_host_irq(&self, host_irq: usize) -> AxResult;

    /// Deliver pending routed interrupts for one vCPU.
    fn drain_pending(
        &self,
        vcpu_id: VCpuId,
        deliver: &mut dyn FnMut(PendingInterrupt) -> AxResult,
    ) -> AxResult;
}

/// Resolves device interrupt lines against one VM's interrupt backend.
///
/// The fabric owns only the backend capability. It never owns or references the
/// containing VM, so devices can retain [`IrqLine`] objects without creating an
/// `AxVM -> device -> IRQ -> AxVM` reference cycle.
pub struct InterruptFabric {
    mode: VMInterruptMode,
    sink: Option<Arc<dyn IrqSink>>,
    controller: Option<Arc<dyn InterruptControllerOps>>,
}

impl InterruptFabric {
    /// Creates a fabric without an interrupt backend.
    pub const fn new(mode: VMInterruptMode) -> Self {
        Self {
            mode,
            sink: None,
            controller: None,
        }
    }

    /// Creates a fabric that routes lines to `sink`.
    pub fn with_sink(mode: VMInterruptMode, sink: Arc<dyn IrqSink>) -> AxResult<Self> {
        if mode == VMInterruptMode::NoIrq {
            return ax_err!(
                InvalidInput,
                "a VM configured with interrupt_mode=no_irq cannot install an IRQ backend"
            );
        }
        Ok(Self {
            mode,
            sink: Some(sink),
            controller: None,
        })
    }

    /// Creates a fabric with line signaling and controller runtime operations.
    pub fn with_controller(
        mode: VMInterruptMode,
        sink: Arc<dyn IrqSink>,
        controller: Arc<dyn InterruptControllerOps>,
    ) -> AxResult<Self> {
        if mode == VMInterruptMode::NoIrq {
            return ax_err!(
                InvalidInput,
                "a VM configured with interrupt_mode=no_irq cannot install an IRQ backend"
            );
        }
        Ok(Self {
            mode,
            sink: Some(sink),
            controller: Some(controller),
        })
    }

    /// Returns the VM interrupt mode associated with this fabric.
    pub const fn mode(&self) -> VMInterruptMode {
        self.mode
    }

    /// Returns whether this fabric has an interrupt backend.
    pub const fn has_backend(&self) -> bool {
        self.sink.is_some()
    }

    /// Returns whether this fabric has controller runtime operations.
    pub const fn has_controller(&self) -> bool {
        self.controller.is_some()
    }

    fn sink_for_line(&self, line: usize) -> AxResult<&Arc<dyn IrqSink>> {
        let Some(sink) = &self.sink else {
            if self.mode == VMInterruptMode::NoIrq {
                return ax_err!(
                    InvalidInput,
                    format_args!("cannot signal IRQ line {line}: the VM interrupt mode is NoIrq")
                );
            }
            return ax_err!(
                Unsupported,
                format_args!("cannot signal IRQ line {line}: no VM interrupt backend is installed")
            );
        };
        Ok(sink)
    }

    /// Sets the asserted state of a VM-local interrupt line.
    pub fn set_level(&self, line: usize, asserted: bool) -> AxResult {
        self.sink_for_line(line)?
            .set_level(IrqLineId(line), asserted)
    }

    /// Delivers one pulse on a VM-local interrupt line.
    pub fn pulse(&self, line: usize) -> AxResult {
        self.sink_for_line(line)?.pulse(IrqLineId(line))
    }

    /// Processes an EOI broadcast for this VM.
    pub fn eoi(&self, vcpu_id: VCpuId, vector: InterruptVector) -> AxResult {
        let Some(controller) = &self.controller else {
            return Ok(());
        };
        controller.eoi(vcpu_id, vector)
    }

    /// Forwards one host interrupt through this VM's controller backend.
    pub fn forward_host_irq(&self, host_irq: usize) -> AxResult {
        let Some(controller) = &self.controller else {
            return ax_err!(
                Unsupported,
                format_args!(
                    "cannot forward host IRQ {host_irq}: no VM interrupt controller is installed"
                )
            );
        };
        controller.forward_host_irq(host_irq)
    }

    /// Delivers pending routed interrupts for one vCPU.
    pub fn drain_pending(
        &self,
        vcpu_id: VCpuId,
        mut deliver: impl FnMut(PendingInterrupt) -> AxResult,
    ) -> AxResult {
        let Some(controller) = &self.controller else {
            return Ok(());
        };
        controller.drain_pending(vcpu_id, &mut deliver)
    }

    pub(crate) fn validate_mode(&self, mode: VMInterruptMode) -> AxResult {
        if self.mode != mode {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "interrupt fabric mode {:?} does not match VM interrupt mode {:?}",
                    self.mode, mode
                )
            );
        }
        Ok(())
    }
}

impl Default for InterruptFabric {
    fn default() -> Self {
        Self::new(VMInterruptMode::NoIrq)
    }
}

impl IrqResolver for InterruptFabric {
    fn resolve_irq(&self, line: usize, trigger: InterruptTriggerMode) -> AxResult<IrqLine> {
        Ok(IrqLine::new(
            IrqLineId(line),
            trigger,
            self.sink_for_line(line)?.clone(),
        ))
    }
}
