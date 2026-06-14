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

//! LoongArch virtual interrupt backend.

use alloc::{collections::VecDeque, sync::Arc};

use ax_errno::{AxResult, ax_err};
use ax_kspin::SpinNoPreempt as Mutex;
use axdevice_base::{InterruptTriggerMode, IrqLineId, IrqSink};
use axvm_types::{InterruptVector, VCpuId, VMInterruptMode};

use super::{InterruptControllerOps, InterruptFabric, PendingInterrupt};

const BSP_VCPU_ID: VCpuId = 0;
const DEVICE_IRQ_START: usize = 2;
const DEVICE_IRQ_END: usize = 10;

#[derive(Clone, Copy)]
struct QueuedInterrupt {
    vcpu_id: VCpuId,
    interrupt: PendingInterrupt,
}

pub(crate) struct LoongArchInterruptBackend {
    vcpu_count: usize,
    pending: Mutex<VecDeque<QueuedInterrupt>>,
}

impl LoongArchInterruptBackend {
    fn new(vcpu_count: usize) -> AxResult<Self> {
        if vcpu_count == 0 {
            return ax_err!(
                InvalidInput,
                "a LoongArch interrupt backend requires at least one vCPU"
            );
        }
        Ok(Self {
            vcpu_count,
            pending: Mutex::new(VecDeque::new()),
        })
    }

    fn validate_device_line(line: IrqLineId) -> AxResult {
        if !(DEVICE_IRQ_START..DEVICE_IRQ_END).contains(&line.0) {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "invalid LoongArch device IRQ line {}; valid lines are {}..{}",
                    line.0, DEVICE_IRQ_START, DEVICE_IRQ_END
                )
            );
        }
        Ok(())
    }

    fn target_vcpu(&self, line: IrqLineId) -> AxResult<VCpuId> {
        Self::validate_device_line(line)?;
        if BSP_VCPU_ID >= self.vcpu_count {
            return ax_err!(
                BadState,
                format_args!(
                    "LoongArch IRQ line {} targets missing bootstrap vCPU {}",
                    line.0, BSP_VCPU_ID
                )
            );
        }
        Ok(BSP_VCPU_ID)
    }
}

impl IrqSink for LoongArchInterruptBackend {
    fn validate_line(&self, line: IrqLineId, trigger: InterruptTriggerMode) -> AxResult {
        self.target_vcpu(line)?;
        if trigger != InterruptTriggerMode::EdgeTriggered {
            return ax_err!(
                Unsupported,
                format_args!(
                    "LoongArch device IRQ line {} supports edge pulses only",
                    line.0
                )
            );
        }
        Ok(())
    }

    fn set_level(&self, line: IrqLineId, _asserted: bool) -> AxResult {
        Self::validate_device_line(line)?;
        ax_err!(
            Unsupported,
            format_args!(
                "LoongArch device IRQ line {} does not support level assert or lower",
                line.0
            )
        )
    }

    fn pulse(&self, line: IrqLineId) -> AxResult {
        let vcpu_id = self.target_vcpu(line)?;
        self.pending.lock().push_back(QueuedInterrupt {
            vcpu_id,
            interrupt: PendingInterrupt {
                vector: line.0,
                trigger: InterruptTriggerMode::EdgeTriggered,
            },
        });
        Ok(())
    }
}

impl InterruptControllerOps for LoongArchInterruptBackend {
    fn eoi(&self, _vcpu_id: VCpuId, vector: InterruptVector) -> AxResult {
        ax_err!(
            Unsupported,
            format_args!("LoongArch device IRQ vector {vector} has no generic EOI operation")
        )
    }

    fn forward_host_irq(&self, host_irq: usize) -> AxResult {
        self.pulse(IrqLineId(host_irq))
    }

    fn drain_pending(
        &self,
        vcpu_id: VCpuId,
        deliver: &mut dyn FnMut(PendingInterrupt) -> AxResult,
    ) -> AxResult {
        loop {
            let queued = {
                let mut pending = self.pending.lock();
                let Some(index) = pending.iter().position(|entry| entry.vcpu_id == vcpu_id) else {
                    return Ok(());
                };
                let Some(queued) = pending.remove(index) else {
                    return ax_err!(BadState, "pending LoongArch IRQ index disappeared");
                };
                queued
            };

            if let Err(err) = deliver(queued.interrupt) {
                self.pending.lock().push_front(queued);
                return Err(err);
            }
        }
    }
}

#[cfg(target_arch = "loongarch64")]
pub(crate) fn configure(mode: VMInterruptMode, vcpu_count: usize) -> AxResult<InterruptFabric> {
    if mode == VMInterruptMode::NoIrq {
        return Ok(InterruptFabric::new(mode));
    }

    let backend = Arc::new(LoongArchInterruptBackend::new(vcpu_count)?);
    let sink: Arc<dyn IrqSink> = backend.clone();
    let controller: Arc<dyn InterruptControllerOps> = backend;
    InterruptFabric::with_controller(mode, sink, controller)
}

#[cfg(test)]
mod tests {
    use alloc::{vec, vec::Vec};

    use ax_errno::AxError;
    use axdevice::IrqResolver;

    use super::*;

    #[test]
    fn legal_device_line_targets_the_bootstrap_vcpu() {
        let backend = Arc::new(LoongArchInterruptBackend::new(2).unwrap());
        let sink: Arc<dyn IrqSink> = backend.clone();
        let controller: Arc<dyn InterruptControllerOps> = backend.clone();
        let fabric =
            InterruptFabric::with_controller(VMInterruptMode::Emulated, sink, controller).unwrap();
        let line = fabric
            .resolve_irq(DEVICE_IRQ_START, InterruptTriggerMode::EdgeTriggered)
            .unwrap();
        line.pulse().unwrap();

        let mut secondary = Vec::new();
        backend
            .drain_pending(1, &mut |interrupt| {
                secondary.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert!(secondary.is_empty());

        let mut bootstrap = Vec::new();
        backend
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                bootstrap.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert_eq!(
            bootstrap,
            vec![PendingInterrupt {
                vector: DEVICE_IRQ_START,
                trigger: InterruptTriggerMode::EdgeTriggered,
            }]
        );
    }

    #[test]
    fn rejects_backend_without_vcpus() {
        assert_eq!(
            LoongArchInterruptBackend::new(0).err(),
            Some(AxError::InvalidInput)
        );
    }

    #[test]
    fn rejects_out_of_range_device_lines() {
        let backend = LoongArchInterruptBackend::new(1).unwrap();
        assert_eq!(
            backend.pulse(IrqLineId(DEVICE_IRQ_START - 1)),
            Err(AxError::InvalidInput)
        );
        assert_eq!(
            backend.pulse(IrqLineId(DEVICE_IRQ_END)),
            Err(AxError::InvalidInput)
        );
    }

    #[test]
    fn rejects_level_lines_during_resolution() {
        let backend = Arc::new(LoongArchInterruptBackend::new(1).unwrap());
        let fabric =
            InterruptFabric::with_sink(VMInterruptMode::Emulated, backend.clone()).unwrap();

        assert_eq!(
            fabric
                .resolve_irq(DEVICE_IRQ_START, InterruptTriggerMode::LevelTriggered)
                .err(),
            Some(AxError::Unsupported)
        );
        assert_eq!(
            backend.set_level(IrqLineId(DEVICE_IRQ_START), true),
            Err(AxError::Unsupported)
        );
        assert_eq!(
            backend.set_level(IrqLineId(DEVICE_IRQ_START), false),
            Err(AxError::Unsupported)
        );
    }

    #[test]
    fn failed_delivery_keeps_interrupt_pending() {
        let backend = LoongArchInterruptBackend::new(1).unwrap();
        backend.pulse(IrqLineId(DEVICE_IRQ_START + 1)).unwrap();
        assert_eq!(
            backend.drain_pending(BSP_VCPU_ID, &mut |_| Err(AxError::WouldBlock)),
            Err(AxError::WouldBlock)
        );

        let mut delivered = Vec::new();
        backend
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                delivered.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].vector, DEVICE_IRQ_START + 1);
    }
}
