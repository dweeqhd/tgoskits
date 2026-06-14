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

//! AArch64 virtual GIC interrupt backend and GPPT device factories.

use alloc::{collections::VecDeque, vec, vec::Vec};

use ax_errno::{AxError, AxResult, ax_err};
use ax_kspin::SpinNoPreempt as Mutex;
use axdevice_base::{InterruptTriggerMode, IrqLineId, IrqSink};
use axvm_types::{InterruptVector, VCpuId};

use super::{InterruptControllerOps, PendingInterrupt};

const BSP_VCPU_ID: VCpuId = 0;
const SGI_END: usize = 16;
#[cfg(test)]
const PPI_END: usize = 32;
const SPECIAL_INTERRUPT_START: usize = 1020;

#[derive(Clone, Copy)]
struct QueuedInterrupt {
    line: IrqLineId,
    vcpu_id: VCpuId,
    interrupt: PendingInterrupt,
}

struct Aarch64InterruptState {
    asserted: Vec<bool>,
    pending: VecDeque<QueuedInterrupt>,
}

pub(crate) struct Aarch64InterruptBackend {
    vcpu_count: usize,
    state: Mutex<Aarch64InterruptState>,
}

impl Aarch64InterruptBackend {
    fn new(vcpu_count: usize) -> AxResult<Self> {
        if vcpu_count == 0 {
            return ax_err!(
                InvalidInput,
                "an AArch64 interrupt backend requires at least one vCPU"
            );
        }
        Ok(Self {
            vcpu_count,
            state: Mutex::new(Aarch64InterruptState {
                asserted: vec![false; SPECIAL_INTERRUPT_START],
                pending: VecDeque::new(),
            }),
        })
    }

    fn validate_line(&self, line: IrqLineId) -> AxResult {
        if line.0 < SGI_END {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "AArch64 IRQ line {} is an SGI; device lines must be a PPI or SPI",
                    line.0
                )
            );
        }
        if line.0 >= SPECIAL_INTERRUPT_START {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "invalid AArch64 IRQ line {}; valid device lines are {}..{}",
                    line.0, SGI_END, SPECIAL_INTERRUPT_START
                )
            );
        }
        Ok(())
    }

    fn target_vcpu(&self, line: IrqLineId) -> AxResult<VCpuId> {
        self.validate_line(line)?;

        // The current IRQ-line model has no affinity field. Preserve the
        // existing single-target behavior by routing both PPIs and SPIs to the
        // bootstrap vCPU until the configuration model can describe affinity.
        if BSP_VCPU_ID >= self.vcpu_count {
            return ax_err!(
                BadState,
                format_args!(
                    "AArch64 IRQ line {} targets missing bootstrap vCPU {}",
                    line.0, BSP_VCPU_ID
                )
            );
        }
        Ok(BSP_VCPU_ID)
    }

    fn queue_interrupt(&self, line: IrqLineId, trigger: InterruptTriggerMode) -> AxResult {
        let vcpu_id = self.target_vcpu(line)?;
        self.state.lock().pending.push_back(QueuedInterrupt {
            line,
            vcpu_id,
            interrupt: PendingInterrupt {
                vector: line.0,
                trigger,
            },
        });
        Ok(())
    }
}

impl IrqSink for Aarch64InterruptBackend {
    fn set_level(&self, line: IrqLineId, asserted: bool) -> AxResult {
        let vcpu_id = self.target_vcpu(line)?;
        let mut state = self.state.lock();
        if state.asserted[line.0] == asserted {
            return Ok(());
        }

        state.asserted[line.0] = asserted;
        if asserted {
            state.pending.push_back(QueuedInterrupt {
                line,
                vcpu_id,
                interrupt: PendingInterrupt {
                    vector: line.0,
                    trigger: InterruptTriggerMode::LevelTriggered,
                },
            });
        } else {
            state.pending.retain(|entry| entry.line != line);
        }
        Ok(())
    }

    fn pulse(&self, line: IrqLineId) -> AxResult {
        self.queue_interrupt(line, InterruptTriggerMode::EdgeTriggered)
    }
}

impl InterruptControllerOps for Aarch64InterruptBackend {
    fn eoi(&self, _vcpu_id: VCpuId, _vector: InterruptVector) -> AxResult {
        // GICH/ICH list-register state is maintained by hardware. Asserted
        // level lines stay queued and are retried before later VM entries.
        Ok(())
    }

    fn forward_host_irq(&self, host_irq: usize) -> AxResult {
        self.pulse(IrqLineId(host_irq))
    }

    fn drain_pending(
        &self,
        vcpu_id: VCpuId,
        deliver: &mut dyn FnMut(PendingInterrupt) -> AxResult,
    ) -> AxResult {
        let attempts = self
            .state
            .lock()
            .pending
            .iter()
            .filter(|entry| entry.vcpu_id == vcpu_id)
            .count();

        for _ in 0..attempts {
            let queued = {
                let mut state = self.state.lock();
                let Some(index) = state
                    .pending
                    .iter()
                    .position(|entry| entry.vcpu_id == vcpu_id)
                else {
                    return Ok(());
                };
                let Some(queued) = state.pending.remove(index) else {
                    return ax_err!(BadState, "pending AArch64 IRQ index disappeared");
                };
                queued
            };

            match deliver(queued.interrupt) {
                Ok(()) => {
                    if queued.interrupt.trigger == InterruptTriggerMode::LevelTriggered {
                        let mut state = self.state.lock();
                        if state.asserted[queued.line.0] {
                            state.pending.push_back(queued);
                        }
                    }
                }
                Err(AxError::WouldBlock) => {
                    self.state.lock().pending.push_front(queued);
                    return Err(AxError::WouldBlock);
                }
                Err(err) => return Err(err),
            }
        }
        Ok(())
    }
}

#[cfg(target_arch = "aarch64")]
mod platform {
    use alloc::sync::Arc;

    use ax_memory_addr::PhysAddr;
    use axdevice::{
        DeviceBuildContext, DeviceBundle, DeviceFactory, DeviceFactoryRegistry, DeviceRegistration,
    };
    use axvm_types::{EmulatedDeviceConfig, EmulatedDeviceType, VMInterruptMode};

    use super::*;
    use crate::irq::InterruptFabric;

    struct GpptRedistributorFactory;

    impl DeviceFactory for GpptRedistributorFactory {
        fn device_type(&self) -> EmulatedDeviceType {
            EmulatedDeviceType::GPPTRedistributor
        }

        fn build(
            &self,
            config: &EmulatedDeviceConfig,
            _context: &DeviceBuildContext<'_>,
        ) -> AxResult<DeviceBundle> {
            let [cpu_count, stride, first_pcpu_id] = config.cfg_list.as_slice() else {
                return ax_err!(
                    InvalidInput,
                    format_args!(
                        "GPPT redistributor device '{}' requires cpu-count, stride, and \
                         first-pCPU-id",
                        config.name
                    )
                );
            };
            if *cpu_count == 0 || *stride == 0 {
                return ax_err!(
                    InvalidInput,
                    format_args!(
                        "GPPT redistributor device '{}' requires non-zero cpu-count and stride",
                        config.name
                    )
                );
            }
            validate_mmio_range(config, "GPPT redistributor")?;

            let mut bundle = DeviceBundle::new();
            for index in 0..*cpu_count {
                let offset = index.checked_mul(*stride).ok_or(AxError::InvalidInput)?;
                let address = config
                    .base_gpa
                    .checked_add(offset)
                    .ok_or(AxError::InvalidInput)?;
                address
                    .checked_add(config.length)
                    .ok_or(AxError::InvalidInput)?;
                let pcpu_id = first_pcpu_id
                    .checked_add(index)
                    .ok_or(AxError::InvalidInput)?;

                bundle.push(DeviceRegistration::Mmio(Arc::new(
                    arm_vgic::v3::vgicr::VGicR::new(address.into(), Some(config.length), pcpu_id),
                )));
            }
            Ok(bundle)
        }
    }

    struct GpptDistributorFactory;

    impl DeviceFactory for GpptDistributorFactory {
        fn device_type(&self) -> EmulatedDeviceType {
            EmulatedDeviceType::GPPTDistributor
        }

        fn build(
            &self,
            config: &EmulatedDeviceConfig,
            _context: &DeviceBuildContext<'_>,
        ) -> AxResult<DeviceBundle> {
            validate_mmio_range(config, "GPPT distributor")?;
            if !config.cfg_list.is_empty() {
                return ax_err!(
                    InvalidInput,
                    format_args!(
                        "GPPT distributor device '{}' requires an empty config list",
                        config.name
                    )
                );
            }
            Ok(
                DeviceRegistration::Mmio(Arc::new(arm_vgic::v3::vgicd::VGicD::new(
                    config.base_gpa.into(),
                    Some(config.length),
                )))
                .into(),
            )
        }
    }

    struct GpptItsFactory;

    impl DeviceFactory for GpptItsFactory {
        fn device_type(&self) -> EmulatedDeviceType {
            EmulatedDeviceType::GPPTITS
        }

        fn build(
            &self,
            config: &EmulatedDeviceConfig,
            _context: &DeviceBuildContext<'_>,
        ) -> AxResult<DeviceBundle> {
            validate_mmio_range(config, "GPPT ITS")?;
            let [host_gits_base] = config.cfg_list.as_slice() else {
                return ax_err!(
                    InvalidInput,
                    format_args!(
                        "GPPT ITS device '{}' requires exactly one host-GITS-base argument",
                        config.name
                    )
                );
            };
            Ok(
                DeviceRegistration::Mmio(Arc::new(arm_vgic::v3::gits::Gits::new(
                    config.base_gpa.into(),
                    Some(config.length),
                    PhysAddr::from_usize(*host_gits_base),
                    false,
                )))
                .into(),
            )
        }
    }

    fn validate_mmio_range(config: &EmulatedDeviceConfig, device_type: &str) -> AxResult {
        if config.length == 0 || config.base_gpa.checked_add(config.length).is_none() {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "{device_type} device '{}' has an invalid MMIO range",
                    config.name
                )
            );
        }
        Ok(())
    }

    pub(crate) fn configure(
        factories: &mut DeviceFactoryRegistry,
        mode: VMInterruptMode,
        vcpu_count: usize,
    ) -> AxResult<InterruptFabric> {
        factories.register(Arc::new(GpptRedistributorFactory))?;
        factories.register(Arc::new(GpptDistributorFactory))?;
        factories.register(Arc::new(GpptItsFactory))?;

        if mode == VMInterruptMode::NoIrq {
            return Ok(InterruptFabric::new(mode));
        }

        let backend = Arc::new(Aarch64InterruptBackend::new(vcpu_count)?);
        let sink: Arc<dyn IrqSink> = backend.clone();
        let controller: Arc<dyn InterruptControllerOps> = backend;
        InterruptFabric::with_controller(mode, sink, controller)
    }
}

#[cfg(target_arch = "aarch64")]
pub(crate) use platform::configure;

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    #[test]
    fn rejects_sgis_and_special_interrupt_ids() {
        let backend = Aarch64InterruptBackend::new(1).unwrap();
        assert_eq!(
            backend.pulse(IrqLineId(SGI_END - 1)),
            Err(AxError::InvalidInput)
        );
        assert_eq!(
            backend.pulse(IrqLineId(SPECIAL_INTERRUPT_START)),
            Err(AxError::InvalidInput)
        );
    }

    #[test]
    fn spi_lines_target_the_bootstrap_vcpu() {
        let backend = Aarch64InterruptBackend::new(2).unwrap();
        backend.pulse(IrqLineId(PPI_END)).unwrap();

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
        assert_eq!(bootstrap.len(), 1);
        assert_eq!(bootstrap[0].vector, PPI_END);
    }

    #[test]
    fn full_list_register_keeps_interrupt_pending() {
        let backend = Aarch64InterruptBackend::new(1).unwrap();
        backend.pulse(IrqLineId(48)).unwrap();
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
        assert_eq!(delivered[0].vector, 48);
    }

    #[test]
    fn lowering_level_line_stops_retries() {
        let backend = Aarch64InterruptBackend::new(1).unwrap();
        backend.set_level(IrqLineId(49), true).unwrap();

        let mut first = Vec::new();
        backend
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                first.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert_eq!(first.len(), 1);

        backend.set_level(IrqLineId(49), false).unwrap();
        let mut after_lower = vec![];
        backend
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                after_lower.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert!(after_lower.is_empty());
    }
}
