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

//! x86 virtual IO APIC interrupt backend and device factories.

use alloc::{collections::VecDeque, sync::Arc};

use ax_errno::{AxResult, ax_err};
use ax_kspin::SpinNoPreempt as Mutex;
use axdevice::{
    DeviceBuildContext, DeviceBundle, DeviceFactory, DeviceFactoryRegistry, DeviceRegistration,
    PollableDeviceOps,
};
use axdevice_base::{InterruptTriggerMode, IrqLineId, IrqSink};
use axvm_types::{
    EmulatedDeviceConfig, EmulatedDeviceType, InterruptVector, VCpuId, VMInterruptMode,
};
use x86_vlapic::{EmulatedIoApic, EmulatedPit, EmulatedSerialPort, IoApicInterrupt};

use super::{InterruptControllerOps, InterruptFabric, MsiRoute, PendingInterrupt};

pub(crate) const HOST_IOAPIC_VECTOR_BASE: usize = 0x20;
pub(crate) const HOST_IOAPIC_GSI_COUNT: usize = x86_vlapic::IOAPIC_GSI_COUNT;

const BSP_VCPU_ID: VCpuId = 0;
const PIT_TIMER_GSI: usize = 0;
const COM1_GSI: usize = 4;
const COM1_PORT_BASE: usize = 0x3f8;
const COM1_PORT_LENGTH: usize = 0x8;
const PIT_PORT_BASE: usize = 0x40;
const PIT_PORT_LENGTH: usize = 0x22;

#[derive(Clone, Copy)]
struct QueuedInterrupt {
    vcpu_id: VCpuId,
    interrupt: PendingInterrupt,
}

pub(crate) struct X86InterruptBackend {
    ioapic: Arc<EmulatedIoApic>,
    pending: Mutex<VecDeque<QueuedInterrupt>>,
}

impl X86InterruptBackend {
    fn new(ioapic: Arc<EmulatedIoApic>) -> Self {
        Self {
            ioapic,
            pending: Mutex::new(VecDeque::new()),
        }
    }

    fn validate_gsi(gsi: usize) -> AxResult {
        if gsi >= x86_vlapic::IOAPIC_GSI_COUNT {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "invalid x86 GSI {gsi}; valid GSIs are 0..{}",
                    x86_vlapic::IOAPIC_GSI_COUNT
                )
            );
        }
        Ok(())
    }

    fn queue_routed(&self, vcpu_id: VCpuId, interrupt: Option<IoApicInterrupt>) {
        let Some(interrupt) = interrupt else {
            return;
        };
        self.pending.lock().push_back(QueuedInterrupt {
            vcpu_id,
            interrupt: PendingInterrupt {
                vector: interrupt.vector as usize,
                trigger: if interrupt.level_triggered {
                    InterruptTriggerMode::LevelTriggered
                } else {
                    InterruptTriggerMode::EdgeTriggered
                },
            },
        });
    }

    fn route_asserted_lines(&self) {
        for interrupt in self.ioapic.route_asserted_lines() {
            self.queue_routed(BSP_VCPU_ID, Some(interrupt));
        }
    }
}

impl IrqSink for X86InterruptBackend {
    fn set_level(&self, line: IrqLineId, asserted: bool) -> AxResult {
        Self::validate_gsi(line.0)?;
        self.queue_routed(BSP_VCPU_ID, self.ioapic.set_gsi_level(line.0, asserted));
        Ok(())
    }

    fn pulse(&self, line: IrqLineId) -> AxResult {
        Self::validate_gsi(line.0)?;
        self.queue_routed(BSP_VCPU_ID, self.ioapic.pulse_gsi(line.0));
        Ok(())
    }
}

impl InterruptControllerOps for X86InterruptBackend {
    fn inject_msi(&self, route: MsiRoute) -> AxResult {
        self.pending.lock().push_back(QueuedInterrupt {
            vcpu_id: route.target_vcpu,
            interrupt: PendingInterrupt {
                vector: route.vector,
                trigger: InterruptTriggerMode::EdgeTriggered,
            },
        });
        Ok(())
    }

    fn eoi(&self, vcpu_id: VCpuId, vector: InterruptVector) -> AxResult {
        self.queue_routed(vcpu_id, self.ioapic.end_of_interrupt(vector));
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
        self.route_asserted_lines();
        loop {
            let queued = {
                let mut pending = self.pending.lock();
                let Some(index) = pending.iter().position(|entry| entry.vcpu_id == vcpu_id) else {
                    return Ok(());
                };
                let Some(queued) = pending.remove(index) else {
                    return ax_err!(BadState, "pending x86 IRQ index disappeared");
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

struct X86IoApicFactory {
    base_gpa: usize,
    length: usize,
    ioapic: Arc<EmulatedIoApic>,
}

impl DeviceFactory for X86IoApicFactory {
    fn device_type(&self) -> EmulatedDeviceType {
        EmulatedDeviceType::X86IoApic
    }

    fn build(
        &self,
        config: &EmulatedDeviceConfig,
        _context: &DeviceBuildContext<'_>,
    ) -> AxResult<DeviceBundle> {
        if config.base_gpa != self.base_gpa
            || config.length != self.length
            || !config.cfg_list.is_empty()
            || !config.irqs.is_empty()
        {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "x86 IO APIC configuration changed while building device '{}'",
                    config.name
                )
            );
        }
        Ok(DeviceRegistration::Mmio(self.ioapic.clone()).into())
    }
}

struct X86PitFactory;

struct X86PitPoller(Arc<EmulatedPit>);

impl PollableDeviceOps for X86PitPoller {
    fn poll(&self, now_ns: u64) -> AxResult {
        self.0.poll(now_ns)
    }
}

impl DeviceFactory for X86PitFactory {
    fn device_type(&self) -> EmulatedDeviceType {
        EmulatedDeviceType::X86Pit
    }

    fn build(
        &self,
        config: &EmulatedDeviceConfig,
        context: &DeviceBuildContext<'_>,
    ) -> AxResult<DeviceBundle> {
        validate_port_config(config, PIT_PORT_BASE, PIT_PORT_LENGTH, "x86 PIT")?;
        let irq = context.resolve_config_irq(
            config,
            "irq",
            PIT_TIMER_GSI,
            InterruptTriggerMode::EdgeTriggered,
        )?;
        let pit = Arc::new(EmulatedPit::new(irq));
        Ok(DeviceBundle::new()
            .with_registration(DeviceRegistration::Port(pit.clone()))
            .with_registration(DeviceRegistration::Pollable(Arc::new(X86PitPoller(pit)))))
    }
}

struct X86SerialFactory;

struct X86SerialPoller(Arc<EmulatedSerialPort>);

impl PollableDeviceOps for X86SerialPoller {
    fn poll(&self, _now_ns: u64) -> AxResult {
        self.0.poll()
    }
}

impl DeviceFactory for X86SerialFactory {
    fn device_type(&self) -> EmulatedDeviceType {
        EmulatedDeviceType::Console
    }

    fn build(
        &self,
        config: &EmulatedDeviceConfig,
        context: &DeviceBuildContext<'_>,
    ) -> AxResult<DeviceBundle> {
        validate_port_config(config, COM1_PORT_BASE, COM1_PORT_LENGTH, "x86 COM1")?;
        let irq = context.resolve_config_irq(
            config,
            "irq",
            COM1_GSI,
            InterruptTriggerMode::LevelTriggered,
        )?;
        let serial = Arc::new(EmulatedSerialPort::new(irq));
        Ok(DeviceBundle::new()
            .with_registration(DeviceRegistration::Port(serial.clone()))
            .with_registration(DeviceRegistration::Pollable(Arc::new(X86SerialPoller(
                serial,
            )))))
    }
}

fn validate_port_config(
    config: &EmulatedDeviceConfig,
    expected_base: usize,
    expected_length: usize,
    device_name: &str,
) -> AxResult {
    if config.base_gpa != expected_base
        || config.length != expected_length
        || !config.cfg_list.is_empty()
    {
        return ax_err!(
            InvalidInput,
            format_args!(
                "{device_name} device '{}' requires base {expected_base:#x}, length \
                 {expected_length:#x}, and an empty config list",
                config.name
            )
        );
    }
    Ok(())
}

/// Converts a host IO APIC vector into its GSI.
pub(crate) fn host_vector_to_gsi(vector: usize) -> Option<usize> {
    let gsi = vector.checked_sub(HOST_IOAPIC_VECTOR_BASE)?;
    (gsi < HOST_IOAPIC_GSI_COUNT).then_some(gsi)
}

pub(crate) fn configure(
    factories: &mut DeviceFactoryRegistry,
    mode: VMInterruptMode,
    configs: &[EmulatedDeviceConfig],
) -> AxResult<InterruptFabric> {
    factories.register(Arc::new(X86PitFactory))?;
    factories.register(Arc::new(X86SerialFactory))?;

    let mut ioapic_configs = configs
        .iter()
        .filter(|config| config.emu_type == EmulatedDeviceType::X86IoApic);
    let Some(config) = ioapic_configs.next() else {
        return Ok(InterruptFabric::new(mode));
    };
    if ioapic_configs.next().is_some() {
        return ax_err!(
            AlreadyExists,
            "a VM can register only one x86 virtual IO APIC"
        );
    }
    if config.length == 0 || config.base_gpa.checked_add(config.length).is_none() {
        return ax_err!(
            InvalidInput,
            format_args!(
                "x86 IO APIC device '{}' has an invalid MMIO range",
                config.name
            )
        );
    }
    if !config.cfg_list.is_empty() {
        return ax_err!(
            InvalidInput,
            format_args!(
                "x86 IO APIC device '{}' requires an empty config list",
                config.name
            )
        );
    }
    if !config.irqs.is_empty() {
        return ax_err!(
            InvalidInput,
            format_args!(
                "x86 IO APIC device '{}' does not expose device IRQ outputs",
                config.name
            )
        );
    }

    let ioapic = Arc::new(EmulatedIoApic::new(
        config.base_gpa.into(),
        Some(config.length),
    ));
    factories.register(Arc::new(X86IoApicFactory {
        base_gpa: config.base_gpa,
        length: config.length,
        ioapic: ioapic.clone(),
    }))?;

    let backend = Arc::new(X86InterruptBackend::new(ioapic));
    let sink: Arc<dyn IrqSink> = backend.clone();
    let controller: Arc<dyn InterruptControllerOps> = backend;
    InterruptFabric::with_controller(mode, sink, controller)
}

#[cfg(test)]
mod tests {
    use alloc::{vec, vec::Vec};

    use ax_errno::AxError;
    use axdevice_base::{AccessWidth, BaseDeviceOps, MsiMessage};
    use axvm_types::GuestPhysAddr;

    use super::*;

    const IOAPIC_BASE: usize = 0xfec0_0000;

    fn backend_with_route(gsi: usize, vector: u8) -> X86InterruptBackend {
        let ioapic = Arc::new(EmulatedIoApic::new_default());
        ioapic
            .handle_write(
                GuestPhysAddr::from(IOAPIC_BASE),
                AccessWidth::Dword,
                0x10 + 2 * gsi,
            )
            .unwrap();
        ioapic
            .handle_write(
                GuestPhysAddr::from(IOAPIC_BASE + 0x10),
                AccessWidth::Dword,
                vector as usize,
            )
            .unwrap();
        X86InterruptBackend::new(ioapic)
    }

    #[test]
    fn pending_irqs_are_isolated_between_backends() {
        let backend_a = backend_with_route(9, 0x39);
        let backend_b = backend_with_route(9, 0x39);
        backend_a.pulse(IrqLineId(9)).unwrap();

        let mut delivered_a = Vec::new();
        backend_a
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                delivered_a.push(interrupt);
                Ok(())
            })
            .unwrap();
        let mut delivered_b = Vec::new();
        backend_b
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                delivered_b.push(interrupt);
                Ok(())
            })
            .unwrap();

        assert_eq!(
            delivered_a,
            vec![PendingInterrupt {
                vector: 0x39,
                trigger: InterruptTriggerMode::EdgeTriggered,
            }]
        );
        assert!(delivered_b.is_empty());
    }

    #[test]
    fn failed_delivery_keeps_interrupt_pending() {
        let backend = backend_with_route(10, 0x3a);
        backend.pulse(IrqLineId(10)).unwrap();

        assert_eq!(
            backend.drain_pending(BSP_VCPU_ID, &mut |_| Err(AxError::BadState)),
            Err(AxError::BadState)
        );

        let mut delivered = Vec::new();
        backend
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                delivered.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert_eq!(delivered.len(), 1);
        assert_eq!(delivered[0].vector, 0x3a);
    }

    #[test]
    fn msi_route_queues_target_vector() {
        let backend = X86InterruptBackend::new(Arc::new(EmulatedIoApic::new_default()));
        backend
            .inject_msi(MsiRoute {
                message: MsiMessage {
                    address: 0xfee0_0000,
                    data: 0x45,
                },
                target_vcpu: 1,
                vector: 0x45,
            })
            .unwrap();

        let mut delivered_bsp = Vec::new();
        backend
            .drain_pending(BSP_VCPU_ID, &mut |interrupt| {
                delivered_bsp.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert!(delivered_bsp.is_empty());

        let mut delivered_target = Vec::new();
        backend
            .drain_pending(1, &mut |interrupt| {
                delivered_target.push(interrupt);
                Ok(())
            })
            .unwrap();
        assert_eq!(
            delivered_target,
            vec![PendingInterrupt {
                vector: 0x45,
                trigger: InterruptTriggerMode::EdgeTriggered,
            }]
        );
    }

    #[test]
    fn host_vector_mapping_is_bounded() {
        assert_eq!(host_vector_to_gsi(HOST_IOAPIC_VECTOR_BASE), Some(0));
        assert_eq!(
            host_vector_to_gsi(HOST_IOAPIC_VECTOR_BASE + HOST_IOAPIC_GSI_COUNT - 1),
            Some(HOST_IOAPIC_GSI_COUNT - 1)
        );
        assert_eq!(host_vector_to_gsi(HOST_IOAPIC_VECTOR_BASE - 1), None);
        assert_eq!(
            host_vector_to_gsi(HOST_IOAPIC_VECTOR_BASE + HOST_IOAPIC_GSI_COUNT),
            None
        );
    }
}
