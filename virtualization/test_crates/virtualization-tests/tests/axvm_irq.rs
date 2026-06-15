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

use std::{
    collections::VecDeque,
    sync::{Arc, Mutex, Weak},
};

use ax_errno::{AxError, AxResult};
use ax_plat::{console::ConsoleIf, time::TimeIf};
use axdevice::{
    AxVmDeviceConfig, AxVmDevices, DeviceBuildContext, DeviceBundle, DeviceFactory,
    DeviceFactoryRegistry, DeviceRegistration, IrqResolver,
};
use axdevice_base::{
    AccessWidth, BaseDeviceOps, InterruptTriggerMode, IrqLine, IrqLineId, IrqSink, Port,
};
use axvm::InterruptFabric;
use axvm_types::{
    EmulatedDeviceConfig, EmulatedDeviceType, GuestPhysAddr, GuestPhysAddrRange, VMInterruptMode,
};
use x86_vlapic::{EmulatedIoApic, EmulatedPit, EmulatedSerialPort, IoApicInterrupt};

static TEST_CONSOLE_INPUT: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());

struct TestConsole;

#[ax_plat::impl_plat_interface]
impl ConsoleIf for TestConsole {
    fn write_bytes(_bytes: &[u8]) {}

    fn read_bytes(bytes: &mut [u8]) -> usize {
        let mut input = TEST_CONSOLE_INPUT.lock().unwrap();
        let count = bytes.len().min(input.len());
        for byte in &mut bytes[..count] {
            *byte = input.pop_front().unwrap();
        }
        count
    }

    fn irq_num() -> Option<usize> {
        None
    }

    fn set_input_irq_enabled(_enabled: bool) {}

    fn handle_irq() -> ax_plat::console::ConsoleIrqEvent {
        ax_plat::console::ConsoleIrqEvent::empty()
    }
}

struct TestTime;

#[ax_plat::impl_plat_interface]
impl TimeIf for TestTime {
    fn current_ticks() -> u64 {
        0
    }

    fn ticks_to_nanos(ticks: u64) -> u64 {
        ticks
    }

    fn nanos_to_ticks(nanos: u64) -> u64 {
        nanos
    }

    fn epochoffset_nanos() -> u64 {
        0
    }

    fn irq_num() -> usize {
        0
    }

    fn set_oneshot_timer(_deadline_ns: u64) {}
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IrqEvent {
    SetLevel(IrqLineId, bool),
    Pulse(IrqLineId),
}

#[derive(Default)]
struct RecordingIrqSink {
    events: Mutex<Vec<IrqEvent>>,
}

impl RecordingIrqSink {
    fn events(&self) -> Vec<IrqEvent> {
        self.events.lock().unwrap().clone()
    }
}

fn program_ioapic_entry(ioapic: &EmulatedIoApic, gsi: usize, low: u32) {
    let base = GuestPhysAddr::from(0xfec0_0000);
    ioapic
        .handle_write(
            GuestPhysAddr::from(base.as_usize()),
            AccessWidth::Dword,
            0x10 + 2 * gsi,
        )
        .unwrap();
    ioapic
        .handle_write(
            GuestPhysAddr::from(base.as_usize() + 0x10),
            AccessWidth::Dword,
            low as usize,
        )
        .unwrap();
}

impl IrqSink for RecordingIrqSink {
    fn set_level(&self, line: IrqLineId, asserted: bool) -> AxResult {
        self.events
            .lock()
            .unwrap()
            .push(IrqEvent::SetLevel(line, asserted));
        Ok(())
    }

    fn pulse(&self, line: IrqLineId) -> AxResult {
        self.events.lock().unwrap().push(IrqEvent::Pulse(line));
        Ok(())
    }
}

struct IrqMmioDevice {
    range: GuestPhysAddrRange,
    line: IrqLine,
}

impl BaseDeviceOps<GuestPhysAddrRange> for IrqMmioDevice {
    fn address_range(&self) -> GuestPhysAddrRange {
        self.range
    }

    fn emu_type(&self) -> EmulatedDeviceType {
        EmulatedDeviceType::VirtioNet
    }

    fn handle_read(&self, _addr: GuestPhysAddr, _width: AccessWidth) -> AxResult<usize> {
        Ok(0)
    }

    fn handle_write(&self, _addr: GuestPhysAddr, _width: AccessWidth, _val: usize) -> AxResult {
        self.line.pulse()
    }
}

struct IrqMmioFactory;

impl DeviceFactory for IrqMmioFactory {
    fn device_type(&self) -> EmulatedDeviceType {
        EmulatedDeviceType::VirtioNet
    }

    fn build(
        &self,
        config: &EmulatedDeviceConfig,
        context: &DeviceBuildContext<'_>,
    ) -> AxResult<DeviceBundle> {
        let Some(end) = config.base_gpa.checked_add(config.length) else {
            return Err(AxError::InvalidInput);
        };
        let line = context.resolve_irq(config.irq_id, InterruptTriggerMode::EdgeTriggered)?;
        Ok(DeviceRegistration::Mmio(Arc::new(IrqMmioDevice {
            range: GuestPhysAddrRange::new(config.base_gpa.into(), end.into()),
            line,
        }))
        .into())
    }
}

fn irq_device_config(base_gpa: usize, irq_id: usize) -> EmulatedDeviceConfig {
    EmulatedDeviceConfig {
        name: String::from("irq-mmio"),
        base_gpa,
        length: 0x1000,
        irq_id,
        emu_type: EmulatedDeviceType::VirtioNet,
        cfg_list: vec![],
    }
}

fn irq_factory_registry() -> DeviceFactoryRegistry {
    let mut factories = DeviceFactoryRegistry::new();
    factories.register(Arc::new(IrqMmioFactory)).unwrap();
    factories
}

fn recording_fabric(mode: VMInterruptMode) -> (InterruptFabric, Weak<RecordingIrqSink>) {
    let sink = Arc::new(RecordingIrqSink::default());
    let weak = Arc::downgrade(&sink);
    (InterruptFabric::with_sink(mode, sink).unwrap(), weak)
}

#[test]
fn test_no_irq_fabric_rejects_backend_and_line_resolution() {
    let sink = Arc::new(RecordingIrqSink::default());
    assert_eq!(
        InterruptFabric::with_sink(VMInterruptMode::NoIrq, sink).err(),
        Some(AxError::InvalidInput)
    );

    let fabric = InterruptFabric::new(VMInterruptMode::NoIrq);
    let context = DeviceBuildContext::new(&fabric);
    assert_eq!(
        AxVmDevices::build_with_factories(
            AxVmDeviceConfig::new(vec![irq_device_config(0x6_0000, 12)]),
            &irq_factory_registry(),
            &context,
        )
        .err(),
        Some(AxError::InvalidInput)
    );
}

#[test]
fn test_interrupt_fabric_preserves_event_order() {
    let sink = Arc::new(RecordingIrqSink::default());
    let fabric = InterruptFabric::with_sink(VMInterruptMode::Emulated, sink.clone()).unwrap();
    let level = fabric
        .resolve_irq(13, InterruptTriggerMode::LevelTriggered)
        .unwrap();
    let edge = fabric
        .resolve_irq(14, InterruptTriggerMode::EdgeTriggered)
        .unwrap();

    assert_eq!(level.pulse(), Err(AxError::InvalidInput));
    assert_eq!(edge.raise(), Err(AxError::InvalidInput));
    level.raise().unwrap();
    level.lower().unwrap();
    edge.pulse().unwrap();

    assert_eq!(
        sink.events(),
        vec![
            IrqEvent::SetLevel(IrqLineId(13), true),
            IrqEvent::SetLevel(IrqLineId(13), false),
            IrqEvent::Pulse(IrqLineId(14)),
        ]
    );
}

#[test]
fn test_interrupt_fabric_can_signal_backend_directly() {
    let sink = Arc::new(RecordingIrqSink::default());
    let fabric = InterruptFabric::with_sink(VMInterruptMode::Emulated, sink.clone()).unwrap();

    fabric.set_level(21, true).unwrap();
    fabric.set_level(21, false).unwrap();
    fabric.pulse(22).unwrap();

    assert_eq!(
        sink.events(),
        vec![
            IrqEvent::SetLevel(IrqLineId(21), true),
            IrqEvent::SetLevel(IrqLineId(21), false),
            IrqEvent::Pulse(IrqLineId(22)),
        ]
    );
}

#[test]
fn test_factory_device_emits_irq_through_interrupt_fabric() {
    let (fabric, sink) = recording_fabric(VMInterruptMode::Emulated);
    let devices = {
        let context = DeviceBuildContext::new(&fabric);
        AxVmDevices::build_with_factories(
            AxVmDeviceConfig::new(vec![irq_device_config(0x7_0000, 15)]),
            &irq_factory_registry(),
            &context,
        )
        .unwrap()
    };

    devices
        .handle_mmio_write(GuestPhysAddr::from(0x7_0000), AccessWidth::Dword, 1)
        .unwrap();

    assert_eq!(
        sink.upgrade().unwrap().events(),
        vec![IrqEvent::Pulse(IrqLineId(15))]
    );
}

#[test]
fn test_dropping_devices_and_fabric_releases_irq_backend() {
    let (fabric, sink) = recording_fabric(VMInterruptMode::Emulated);
    let devices = {
        let context = DeviceBuildContext::new(&fabric);
        AxVmDevices::build_with_factories(
            AxVmDeviceConfig::new(vec![irq_device_config(0x8_0000, 16)]),
            &irq_factory_registry(),
            &context,
        )
        .unwrap()
    };

    drop(fabric);
    assert!(sink.upgrade().is_some());
    drop(devices);
    assert!(sink.upgrade().is_none());
}

#[test]
fn test_equal_irq_numbers_are_isolated_between_fabrics() {
    let (fabric_a, sink_a) = recording_fabric(VMInterruptMode::Emulated);
    let (fabric_b, sink_b) = recording_fabric(VMInterruptMode::Emulated);
    let devices_a = {
        let context = DeviceBuildContext::new(&fabric_a);
        AxVmDevices::build_with_factories(
            AxVmDeviceConfig::new(vec![irq_device_config(0x9_0000, 17)]),
            &irq_factory_registry(),
            &context,
        )
        .unwrap()
    };
    let devices_b = {
        let context = DeviceBuildContext::new(&fabric_b);
        AxVmDevices::build_with_factories(
            AxVmDeviceConfig::new(vec![irq_device_config(0xa_0000, 17)]),
            &irq_factory_registry(),
            &context,
        )
        .unwrap()
    };

    devices_a
        .handle_mmio_write(GuestPhysAddr::from(0x9_0000), AccessWidth::Dword, 1)
        .unwrap();

    assert_eq!(
        sink_a.upgrade().unwrap().events(),
        vec![IrqEvent::Pulse(IrqLineId(17))]
    );
    assert!(sink_b.upgrade().unwrap().events().is_empty());
    assert_eq!(devices_b.iter_mmio_dev().count(), 1);
}

#[test]
fn test_ioapic_mask_and_level_eoi_semantics() {
    let ioapic = EmulatedIoApic::new_default();
    assert_eq!(ioapic.pulse_gsi(5), None);

    program_ioapic_entry(&ioapic, 5, 0x31);
    assert_eq!(
        ioapic.pulse_gsi(5),
        Some(IoApicInterrupt {
            vector: 0x31,
            level_triggered: false,
        })
    );

    program_ioapic_entry(&ioapic, 6, 0x32 | (1 << 15));
    assert_eq!(
        ioapic.set_gsi_level(6, true),
        Some(IoApicInterrupt {
            vector: 0x32,
            level_triggered: true,
        })
    );
    assert_eq!(ioapic.set_gsi_level(6, true), None);
    assert_eq!(
        ioapic.end_of_interrupt(0x32),
        Some(IoApicInterrupt {
            vector: 0x32,
            level_triggered: true,
        })
    );
    assert_eq!(ioapic.set_gsi_level(6, false), None);
    assert_eq!(ioapic.end_of_interrupt(0x32), None);
}

#[test]
fn test_ioapic_instances_keep_gsi_state_isolated() {
    let ioapic_a = EmulatedIoApic::new_default();
    let ioapic_b = EmulatedIoApic::new_default();
    program_ioapic_entry(&ioapic_a, 7, 0x33);
    program_ioapic_entry(&ioapic_b, 7, 0x33);

    assert!(ioapic_a.set_gsi_level(7, true).is_some());
    assert!(ioapic_b.route_asserted_lines().is_empty());
}

#[test]
fn test_pit_poll_emits_periodic_edge_pulses() {
    let sink = Arc::new(RecordingIrqSink::default());
    let irq = IrqLine::new(
        IrqLineId(0),
        InterruptTriggerMode::EdgeTriggered,
        sink.clone(),
    );
    let pit = EmulatedPit::new(irq);

    pit.handle_write(Port(0x43), AccessWidth::Byte, 0x34)
        .unwrap();
    pit.handle_write(Port(0x40), AccessWidth::Byte, 0xa9)
        .unwrap();
    pit.handle_write(Port(0x40), AccessWidth::Byte, 0x04)
        .unwrap();

    pit.poll(500_000).unwrap();
    pit.poll(1_000_000).unwrap();
    pit.poll(2_000_000).unwrap();

    assert_eq!(
        sink.events(),
        vec![IrqEvent::Pulse(IrqLineId(0)), IrqEvent::Pulse(IrqLineId(0)),]
    );
}

#[test]
fn test_serial_rx_irq_stays_asserted_until_fifo_is_empty() {
    TEST_CONSOLE_INPUT.lock().unwrap().extend(*b"ab");
    let sink = Arc::new(RecordingIrqSink::default());
    let irq = IrqLine::new(
        IrqLineId(4),
        InterruptTriggerMode::LevelTriggered,
        sink.clone(),
    );
    let serial = EmulatedSerialPort::new(irq);

    serial
        .handle_write(Port(0x3f9), AccessWidth::Byte, 1)
        .unwrap();
    serial.poll().unwrap();
    assert_eq!(sink.events(), vec![IrqEvent::SetLevel(IrqLineId(4), true)]);

    assert_eq!(
        serial.handle_read(Port(0x3f8), AccessWidth::Byte),
        Ok(b'a' as usize)
    );
    assert_eq!(sink.events(), vec![IrqEvent::SetLevel(IrqLineId(4), true)]);

    assert_eq!(
        serial.handle_read(Port(0x3f8), AccessWidth::Byte),
        Ok(b'b' as usize)
    );
    assert_eq!(
        sink.events(),
        vec![
            IrqEvent::SetLevel(IrqLineId(4), true),
            IrqEvent::SetLevel(IrqLineId(4), false),
        ]
    );

    TEST_CONSOLE_INPUT.lock().unwrap().push_back(b'c');
    serial.poll().unwrap();
    serial
        .handle_write(Port(0x3f9), AccessWidth::Byte, 0)
        .unwrap();
    assert_eq!(
        sink.events(),
        vec![
            IrqEvent::SetLevel(IrqLineId(4), true),
            IrqEvent::SetLevel(IrqLineId(4), false),
            IrqEvent::SetLevel(IrqLineId(4), true),
            IrqEvent::SetLevel(IrqLineId(4), false),
        ]
    );

    serial
        .handle_write(Port(0x3f9), AccessWidth::Byte, 1)
        .unwrap();
    TEST_CONSOLE_INPUT.lock().unwrap().push_back(b'd');
    serial.poll().unwrap();
    serial
        .handle_write(Port(0x3fa), AccessWidth::Byte, 1 << 1)
        .unwrap();
    assert_eq!(
        sink.events().last(),
        Some(&IrqEvent::SetLevel(IrqLineId(4), false))
    );
}
