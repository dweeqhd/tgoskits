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

use alloc::{sync::Arc, vec::Vec};
use core::ops::Range;

use ax_errno::{AxResult, ax_err};
use ax_kspin::SpinNoIrq as Mutex;
use ax_memory_addr::is_aligned_4k;
use axdevice_base::{
    AccessWidth, BaseDeviceOps, BaseMmioDeviceOps, BasePortDeviceOps, BaseSysRegDeviceOps,
    BusAccess, BusAddress, BusOperation, BusResponse, DeviceAddrRange, DeviceLifecycle, Port,
    PortRange, SysRegAddr, SysRegAddrRange,
};
use axvm_types::{EmulatedDeviceConfig, GuestPhysAddr, GuestPhysAddrRange};

use crate::{
    AxVmDeviceConfig, DeviceBuildContext, DeviceBundle, DeviceFactoryRegistry, DeviceRegistration,
    PollableDeviceOps, range_alloc::RangeAllocator,
};

/// A set of emulated device types that can be accessed by a specific address range type.
pub struct AxEmuDevices<R: DeviceAddrRange> {
    emu_devices: Vec<Arc<dyn BaseDeviceOps<R>>>,
}

impl<R: DeviceAddrRange + 'static> AxEmuDevices<R> {
    /// Creates a new [`AxEmuDevices`] instance.
    pub fn new() -> Self {
        Self {
            emu_devices: Vec::new(),
        }
    }

    fn validate_dev_against<'a>(
        dev: &Arc<dyn BaseDeviceOps<R>>,
        existing_devices: impl IntoIterator<Item = &'a Arc<dyn BaseDeviceOps<R>>>,
    ) -> AxResult
    where
        R: 'a,
    {
        let new_range = dev.address_range();
        let new_type = dev.emu_type();

        if new_range.is_empty() {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "failed to register {} device type {} at range {new_range:#x}: range is empty \
                     or invalid, possibly due to address overflow",
                    R::BUS_NAME,
                    new_type,
                )
            );
        }

        for existing in existing_devices {
            let existing_range = existing.address_range();
            let existing_type = existing.emu_type();

            if Arc::ptr_eq(existing, dev) {
                return ax_err!(
                    AlreadyExists,
                    format_args!(
                        "failed to register {} device type {} at range {new_range:#x}: the same \
                         device is already registered as type {} at range {existing_range:#x}",
                        R::BUS_NAME,
                        new_type,
                        existing_type,
                    )
                );
            }

            if new_range == existing_range {
                return ax_err!(
                    AlreadyExists,
                    format_args!(
                        "failed to register {} device type {} at range {new_range:#x}: range is \
                         already registered by device type {} at range {existing_range:#x}",
                        R::BUS_NAME,
                        new_type,
                        existing_type,
                    )
                );
            }

            if new_range.overlaps(&existing_range) {
                return ax_err!(
                    AddrInUse,
                    format_args!(
                        "failed to register {} device type {} at range {new_range:#x}: overlaps \
                         device type {} at range {existing_range:#x}",
                        R::BUS_NAME,
                        new_type,
                        existing_type,
                    )
                );
            }
        }

        Ok(())
    }

    /// Validates a group of devices without modifying this set.
    fn validate_devices(&self, devices: &[Arc<dyn BaseDeviceOps<R>>]) -> AxResult {
        for (index, device) in devices.iter().enumerate() {
            Self::validate_dev_against(
                device,
                self.emu_devices.iter().chain(devices[..index].iter()),
            )?;
        }
        Ok(())
    }

    /// Adds a device to the set after validating its range.
    pub fn add_dev(&mut self, dev: Arc<dyn BaseDeviceOps<R>>) -> AxResult {
        self.validate_devices(core::slice::from_ref(&dev))?;
        self.emu_devices.push(dev);
        Ok(())
    }

    fn commit_devices(&mut self, devices: Vec<Arc<dyn BaseDeviceOps<R>>>) {
        self.emu_devices.extend(devices);
    }

    // pub fn remove_dev(&mut self, ...)
    //
    // `remove_dev` seems to need something like `downcast-rs` to make sense. As it's not likely to
    // be able to have a proper predicate to remove a device from the list without knowing the
    // concrete type of the device.

    /// Find a device by address.
    pub fn find_dev(&self, addr: R::Addr) -> Option<Arc<dyn BaseDeviceOps<R>>> {
        self.emu_devices
            .iter()
            .find(|&dev| dev.address_range().contains(addr))
            .cloned()
    }

    /// Iterates over the devices in the set.
    pub fn iter(&self) -> impl Iterator<Item = &Arc<dyn BaseDeviceOps<R>>> {
        self.emu_devices.iter()
    }

    /// Iterates over the devices in the set mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Arc<dyn BaseDeviceOps<R>>> {
        self.emu_devices.iter_mut()
    }
}

impl<R: DeviceAddrRange + 'static> Default for AxEmuDevices<R> {
    fn default() -> Self {
        Self::new()
    }
}

type AxEmuMmioDevices = AxEmuDevices<GuestPhysAddrRange>;
type AxEmuSysRegDevices = AxEmuDevices<SysRegAddrRange>;
type AxEmuPortDevices = AxEmuDevices<PortRange>;

/// represent A vm own devices
pub struct AxVmDevices {
    /// emu devices
    emu_mmio_devices: AxEmuMmioDevices,
    emu_sys_reg_devices: AxEmuSysRegDevices,
    emu_port_devices: AxEmuPortDevices,
    pollable_devices: Vec<Arc<dyn PollableDeviceOps>>,
    lifecycle_devices: Vec<Arc<dyn DeviceLifecycle>>,
    /// IVC channel range allocator
    ivc_channel: Option<Mutex<RangeAllocator>>,
}

#[inline]
fn log_device_io(
    addr_type: &'static str,
    addr: impl core::fmt::LowerHex,
    addr_range: impl core::fmt::LowerHex,
    read: bool,
    width: AccessWidth,
) {
    let rw = if read { "read" } else { "write" };
    trace!("emu_device {rw}: {addr_type} {addr:#x} in range {addr_range:#x} with width {width:?}")
}

#[inline]
fn device_not_found<T>(
    addr_type: &'static str,
    addr: impl core::fmt::LowerHex,
    read: bool,
    width: AccessWidth,
) -> AxResult<T> {
    let rw = if read { "read" } else { "write" };
    ax_err!(
        NotFound,
        format_args!(
            "emu_device {rw} failed: device not found for {addr_type} {addr:#x} with width \
             {width:?}"
        )
    )
}

/// The implemention for AxVmDevices
impl AxVmDevices {
    fn empty() -> Self {
        Self {
            emu_mmio_devices: AxEmuMmioDevices::new(),
            emu_sys_reg_devices: AxEmuSysRegDevices::new(),
            emu_port_devices: AxEmuPortDevices::new(),
            pollable_devices: Vec::new(),
            lifecycle_devices: Vec::new(),
            ivc_channel: None,
        }
    }

    /// Creates an empty device registry.
    ///
    /// Configured devices are built through [`Self::build_with_factories`] so
    /// architecture-specific construction remains outside `axdevice`.
    pub fn new(config: AxVmDeviceConfig) -> AxResult<Self> {
        if !config.emu_configs.is_empty() {
            return ax_err!(
                Unsupported,
                "AxVmDevices::new no longer builds configured devices; use build_with_factories"
            );
        }
        Ok(Self::empty())
    }

    /// Builds devices with registered factories.
    pub fn build_with_factories(
        config: AxVmDeviceConfig,
        factories: &DeviceFactoryRegistry,
        context: &DeviceBuildContext<'_>,
    ) -> AxResult<Self> {
        let mut this = Self::empty();
        for config in &config.emu_configs {
            this.register_factory_device(config, factories, context)?;
        }
        Ok(this)
    }

    /// Builds and atomically registers one factory-managed device.
    pub fn register_factory_device(
        &mut self,
        config: &EmulatedDeviceConfig,
        factories: &DeviceFactoryRegistry,
        context: &DeviceBuildContext<'_>,
    ) -> AxResult {
        let bundle = factories.build(config, context)?;
        self.register_bundle(bundle)
    }

    /// Allocates an IVC (Inter-VM Communication) channel of the specified size.
    pub fn alloc_ivc_channel(&self, size: usize) -> AxResult<GuestPhysAddr> {
        if size == 0 {
            return ax_err!(InvalidInput, "Size must be greater than 0");
        }
        if !is_aligned_4k(size) {
            return ax_err!(InvalidInput, "Size must be aligned to 4K");
        }

        if let Some(allocator) = &self.ivc_channel {
            allocator
                .lock()
                .allocate_range(size)
                .ok_or_else(|| {
                    warn!("Failed to allocate IVC channel range with size {size:#x}");
                    ax_errno::ax_err_type!(NoMemory, "IVC channel allocation failed")
                })
                .map(|range| {
                    debug!("Allocated IVC channel range: {range:x?}");
                    GuestPhysAddr::from_usize(range.start)
                })
        } else {
            ax_err!(InvalidInput, "IVC channel not exists")
        }
    }

    /// Releases an IVC channel at the specified address and size.
    pub fn release_ivc_channel(&self, addr: GuestPhysAddr, size: usize) -> AxResult {
        if size == 0 {
            return ax_err!(InvalidInput, "Size must be greater than 0");
        }
        if !is_aligned_4k(size) {
            return ax_err!(InvalidInput, "Size must be aligned to 4K");
        }

        if let Some(allocator) = &self.ivc_channel {
            let range = addr.as_usize()..addr.as_usize() + size;
            if allocator.lock().free_range(range.clone()) {
                debug!("Released IVC channel range: {range:x?}");
                Ok(())
            } else {
                ax_err!(InvalidInput, "Invalid IVC channel range")
            }
        } else {
            ax_err!(InvalidInput, "IVC channel not exists")
        }
    }

    /// Registers a bundle atomically after validating all capabilities.
    pub fn register_bundle(&mut self, bundle: DeviceBundle) -> AxResult {
        self.emu_mmio_devices.validate_devices(&bundle.mmio)?;
        self.emu_port_devices.validate_devices(&bundle.port)?;
        self.emu_sys_reg_devices.validate_devices(&bundle.sysreg)?;
        self.validate_ivc_channels(&bundle.ivc_channels)?;

        for (index, pollable) in bundle.pollable.iter().enumerate() {
            if self
                .pollable_devices
                .iter()
                .chain(bundle.pollable[..index].iter())
                .any(|existing| Arc::ptr_eq(existing, pollable))
            {
                return ax_err!(
                    AlreadyExists,
                    "failed to register pollable device: the same capability is already registered"
                );
            }
        }

        for (index, lifecycle) in bundle.lifecycle.iter().enumerate() {
            if self
                .lifecycle_devices
                .iter()
                .chain(bundle.lifecycle[..index].iter())
                .any(|existing| Arc::ptr_eq(existing, lifecycle))
            {
                return ax_err!(
                    AlreadyExists,
                    "failed to register lifecycle device: the same capability is already \
                     registered"
                );
            }
        }

        self.emu_mmio_devices.commit_devices(bundle.mmio);
        self.emu_port_devices.commit_devices(bundle.port);
        self.emu_sys_reg_devices.commit_devices(bundle.sysreg);
        self.pollable_devices.extend(bundle.pollable);
        self.lifecycle_devices.extend(bundle.lifecycle);
        for range in bundle.ivc_channels {
            info!(
                "IVCChannel initialized with base GPA {base_gpa:#x} and length {length:#x}",
                base_gpa = range.start,
                length = range.end - range.start
            );
            self.ivc_channel = Some(Mutex::new(RangeAllocator::new(range)));
        }
        Ok(())
    }

    fn validate_ivc_channels(&self, channels: &[Range<usize>]) -> AxResult {
        if channels.is_empty() {
            return Ok(());
        }
        if self.ivc_channel.is_some() || channels.len() > 1 {
            return ax_err!(
                AlreadyExists,
                "failed to register IVCChannel: channel allocator is already registered"
            );
        }
        let range = &channels[0];
        if range.start >= range.end {
            return ax_err!(
                InvalidInput,
                format_args!(
                    "failed to register IVCChannel range {:#x}..{:#x}: range is empty or invalid",
                    range.start, range.end
                )
            );
        }
        Ok(())
    }

    /// Add a MMIO device to the device list
    pub fn add_mmio_dev(&mut self, dev: Arc<dyn BaseMmioDeviceOps>) -> AxResult {
        self.register_bundle(DeviceRegistration::Mmio(dev).into())
    }

    /// Add a system register device to the device list
    pub fn add_sys_reg_dev(&mut self, dev: Arc<dyn BaseSysRegDeviceOps>) -> AxResult {
        self.register_bundle(DeviceRegistration::SysReg(dev).into())
    }

    /// Add a port device to the device list
    pub fn add_port_dev(&mut self, dev: Arc<dyn BasePortDeviceOps>) -> AxResult {
        self.register_bundle(DeviceRegistration::Port(dev).into())
    }

    /// Iterates over the MMIO devices in the set.
    pub fn iter_mmio_dev(&self) -> impl Iterator<Item = &Arc<dyn BaseMmioDeviceOps>> {
        self.emu_mmio_devices.iter()
    }

    /// Iterates over the system register devices in the set.
    pub fn iter_sys_reg_dev(&self) -> impl Iterator<Item = &Arc<dyn BaseSysRegDeviceOps>> {
        self.emu_sys_reg_devices.iter()
    }

    /// Iterates over the port devices in the set.
    pub fn iter_port_dev(&self) -> impl Iterator<Item = &Arc<dyn BasePortDeviceOps>> {
        self.emu_port_devices.iter()
    }

    /// Iterates over devices that require periodic polling.
    pub fn iter_pollable_dev(&self) -> impl Iterator<Item = &Arc<dyn PollableDeviceOps>> {
        self.pollable_devices.iter()
    }

    /// Iterates over registered lifecycle capabilities.
    pub fn iter_lifecycle_dev(&self) -> impl Iterator<Item = &Arc<dyn DeviceLifecycle>> {
        self.lifecycle_devices.iter()
    }

    /// Resets all lifecycle-aware devices in registration order.
    pub fn reset_devices(&self) -> AxResult {
        for device in &self.lifecycle_devices {
            device.reset()?;
        }
        Ok(())
    }

    /// Suspends all lifecycle-aware devices in registration order.
    pub fn suspend_devices(&self) -> AxResult {
        for device in &self.lifecycle_devices {
            device.suspend()?;
        }
        Ok(())
    }

    /// Resumes all lifecycle-aware devices in reverse registration order.
    pub fn resume_devices(&self) -> AxResult {
        for device in self.lifecycle_devices.iter().rev() {
            device.resume()?;
        }
        Ok(())
    }

    /// Dispatches a normalized bus transaction.
    pub fn handle_bus_access(&self, access: BusAccess) -> AxResult<BusResponse> {
        match (access.address, access.operation) {
            (BusAddress::Mmio(addr), BusOperation::Read) => self
                .dispatch_mmio_read(addr, access.width)
                .map(|value| BusResponse::Read { value }),
            (BusAddress::Mmio(addr), BusOperation::Write { value }) => {
                self.dispatch_mmio_write(addr, access.width, value)?;
                Ok(BusResponse::WriteComplete)
            }
            (BusAddress::Port(port), BusOperation::Read) => self
                .dispatch_port_read(port, access.width)
                .map(|value| BusResponse::Read { value }),
            (BusAddress::Port(port), BusOperation::Write { value }) => {
                self.dispatch_port_write(port, access.width, value)?;
                Ok(BusResponse::WriteComplete)
            }
            (BusAddress::SysReg(addr), BusOperation::Read) => self
                .dispatch_sys_reg_read(addr, access.width)
                .map(|value| BusResponse::Read { value }),
            (BusAddress::SysReg(addr), BusOperation::Write { value }) => {
                self.dispatch_sys_reg_write(addr, access.width, value)?;
                Ok(BusResponse::WriteComplete)
            }
            (BusAddress::PciConfig { .. }, _) => ax_err!(
                Unsupported,
                "PCI config space routing is not installed for this VM"
            ),
        }
    }

    fn expect_read_response(response: BusResponse) -> AxResult<usize> {
        match response {
            BusResponse::Read { value } => Ok(value),
            BusResponse::WriteComplete => ax_err!(InvalidInput, "bus read returned write response"),
        }
    }

    fn expect_write_response(response: BusResponse) -> AxResult {
        match response {
            BusResponse::WriteComplete => Ok(()),
            BusResponse::Read { .. } => ax_err!(InvalidInput, "bus write returned read response"),
        }
    }

    fn dispatch_mmio_read(&self, addr: GuestPhysAddr, width: AccessWidth) -> AxResult<usize> {
        if let Some(emu_dev) = self.emu_mmio_devices.find_dev(addr) {
            log_device_io("mmio", addr, emu_dev.address_range(), true, width);

            return emu_dev.handle_read(addr, width);
        }
        device_not_found("mmio", addr, true, width)
    }

    fn dispatch_mmio_write(&self, addr: GuestPhysAddr, width: AccessWidth, val: usize) -> AxResult {
        if let Some(emu_dev) = self.emu_mmio_devices.find_dev(addr) {
            log_device_io("mmio", addr, emu_dev.address_range(), false, width);

            return emu_dev.handle_write(addr, width, val);
        }
        device_not_found("mmio", addr, false, width)
    }

    fn dispatch_sys_reg_read(&self, addr: SysRegAddr, width: AccessWidth) -> AxResult<usize> {
        if let Some(emu_dev) = self.emu_sys_reg_devices.find_dev(addr) {
            log_device_io("sys_reg", addr.0, emu_dev.address_range(), true, width);

            return emu_dev.handle_read(addr, width);
        }
        device_not_found("sys_reg", addr, true, width)
    }

    fn dispatch_sys_reg_write(&self, addr: SysRegAddr, width: AccessWidth, val: usize) -> AxResult {
        if let Some(emu_dev) = self.emu_sys_reg_devices.find_dev(addr) {
            log_device_io("sys_reg", addr.0, emu_dev.address_range(), false, width);

            return emu_dev.handle_write(addr, width, val);
        }
        device_not_found("sys_reg", addr, false, width)
    }

    fn dispatch_port_read(&self, port: Port, width: AccessWidth) -> AxResult<usize> {
        if let Some(emu_dev) = self.emu_port_devices.find_dev(port) {
            log_device_io("port", port.0, emu_dev.address_range(), true, width);

            return emu_dev.handle_read(port, width);
        }
        device_not_found("port", port, true, width)
    }

    fn dispatch_port_write(&self, port: Port, width: AccessWidth, val: usize) -> AxResult {
        if let Some(emu_dev) = self.emu_port_devices.find_dev(port) {
            log_device_io("port", port.0, emu_dev.address_range(), false, width);

            return emu_dev.handle_write(port, width, val);
        }
        device_not_found("port", port, false, width)
    }

    /// Handle the MMIO read by GuestPhysAddr and data width, return the value of the guest want to read
    pub fn handle_mmio_read(&self, addr: GuestPhysAddr, width: AccessWidth) -> AxResult<usize> {
        Self::expect_read_response(
            self.handle_bus_access(BusAccess::read(BusAddress::Mmio(addr), width))?,
        )
    }

    /// Handle the MMIO write by GuestPhysAddr, data width and the value need to write, call specific device to write the value
    pub fn handle_mmio_write(
        &self,
        addr: GuestPhysAddr,
        width: AccessWidth,
        val: usize,
    ) -> AxResult {
        Self::expect_write_response(self.handle_bus_access(BusAccess::write(
            BusAddress::Mmio(addr),
            width,
            val,
        ))?)
    }

    /// Handle the system register read by SysRegAddr and data width, return the value of the guest want to read
    pub fn handle_sys_reg_read(&self, addr: SysRegAddr, width: AccessWidth) -> AxResult<usize> {
        Self::expect_read_response(
            self.handle_bus_access(BusAccess::read(BusAddress::SysReg(addr), width))?,
        )
    }

    /// Handle the system register write by SysRegAddr, data width and the value need to write, call specific device to write the value
    pub fn handle_sys_reg_write(
        &self,
        addr: SysRegAddr,
        width: AccessWidth,
        val: usize,
    ) -> AxResult {
        Self::expect_write_response(self.handle_bus_access(BusAccess::write(
            BusAddress::SysReg(addr),
            width,
            val,
        ))?)
    }

    /// Handle the port read by port number and data width, return the value of the guest want to read
    pub fn handle_port_read(&self, port: Port, width: AccessWidth) -> AxResult<usize> {
        Self::expect_read_response(
            self.handle_bus_access(BusAccess::read(BusAddress::Port(port), width))?,
        )
    }

    /// Handle the port write by port number, data width and the value need to write, call specific device to write the value
    pub fn handle_port_write(&self, port: Port, width: AccessWidth, val: usize) -> AxResult {
        Self::expect_write_response(self.handle_bus_access(BusAccess::write(
            BusAddress::Port(port),
            width,
            val,
        ))?)
    }
}
