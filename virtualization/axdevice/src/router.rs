//! Device registry and unified bus router.

use alloc::{sync::Arc, vec::Vec};

use ax_errno::{AxResult, ax_err};
use axdevice_base::{
    BaseMmioDeviceOps, BasePortDeviceOps, BaseSysRegDeviceOps, BusAccess, BusAddress, BusOperation,
    BusResponse, DeviceAddrRange, DeviceCapabilities, DeviceDescriptor, DeviceId, Resource,
    ResourceSet,
};

/// A registered device descriptor owned by a VM.
pub struct RegisteredDevice {
    id: DeviceId,
    name: &'static str,
    resources: ResourceSet,
    capabilities: DeviceCapabilities,
}

impl RegisteredDevice {
    /// Creates a registered device descriptor.
    pub const fn new(
        id: DeviceId,
        name: &'static str,
        resources: ResourceSet,
        capabilities: DeviceCapabilities,
    ) -> Self {
        Self {
            id,
            name,
            resources,
            capabilities,
        }
    }
}

impl DeviceDescriptor for RegisteredDevice {
    fn device_id(&self) -> DeviceId {
        self.id
    }

    fn name(&self) -> &str {
        self.name
    }

    fn resources(&self) -> ResourceSet {
        self.resources.clone()
    }

    fn capabilities(&self) -> DeviceCapabilities {
        self.capabilities
    }
}

/// Registry of device resources and capabilities.
#[derive(Default)]
pub struct DeviceRegistry {
    next_id: u64,
    devices: Vec<RegisteredDevice>,
    msi_vector_limit: Option<usize>,
}

impl DeviceRegistry {
    /// Creates an empty registry.
    pub const fn new() -> Self {
        Self {
            next_id: 1,
            devices: Vec::new(),
            msi_vector_limit: None,
        }
    }

    /// Creates an empty registry with a maximum number of MSI/MSI-X vectors.
    pub const fn with_msi_vector_limit(limit: usize) -> Self {
        Self {
            next_id: 1,
            devices: Vec::new(),
            msi_vector_limit: Some(limit),
        }
    }

    /// Reserves the next device identifier.
    pub fn allocate_id(&mut self) -> DeviceId {
        let id = DeviceId::new(self.next_id);
        self.next_id = self.next_id.saturating_add(1);
        id
    }

    /// Registers a descriptor after validating resources.
    pub fn register(&mut self, device: RegisteredDevice) -> AxResult<DeviceId> {
        self.validate_resources(device.resources.as_slice())?;
        let id = device.id;
        self.devices.push(device);
        Ok(id)
    }

    /// Allocates and registers a descriptor from raw resource data.
    pub fn register_device(
        &mut self,
        name: &'static str,
        resources: ResourceSet,
        capabilities: DeviceCapabilities,
    ) -> AxResult<DeviceId> {
        self.validate_resources(resources.as_slice())?;
        let id = self.allocate_id();
        self.devices
            .push(RegisteredDevice::new(id, name, resources, capabilities));
        Ok(id)
    }

    /// Returns all registered device descriptors.
    pub fn devices(&self) -> &[RegisteredDevice] {
        &self.devices
    }

    fn validate_resources(&self, resources: &[Resource]) -> AxResult {
        if let Some(resource) = resources
            .iter()
            .find(|resource| resource.is_empty_or_invalid())
        {
            return ax_err!(
                InvalidInput,
                format_args!("invalid device resource: {resource:?}")
            );
        }

        for resource in resources {
            for existing in self
                .devices
                .iter()
                .flat_map(|device| device.resources.as_slice())
            {
                if resource.overlaps(existing) {
                    return ax_err!(
                        AddrInUse,
                        format_args!("device resource {resource:?} overlaps {existing:?}")
                    );
                }
            }
        }

        for (index, resource) in resources.iter().enumerate() {
            for other in &resources[..index] {
                if resource.overlaps(other) {
                    return ax_err!(
                        AddrInUse,
                        format_args!(
                            "device resource {resource:?} overlaps another resource {other:?}"
                        )
                    );
                }
            }
        }

        if let Some(limit) = self.msi_vector_limit {
            let used = self
                .devices
                .iter()
                .flat_map(|device| device.resources.as_slice())
                .map(msi_vectors)
                .sum::<usize>();
            let requested = resources.iter().map(msi_vectors).sum::<usize>();
            if used.saturating_add(requested) > limit {
                return ax_err!(
                    NoMemory,
                    format_args!(
                        "MSI vector request {requested} exceeds remaining capacity {}",
                        limit.saturating_sub(used)
                    )
                );
            }
        }

        Ok(())
    }
}

fn msi_vectors(resource: &Resource) -> usize {
    match resource {
        Resource::Msi { vectors } => *vectors,
        _ => 0,
    }
}

/// Unified bus router for legacy device handlers.
#[derive(Default)]
pub struct BusRouter {
    registry: DeviceRegistry,
    mmio: Vec<(DeviceId, Arc<dyn BaseMmioDeviceOps>)>,
    port: Vec<(DeviceId, Arc<dyn BasePortDeviceOps>)>,
    sysreg: Vec<(DeviceId, Arc<dyn BaseSysRegDeviceOps>)>,
}

impl BusRouter {
    /// Creates an empty bus router.
    pub const fn new() -> Self {
        Self {
            registry: DeviceRegistry::new(),
            mmio: Vec::new(),
            port: Vec::new(),
            sysreg: Vec::new(),
        }
    }

    /// Returns the device registry.
    pub const fn registry(&self) -> &DeviceRegistry {
        &self.registry
    }

    /// Registers a legacy MMIO device.
    pub fn register_mmio(
        &mut self,
        name: &'static str,
        device: Arc<dyn BaseMmioDeviceOps>,
    ) -> AxResult<DeviceId> {
        let id = self.registry.allocate_id();
        let descriptor = RegisteredDevice::new(
            id,
            name,
            ResourceSet::new().with(Resource::Mmio(device.address_range())),
            DeviceCapabilities::default(),
        );
        self.registry.register(descriptor)?;
        self.mmio.push((id, device));
        Ok(id)
    }

    /// Registers a legacy port I/O device.
    pub fn register_port(
        &mut self,
        name: &'static str,
        device: Arc<dyn BasePortDeviceOps>,
    ) -> AxResult<DeviceId> {
        let id = self.registry.allocate_id();
        let descriptor = RegisteredDevice::new(
            id,
            name,
            ResourceSet::new().with(Resource::Port(device.address_range())),
            DeviceCapabilities::default(),
        );
        self.registry.register(descriptor)?;
        self.port.push((id, device));
        Ok(id)
    }

    /// Registers a legacy system-register device.
    pub fn register_sysreg(
        &mut self,
        name: &'static str,
        device: Arc<dyn BaseSysRegDeviceOps>,
    ) -> AxResult<DeviceId> {
        let id = self.registry.allocate_id();
        let descriptor = RegisteredDevice::new(
            id,
            name,
            ResourceSet::new().with(Resource::SysReg(device.address_range())),
            DeviceCapabilities::default(),
        );
        self.registry.register(descriptor)?;
        self.sysreg.push((id, device));
        Ok(id)
    }

    /// Dispatches a unified bus access.
    pub fn dispatch(&self, access: BusAccess) -> AxResult<BusResponse> {
        match (access.address, access.operation) {
            (BusAddress::Mmio(addr), BusOperation::Read) => {
                let dev = self.find_mmio(addr)?;
                dev.handle_read(addr, access.width)
                    .map(|value| BusResponse::Read { value })
            }
            (BusAddress::Mmio(addr), BusOperation::Write { value }) => {
                let dev = self.find_mmio(addr)?;
                dev.handle_write(addr, access.width, value)?;
                Ok(BusResponse::WriteComplete)
            }
            (BusAddress::Port(port), BusOperation::Read) => {
                let dev = self.find_port(port)?;
                dev.handle_read(port, access.width)
                    .map(|value| BusResponse::Read { value })
            }
            (BusAddress::Port(port), BusOperation::Write { value }) => {
                let dev = self.find_port(port)?;
                dev.handle_write(port, access.width, value)?;
                Ok(BusResponse::WriteComplete)
            }
            (BusAddress::SysReg(addr), BusOperation::Read) => {
                let dev = self.find_sysreg(addr)?;
                dev.handle_read(addr, access.width)
                    .map(|value| BusResponse::Read { value })
            }
            (BusAddress::SysReg(addr), BusOperation::Write { value }) => {
                let dev = self.find_sysreg(addr)?;
                dev.handle_write(addr, access.width, value)?;
                Ok(BusResponse::WriteComplete)
            }
            (BusAddress::PciConfig { .. }, _) => {
                ax_err!(Unsupported, "PCI config routing is not installed")
            }
        }
    }

    fn find_mmio(&self, addr: axdevice_base::GuestPhysAddr) -> AxResult<&dyn BaseMmioDeviceOps> {
        self.mmio
            .iter()
            .map(|(_, device)| device.as_ref())
            .find(|device| device.address_range().contains(addr))
            .ok_or_else(|| ax_errno::ax_err_type!(NotFound, "MMIO device not found"))
    }

    fn find_port(&self, port: axdevice_base::Port) -> AxResult<&dyn BasePortDeviceOps> {
        self.port
            .iter()
            .map(|(_, device)| device.as_ref())
            .find(|device| device.address_range().contains(port))
            .ok_or_else(|| ax_errno::ax_err_type!(NotFound, "port I/O device not found"))
    }

    fn find_sysreg(&self, addr: axdevice_base::SysRegAddr) -> AxResult<&dyn BaseSysRegDeviceOps> {
        self.sysreg
            .iter()
            .map(|(_, device)| device.as_ref())
            .find(|device| device.address_range().contains(addr))
            .ok_or_else(|| ax_errno::ax_err_type!(NotFound, "system register device not found"))
    }
}
