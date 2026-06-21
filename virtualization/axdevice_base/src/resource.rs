//! Device resource and capability descriptors.

use alloc::vec::Vec;
use core::ops::Range;

use ax_memory_addr::AddrRange;
use axvm_types::{GuestPhysAddr, GuestPhysAddrRange, InterruptTriggerMode, IrqLineId};

use crate::{Port, PortRange, SysRegAddrRange};

/// Stable identifier assigned to a registered emulated device.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct DeviceId(pub u64);

impl DeviceId {
    /// Creates a new device identifier from a raw value.
    pub const fn new(id: u64) -> Self {
        Self(id)
    }

    /// Returns the raw identifier value.
    pub const fn as_u64(self) -> u64 {
        self.0
    }
}

/// Kind of bus used by a guest access.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusKind {
    /// Memory-mapped I/O bus.
    Mmio,
    /// x86-style port I/O bus.
    PortIo,
    /// Architecture system register bus.
    SysReg,
    /// PCI configuration space.
    PciConfig,
}

/// PCI function selector.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PciFunction {
    /// PCI segment number.
    pub segment: u16,
    /// PCI bus number.
    pub bus: u8,
    /// PCI device number.
    pub device: u8,
    /// PCI function number.
    pub function: u8,
}

/// MSI or MSI-X message payload.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MsiMessage {
    /// Message address.
    pub address: u64,
    /// Message data.
    pub data: u32,
}

/// Target selected for a routed interrupt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IrqTarget {
    /// Bootstrap vCPU.
    Bootstrap,
    /// A single VM-local vCPU.
    Vcpu(usize),
    /// All VM-local vCPUs.
    All,
}

/// One resource claimed by a device.
#[derive(Clone, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum Resource {
    /// MMIO guest physical address range.
    Mmio(GuestPhysAddrRange),
    /// Port I/O inclusive range.
    Port(PortRange),
    /// System register inclusive range.
    SysReg(SysRegAddrRange),
    /// VM-local IRQ line.
    Irq {
        /// Interrupt line identifier.
        line: IrqLineId,
        /// Required trigger mode.
        trigger: InterruptTriggerMode,
        /// Preferred target.
        target: IrqTarget,
    },
    /// MSI/MSI-X vector allocation request.
    Msi {
        /// Number of vectors requested.
        vectors: usize,
    },
    /// DMA aperture visible to the device.
    Dma {
        /// Optional guest physical aperture.
        aperture: Option<GuestPhysAddrRange>,
    },
    /// PCI BAR requirement.
    PciBar {
        /// BAR index.
        bar: u8,
        /// Assigned guest physical address, if the BAR is already placed.
        addr: Option<GuestPhysAddrRange>,
        /// BAR size in bytes.
        size: usize,
        /// Whether the BAR is prefetchable.
        prefetchable: bool,
    },
}

impl Resource {
    /// Returns the bus kind for addressable resources.
    pub const fn bus_kind(&self) -> Option<BusKind> {
        match self {
            Self::Mmio(_) => Some(BusKind::Mmio),
            Self::Port(_) => Some(BusKind::PortIo),
            Self::SysReg(_) => Some(BusKind::SysReg),
            Self::Irq { .. } | Self::Msi { .. } | Self::Dma { .. } | Self::PciBar { .. } => None,
        }
    }

    /// Returns whether the resource is empty or malformed.
    pub fn is_empty_or_invalid(&self) -> bool {
        match self {
            Self::Mmio(range) => range.is_empty(),
            Self::Port(range) => range.start > range.end,
            Self::SysReg(range) => range.start > range.end,
            Self::Irq { .. } => false,
            Self::Msi { vectors } => *vectors == 0,
            Self::Dma { aperture } => aperture.is_some_and(|range| range.is_empty()),
            Self::PciBar {
                addr, size, bar, ..
            } => *size == 0 || *bar >= 6 || addr.is_some_and(|range| range.is_empty()),
        }
    }

    /// Returns whether this resource overlaps another resource.
    pub fn overlaps(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Mmio(a), Self::Mmio(b)) => a.overlaps(*b),
            (Self::Port(a), Self::Port(b)) => {
                a.start <= b.end && b.start <= a.end && a.start <= a.end && b.start <= b.end
            }
            (Self::SysReg(a), Self::SysReg(b)) => {
                a.start <= b.end && b.start <= a.end && a.start <= a.end && b.start <= b.end
            }
            (Self::Irq { line: a, .. }, Self::Irq { line: b, .. }) => a == b,
            (
                Self::Dma {
                    aperture: Some(a), ..
                },
                Self::Dma {
                    aperture: Some(b), ..
                },
            ) => a.overlaps(*b),
            (Self::PciBar { addr: Some(a), .. }, Self::PciBar { addr: Some(b), .. }) => {
                a.overlaps(*b)
            }
            _ => false,
        }
    }
}

/// A group of resources claimed by one device.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ResourceSet {
    resources: Vec<Resource>,
}

impl ResourceSet {
    /// Creates an empty resource set.
    pub const fn new() -> Self {
        Self {
            resources: Vec::new(),
        }
    }

    /// Creates a resource set from a vector.
    pub fn from_vec(resources: Vec<Resource>) -> Self {
        Self { resources }
    }

    /// Adds one resource.
    pub fn push(&mut self, resource: Resource) {
        self.resources.push(resource);
    }

    /// Adds one resource and returns the set.
    pub fn with(mut self, resource: Resource) -> Self {
        self.push(resource);
        self
    }

    /// Returns all resources.
    pub fn as_slice(&self) -> &[Resource] {
        &self.resources
    }

    /// Returns whether the set contains no resources.
    pub fn is_empty(&self) -> bool {
        self.resources.is_empty()
    }

    /// Returns the first invalid resource, if any.
    pub fn first_invalid(&self) -> Option<&Resource> {
        self.resources
            .iter()
            .find(|resource| resource.is_empty_or_invalid())
    }
}

/// Device capability flags consumed by VM lifecycle and bus policy.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DeviceCapabilities {
    /// Device has a reset implementation.
    pub reset: bool,
    /// Device has a suspend implementation.
    pub suspend: bool,
    /// Device has a resume implementation.
    pub resume: bool,
    /// Device can initiate DMA.
    pub dma: bool,
    /// Device supports MSI.
    pub msi: bool,
    /// Device supports MSI-X.
    pub msix: bool,
}

impl DeviceCapabilities {
    /// Empty capability set.
    pub const NONE: Self = Self {
        reset: false,
        suspend: false,
        resume: false,
        dma: false,
        msi: false,
        msix: false,
    };
}

/// A declarative description supplied by one device.
pub trait DeviceDescriptor {
    /// Returns the assigned device identifier.
    fn device_id(&self) -> DeviceId;

    /// Returns the human-readable device name.
    fn name(&self) -> &str;

    /// Returns the resources claimed by the device.
    fn resources(&self) -> ResourceSet;

    /// Returns capability flags for the device.
    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities::default()
    }
}

/// Creates an MMIO resource from base and size.
pub fn mmio_resource(base: GuestPhysAddr, size: usize) -> Option<Resource> {
    let end = base.as_usize().checked_add(size)?;
    Some(Resource::Mmio(AddrRange::new(base, end.into())))
}

/// Creates an inclusive port resource from base and size.
pub fn port_resource(base: Port, size: u16) -> Option<Resource> {
    let end = base.0.checked_add(size.checked_sub(1)?)?;
    Some(Resource::Port(PortRange::new(base, Port(end))))
}

/// Converts a half-open host-style range into a DMA resource.
pub fn dma_resource(range: Range<usize>) -> Option<Resource> {
    let gpa_range = AddrRange::new(range.start.into(), range.end.into());
    (!gpa_range.is_empty()).then_some(Resource::Dma {
        aperture: Some(gpa_range),
    })
}

/// Creates a PCI BAR resource. `addr` is optional while firmware or PCI code
/// is still assigning the final placement.
pub fn pci_bar_resource(
    bar: u8,
    addr: Option<GuestPhysAddrRange>,
    size: usize,
    prefetchable: bool,
) -> Option<Resource> {
    let resource = Resource::PciBar {
        bar,
        addr,
        size,
        prefetchable,
    };
    (!resource.is_empty_or_invalid()).then_some(resource)
}
