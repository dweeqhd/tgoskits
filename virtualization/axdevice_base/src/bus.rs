//! Unified bus transaction types.

use axvm_types::GuestPhysAddr;

use crate::{AccessWidth, BusKind, Port, SysRegAddr};

/// Address carried by a bus transaction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusAddress {
    /// MMIO guest physical address.
    Mmio(GuestPhysAddr),
    /// Port I/O address.
    Port(Port),
    /// System register address.
    SysReg(SysRegAddr),
    /// PCI configuration address.
    PciConfig {
        /// PCI segment number.
        segment: u16,
        /// PCI bus number.
        bus: u8,
        /// PCI device number.
        device: u8,
        /// PCI function number.
        function: u8,
        /// Register offset.
        offset: u16,
    },
}

impl BusAddress {
    /// Returns the bus kind implied by this address.
    pub const fn kind(self) -> BusKind {
        match self {
            Self::Mmio(_) => BusKind::Mmio,
            Self::Port(_) => BusKind::PortIo,
            Self::SysReg(_) => BusKind::SysReg,
            Self::PciConfig { .. } => BusKind::PciConfig,
        }
    }
}

/// Operation carried by a bus transaction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusOperation {
    /// Read from the target address.
    Read,
    /// Write the given value to the target address.
    Write {
        /// Value to write.
        value: usize,
    },
}

/// A normalized VM-exit device access.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BusAccess {
    /// Bus address.
    pub address: BusAddress,
    /// Access width.
    pub width: AccessWidth,
    /// Read or write operation.
    pub operation: BusOperation,
}

impl BusAccess {
    /// Creates a read access.
    pub const fn read(address: BusAddress, width: AccessWidth) -> Self {
        Self {
            address,
            width,
            operation: BusOperation::Read,
        }
    }

    /// Creates a write access.
    pub const fn write(address: BusAddress, width: AccessWidth, value: usize) -> Self {
        Self {
            address,
            width,
            operation: BusOperation::Write { value },
        }
    }

    /// Returns the bus kind.
    pub const fn kind(self) -> BusKind {
        self.address.kind()
    }
}

/// Result returned by a bus handler.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BusResponse {
    /// Read completed with the returned value.
    Read {
        /// Read value.
        value: usize,
    },
    /// Write completed.
    WriteComplete,
}

/// Structured device access error used by routers and registries.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeviceError {
    /// No device owns the bus address.
    NotFound,
    /// The access width is not supported at the target address.
    InvalidWidth,
    /// A write targeted a read-only register.
    ReadOnly,
    /// A read targeted a write-only register.
    WriteOnly,
    /// A resource conflicts with an existing resource.
    ResourceConflict,
    /// The target architecture or backend does not support the request.
    Unsupported,
}

impl DeviceError {
    /// Converts a structured device error into the nearest shared errno kind.
    pub const fn as_ax_error(self) -> ax_errno::AxError {
        match self {
            Self::NotFound => ax_errno::AxError::NotFound,
            Self::InvalidWidth | Self::ReadOnly | Self::WriteOnly => {
                ax_errno::AxError::InvalidInput
            }
            Self::ResourceConflict => ax_errno::AxError::AddrInUse,
            Self::Unsupported => ax_errno::AxError::Unsupported,
        }
    }
}
