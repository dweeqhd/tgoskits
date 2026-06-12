use alloc::vec::Vec;

use ax_errno::{AxResult, ax_err};
use ax_kspin::SpinNoPreempt as Mutex;
use ax_memory_addr::AddrRange;
use axdevice_base::{AccessWidth, BaseDeviceOps, EmuDeviceType};
use axvm_types::{GuestPhysAddr, GuestPhysAddrRange};

const IOAPIC_BASE: usize = 0xfec0_0000;
const IOAPIC_SIZE: usize = 0x1000;

const IOREGSEL: usize = 0x00;
const IOWIN: usize = 0x10;

const IOAPIC_ID: u32 = 0x00;
const IOAPIC_VER: u32 = 0x01;
const IOAPIC_ARB: u32 = 0x02;
const IOREDTBL_BASE: u32 = 0x10;

const IOAPIC_ID_VALUE: u32 = 1 << 24;
const IOAPIC_VERSION_VALUE: u32 = 0x11 | ((MAX_REDIRECTION_ENTRY as u32) << 16);
const MAX_REDIRECTION_ENTRY: usize = 23;
/// Number of virtual IO APIC input lines.
pub const IOAPIC_GSI_COUNT: usize = MAX_REDIRECTION_ENTRY + 1;
const REDIRECTION_ENTRY_MASKED: u64 = 1 << 16;
const REDIRECTION_ENTRY_TRIGGER_MODE: u64 = 1 << 15;
const REDIRECTION_ENTRY_REMOTE_IRR: u64 = 1 << 14;
const REDIRECTION_ENTRY_DELIVERY_MODE_MASK: u64 = 0b111 << 8;

#[derive(Debug)]
struct IoApicState {
    selector: u32,
    redirection_table: [u64; IOAPIC_GSI_COUNT],
    asserted: [bool; IOAPIC_GSI_COUNT],
    assertion_delivered: [bool; IOAPIC_GSI_COUNT],
    pending_level: [bool; IOAPIC_GSI_COUNT],
}

impl IoApicState {
    const fn new() -> Self {
        Self {
            selector: 0,
            redirection_table: [REDIRECTION_ENTRY_MASKED; IOAPIC_GSI_COUNT],
            asserted: [false; IOAPIC_GSI_COUNT],
            assertion_delivered: [false; IOAPIC_GSI_COUNT],
            pending_level: [false; IOAPIC_GSI_COUNT],
        }
    }

    fn interrupt_for_entry(
        &mut self,
        gsi: usize,
        queue_if_remote_irr: bool,
    ) -> Option<IoApicInterrupt> {
        let entry = self.redirection_table.get_mut(gsi)?;
        if *entry & REDIRECTION_ENTRY_MASKED != 0 {
            return None;
        }

        if *entry & REDIRECTION_ENTRY_DELIVERY_MODE_MASK != 0 {
            debug!("vIOAPIC GSI {gsi} uses unsupported delivery mode entry {entry:#x}");
            return None;
        }

        let vector = (*entry & 0xff) as u8;
        if vector < 16 {
            return None;
        }

        let level_triggered = *entry & REDIRECTION_ENTRY_TRIGGER_MODE != 0;
        if level_triggered {
            if *entry & REDIRECTION_ENTRY_REMOTE_IRR != 0 {
                if queue_if_remote_irr {
                    self.pending_level[gsi] = true;
                }
                return None;
            }
            *entry |= REDIRECTION_ENTRY_REMOTE_IRR;
        }

        Some(IoApicInterrupt {
            vector,
            level_triggered,
        })
    }
}

/// A routed interrupt from the virtual IO APIC.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IoApicInterrupt {
    /// Guest interrupt vector.
    pub vector: u8,
    /// Whether the redirection entry is level-triggered.
    pub level_triggered: bool,
}

/// A minimal emulated x86 IO APIC.
pub struct EmulatedIoApic {
    base: GuestPhysAddr,
    size: usize,
    state: Mutex<IoApicState>,
}

impl EmulatedIoApic {
    /// Create a new `EmulatedIoApic`.
    pub fn new(base: GuestPhysAddr, size: Option<usize>) -> Self {
        Self {
            base,
            size: size.unwrap_or(IOAPIC_SIZE),
            state: Mutex::new(IoApicState::new()),
        }
    }

    /// Create an IO APIC at the default PC-compatible GPA.
    pub fn new_default() -> Self {
        Self::new(GuestPhysAddr::from_usize(IOAPIC_BASE), Some(IOAPIC_SIZE))
    }

    /// Return the guest interrupt vector programmed for a GSI.
    pub fn vector_for_gsi(&self, gsi: usize) -> Option<u8> {
        let state = self.state.lock();
        let entry = *state.redirection_table.get(gsi)?;
        if entry & REDIRECTION_ENTRY_MASKED != 0 {
            return None;
        }

        if entry & REDIRECTION_ENTRY_DELIVERY_MODE_MASK != 0 {
            debug!("vIOAPIC GSI {gsi} uses unsupported delivery mode entry {entry:#x}");
            return None;
        }

        let vector = (entry & 0xff) as u8;
        if vector < 16 {
            return None;
        }

        Some(vector)
    }

    /// Set the asserted state of an IO APIC input line.
    pub fn set_gsi_level(&self, gsi: usize, asserted: bool) -> Option<IoApicInterrupt> {
        let mut state = self.state.lock();
        let previous = *state.asserted.get(gsi)?;
        state.asserted[gsi] = asserted;

        if !asserted {
            state.assertion_delivered[gsi] = false;
            return None;
        }
        if previous {
            return None;
        }

        let interrupt = state.interrupt_for_entry(gsi, false);
        if interrupt.is_some() {
            state.assertion_delivered[gsi] = true;
        }
        interrupt
    }

    /// Deliver one pulse on an IO APIC input line.
    pub fn pulse_gsi(&self, gsi: usize) -> Option<IoApicInterrupt> {
        self.state.lock().interrupt_for_entry(gsi, true)
    }

    /// Route asserted inputs that became deliverable after guest reconfiguration.
    pub fn route_asserted_lines(&self) -> Vec<IoApicInterrupt> {
        let mut state = self.state.lock();
        let mut interrupts = Vec::new();
        for gsi in 0..IOAPIC_GSI_COUNT {
            if !state.asserted[gsi] || state.assertion_delivered[gsi] {
                continue;
            }
            if let Some(interrupt) = state.interrupt_for_entry(gsi, false) {
                state.assertion_delivered[gsi] = true;
                interrupts.push(interrupt);
            }
        }
        interrupts
    }

    /// Process an EOI broadcast from the local APIC.
    pub fn end_of_interrupt(&self, vector: u8) -> Option<IoApicInterrupt> {
        let mut state = self.state.lock();
        for gsi in 0..IOAPIC_GSI_COUNT {
            let entry = &mut state.redirection_table[gsi];
            if (*entry & 0xff) as u8 != vector
                || *entry & REDIRECTION_ENTRY_TRIGGER_MODE == 0
                || *entry & REDIRECTION_ENTRY_REMOTE_IRR == 0
            {
                continue;
            }

            *entry &= !REDIRECTION_ENTRY_REMOTE_IRR;
            if state.asserted[gsi] || core::mem::take(&mut state.pending_level[gsi]) {
                let interrupt = state.interrupt_for_entry(gsi, false);
                if interrupt.is_some() {
                    state.assertion_delivered[gsi] = true;
                }
                return interrupt;
            }
        }

        None
    }

    fn offset(&self, addr: GuestPhysAddr) -> usize {
        addr.as_usize() - self.base.as_usize()
    }

    fn read_selected_register(state: &IoApicState) -> AxResult<u32> {
        match state.selector {
            IOAPIC_ID => Ok(IOAPIC_ID_VALUE),
            IOAPIC_VER => Ok(IOAPIC_VERSION_VALUE),
            IOAPIC_ARB => Ok(IOAPIC_ID_VALUE),
            reg @ IOREDTBL_BASE..=0x3f => {
                let index = ((reg - IOREDTBL_BASE) / 2) as usize;
                if index >= IOAPIC_GSI_COUNT {
                    return ax_err!(InvalidInput, "IOAPIC redirection index out of range");
                }
                let entry = state.redirection_table[index];
                if (reg - IOREDTBL_BASE) & 1 == 0 {
                    Ok(entry as u32)
                } else {
                    Ok((entry >> 32) as u32)
                }
            }
            reg => {
                debug!("vIOAPIC read from unsupported register {reg:#x}");
                Ok(0)
            }
        }
    }

    fn write_selected_register(state: &mut IoApicState, value: u32) -> AxResult {
        match state.selector {
            IOAPIC_ID | IOAPIC_VER | IOAPIC_ARB => Ok(()),
            reg @ IOREDTBL_BASE..=0x3f => {
                let index = ((reg - IOREDTBL_BASE) / 2) as usize;
                if index >= IOAPIC_GSI_COUNT {
                    return ax_err!(InvalidInput, "IOAPIC redirection index out of range");
                }
                let entry = &mut state.redirection_table[index];
                if (reg - IOREDTBL_BASE) & 1 == 0 {
                    let old_low = *entry & !REDIRECTION_ENTRY_REMOTE_IRR & 0xffff_ffff;
                    let new_low = (value as u64) & !REDIRECTION_ENTRY_REMOTE_IRR;
                    let remote_irr = if old_low == new_low {
                        *entry & REDIRECTION_ENTRY_REMOTE_IRR
                    } else {
                        state.pending_level[index] = false;
                        state.assertion_delivered[index] = false;
                        0
                    };
                    *entry = (*entry & !0xffff_ffff) | new_low | remote_irr;
                    if *entry & REDIRECTION_ENTRY_MASKED != 0 {
                        state.pending_level[index] = false;
                        state.assertion_delivered[index] = false;
                    }
                } else {
                    *entry = (*entry & 0xffff_ffff) | ((value as u64) << 32);
                }
                Ok(())
            }
            reg => {
                debug!("vIOAPIC write to unsupported register {reg:#x} = {value:#x}");
                Ok(())
            }
        }
    }
}

impl Default for EmulatedIoApic {
    fn default() -> Self {
        Self::new_default()
    }
}

impl BaseDeviceOps<GuestPhysAddrRange> for EmulatedIoApic {
    fn emu_type(&self) -> EmuDeviceType {
        EmuDeviceType::X86IoApic
    }

    fn address_range(&self) -> GuestPhysAddrRange {
        AddrRange::new(
            self.base,
            GuestPhysAddr::from_usize(self.base.as_usize() + self.size),
        )
    }

    fn handle_read(&self, addr: GuestPhysAddr, width: AccessWidth) -> AxResult<usize> {
        if !matches!(width, AccessWidth::Dword | AccessWidth::Qword) {
            return ax_err!(Unsupported, "unsupported IOAPIC read width");
        }

        let offset = self.offset(addr);
        let state = self.state.lock();
        match offset {
            IOREGSEL => Ok(state.selector as usize),
            IOWIN => Ok(Self::read_selected_register(&state)? as usize),
            _ => {
                debug!("vIOAPIC read from unsupported offset {offset:#x}");
                Ok(0)
            }
        }
    }

    fn handle_write(&self, addr: GuestPhysAddr, width: AccessWidth, val: usize) -> AxResult {
        if !matches!(width, AccessWidth::Dword | AccessWidth::Qword) {
            return ax_err!(Unsupported, "unsupported IOAPIC write width");
        }

        let offset = self.offset(addr);
        let mut state = self.state.lock();
        match offset {
            IOREGSEL => {
                state.selector = val as u32;
                Ok(())
            }
            IOWIN => Self::write_selected_register(&mut state, val as u32),
            _ => {
                debug!("vIOAPIC write to unsupported offset {offset:#x} = {val:#x}");
                Ok(())
            }
        }
    }
}
