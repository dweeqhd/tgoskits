use core::{
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};

use axvm_types::VMInterruptMode;

use crate::{
    host::irq,
    irq::x86::{HOST_IOAPIC_GSI_COUNT, HOST_IOAPIC_VECTOR_BASE, host_vector_to_gsi},
    runtime::{VCpuRef, VMRef},
};

const PIT_TIMER_GSI: usize = 0;
static IOAPIC_IRQ_FORWARDING_ENABLED: AtomicBool = AtomicBool::new(false);
static IOAPIC_IRQ_HOOK_REGISTERED: AtomicBool = AtomicBool::new(false);
static IOAPIC_IRQ_FORWARD_VM_ID: AtomicUsize = AtomicUsize::new(usize::MAX);
static IOAPIC_IRQ_PENDING: AtomicUsize = AtomicUsize::new(0);
static IOAPIC_IRQ_HANDLES: [AtomicUsize; HOST_IOAPIC_GSI_COUNT] =
    [const { AtomicUsize::new(0) }; HOST_IOAPIC_GSI_COUNT];

pub fn poll_devices(vm: &VMRef) {
    let now_ns = crate::host::arceos::monotonic_time_nanos();
    for device in vm.get_devices().iter_pollable_dev() {
        if let Err(err) = device.poll(now_ns) {
            warn!("failed to poll a VM[{}] device: {err:?}", vm.id());
        }
    }
}

pub fn drain_routed_irqs(vm: &crate::AxVM, vcpu: &VCpuRef) {
    if let Err(err) = vm.interrupt_fabric().drain_pending(vcpu.id(), |irq| {
        vcpu.inject_interrupt_with_trigger(irq.vector as usize, irq.trigger)
    }) {
        warn!(
            "failed to drain routed interrupts for VM[{}] VCpu[{}]: {err:?}",
            vm.id(),
            vcpu.id()
        );
    }
}

pub fn forward_passthrough_irq_from_vmexit(vm: &VMRef, vector: usize) {
    if vm.interrupt_mode() != VMInterruptMode::Passthrough
        || !vm.interrupt_fabric().has_controller()
    {
        return;
    }

    if vector == HOST_IOAPIC_VECTOR_BASE + PIT_TIMER_GSI {
        return;
    }

    if !ioapic_irq_hook_registered(vector) {
        forward_host_vector(vm, vector);
    }
}

pub fn handle_eoi(vm: &VMRef, vcpu: &VCpuRef, vector: u8) {
    if let Err(err) = vm.interrupt_fabric().eoi(vcpu.id(), vector) {
        warn!(
            "failed to process VM[{}] VCpu[{}] EOI for vector {vector:#x}: {err:?}",
            vm.id(),
            vcpu.id()
        );
    }
}

pub fn drain_pending_ioapic_irqs(vm: &VMRef) {
    if vm.interrupt_mode() != VMInterruptMode::Passthrough
        || !vm.interrupt_fabric().has_controller()
    {
        return;
    }

    if !IOAPIC_IRQ_HOOK_REGISTERED.load(Ordering::Acquire) {
        return;
    }

    if IOAPIC_IRQ_FORWARD_VM_ID.load(Ordering::Acquire) != vm.id() {
        return;
    }

    loop {
        let pending = IOAPIC_IRQ_PENDING.swap(0, Ordering::AcqRel);
        if pending == 0 {
            break;
        }

        for gsi in 0..HOST_IOAPIC_GSI_COUNT {
            if pending & (1usize << gsi) == 0 {
                continue;
            }
            if let Err(err) = vm.interrupt_fabric().forward_host_irq(gsi) {
                trace!(
                    "VM[{}] host GSI {gsi} has no injectable virtual IO APIC route: {err:?}",
                    vm.id()
                );
            }
        }
    }
}

pub fn enable_ioapic_irq_forwarding(vm: &VMRef) {
    if vm.interrupt_mode() != VMInterruptMode::Passthrough
        || !vm.interrupt_fabric().has_controller()
    {
        return;
    }

    match IOAPIC_IRQ_FORWARD_VM_ID.compare_exchange(
        usize::MAX,
        vm.id(),
        Ordering::AcqRel,
        Ordering::Acquire,
    ) {
        Ok(_) => {}
        Err(current_vm_id) if current_vm_id == vm.id() => {}
        Err(current_vm_id) => {
            warn!(
                "cannot enable host IOAPIC forwarding for VM[{}]: VM[{current_vm_id}] already \
                 owns the forwarding target",
                vm.id()
            );
            return;
        }
    }

    if IOAPIC_IRQ_FORWARDING_ENABLED
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return;
    }

    let mut registered = 0;
    for vector in HOST_IOAPIC_VECTOR_BASE..HOST_IOAPIC_VECTOR_BASE + HOST_IOAPIC_GSI_COUNT {
        let gsi = vector - HOST_IOAPIC_VECTOR_BASE;
        if IOAPIC_IRQ_HANDLES[gsi].load(Ordering::Acquire) != 0 {
            continue;
        }
        match irq::request_shared_irq(vector, ioapic_irq_forwarding_handler, NonNull::dangling()) {
            Ok(handle) => {
                IOAPIC_IRQ_HANDLES[gsi].store(handle.id() as usize, Ordering::Release);
                registered += 1;
            }
            Err(err) => {
                warn!(
                    "failed to request x86 IOAPIC forwarding IRQ action for vector {vector:#x}: \
                     {err:?}"
                );
            }
        }
    }
    if registered != 0 {
        IOAPIC_IRQ_HOOK_REGISTERED.store(true, Ordering::Release);
    } else {
        IOAPIC_IRQ_FORWARDING_ENABLED.store(false, Ordering::Release);
        let _ = IOAPIC_IRQ_FORWARD_VM_ID.compare_exchange(
            vm.id(),
            usize::MAX,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
        warn!("failed to register any x86 IOAPIC forwarding IRQ actions");
        return;
    }
    info!(
        "Enabled x86 IOAPIC IRQ forwarding for host vectors {:#x}..{:#x} ({} newly registered)",
        HOST_IOAPIC_VECTOR_BASE,
        HOST_IOAPIC_VECTOR_BASE + HOST_IOAPIC_GSI_COUNT - 1,
        registered
    );
}

fn ioapic_irq_hook_registered(vector: usize) -> bool {
    let Some(gsi) = host_vector_to_gsi(vector) else {
        return false;
    };
    IOAPIC_IRQ_HANDLES[gsi].load(Ordering::Acquire) != 0
}

pub fn disable_ioapic_irq_forwarding_for_vm(vm_id: usize) {
    if IOAPIC_IRQ_FORWARD_VM_ID.load(Ordering::Acquire) != vm_id {
        return;
    }

    IOAPIC_IRQ_FORWARD_VM_ID.store(usize::MAX, Ordering::Release);
    IOAPIC_IRQ_PENDING.store(0, Ordering::Release);
}

fn forward_host_vector(vm: &VMRef, vector: usize) {
    let Some(host_gsi) = host_vector_to_gsi(vector) else {
        return;
    };
    if let Err(err) = vm.interrupt_fabric().forward_host_irq(host_gsi) {
        trace!(
            "VM[{}] passthrough vector {vector:#x} host GSI {host_gsi} is not routable: {err:?}",
            vm.id()
        );
    }
}

unsafe fn ioapic_irq_forwarding_handler(
    ctx: irq::IrqContext,
    _data: NonNull<()>,
) -> irq::IrqReturn {
    let Some(gsi) = host_vector_to_gsi(ctx.irq.0) else {
        return irq::IrqReturn::Unhandled;
    };

    if IOAPIC_IRQ_FORWARD_VM_ID.load(Ordering::Acquire) == usize::MAX {
        return irq::IrqReturn::Unhandled;
    }

    IOAPIC_IRQ_PENDING.fetch_or(1usize << gsi, Ordering::AcqRel);
    irq::IrqReturn::Handled
}
