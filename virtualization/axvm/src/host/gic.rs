//! AArch64 GIC host operations for the ArceOS-backed AxVM runtime.

use arm_gic_driver::v3::{
    ICH_ELRSR_EL2, ICH_HCR_EL2, ICH_LR_EL2, ICH_VTR_EL2, ReadWriteable, Readable, ich_lr_el2_get,
    ich_lr_el2_write,
};
use ax_errno::{AxError, AxResult, ax_err};
use ax_memory_addr::{PhysAddr, VirtAddr};

use super::{HostMemory, arceos, default_host};

const SPECIAL_INTERRUPT_START: usize = 1020;

fn with_gic<T>(f: impl FnOnce(&mut rdif_intc::Intc) -> T) -> T {
    let mut gic = rdrive::get_one::<rdif_intc::Intc>()
        .expect("failed to get GIC driver")
        .lock()
        .expect("failed to lock GIC driver");
    f(&mut gic)
}

pub(crate) fn inject_interrupt(irq: usize) -> AxResult {
    if irq >= SPECIAL_INTERRUPT_START {
        return ax_err!(
            InvalidInput,
            format_args!(
                "invalid AArch64 virtual interrupt {irq}; valid IDs are \
                 0..{SPECIAL_INTERRUPT_START}"
            )
        );
    }

    debug!("Injecting virtual interrupt: {irq}");

    let driver = rdrive::get_one::<rdif_intc::Intc>().ok_or(AxError::NoSuchDevice)?;
    let mut gic = driver.lock().map_err(|_| AxError::ResourceBusy)?;

    if let Some(gic) = gic.typed_mut::<arm_gic_driver::v2::Gic>() {
        use arm_gic_driver::{
            IntId,
            v2::{VirtualInterruptConfig, VirtualInterruptState},
        };

        let Some(gich) = gic.hypervisor_interface() else {
            return ax_err!(NoSuchDevice, "GICv2 hypervisor interface is unavailable");
        };
        let intid = unsafe { IntId::raw(irq as u32) };
        let lr_count = gich.get_list_register_count();
        for index in 0..lr_count {
            let current = gich.get_virtual_interrupt(index);
            if current.virtual_id == intid
                && !matches!(current.state, VirtualInterruptState::Invalid)
            {
                debug!("Virtual interrupt {irq} already pending/active in LR{index}, skipping");
                return Ok(());
            }
        }

        let Some(free_lr) = (0..lr_count).find(|index| gich.is_list_register_empty(*index)) else {
            warn!("No free GICv2 list register for virtual interrupt {irq}; deferring injection");
            return Err(AxError::WouldBlock);
        };

        gich.enable();
        gich.set_virtual_interrupt(
            free_lr,
            VirtualInterruptConfig::software(
                intid,
                None,
                0,
                VirtualInterruptState::Pending,
                false,
                true,
            ),
        );
        return Ok(());
    }

    if gic.typed_mut::<arm_gic_driver::v3::Gic>().is_some() {
        return inject_interrupt_gic_v3(irq);
    }

    ax_err!(NoSuchDevice, "no supported GIC driver found")
}

fn inject_interrupt_gic_v3(vector: usize) -> AxResult {
    debug!("Injecting virtual interrupt: vector={vector}");
    let elsr = ICH_ELRSR_EL2.read(ICH_ELRSR_EL2::STATUS);
    let lr_num = ICH_VTR_EL2.read(ICH_VTR_EL2::LISTREGS) as usize + 1;

    let mut free_lr = None;
    for i in 0..lr_num {
        if (1 << i) & elsr > 0 {
            free_lr.get_or_insert(i);
            continue;
        }

        let lr_val = ich_lr_el2_get(i);
        if lr_val.read(ICH_LR_EL2::VINTID) == vector as u64
            && lr_val.matches_any(&[ICH_LR_EL2::STATE::Pending, ICH_LR_EL2::STATE::Active])
        {
            debug!("Virtual interrupt {vector} already pending/active in LR{i}, skipping");
            return Ok(());
        }
    }

    let Some(free_lr) = free_lr.or_else(|| {
        (0..lr_num).find(|&i| ich_lr_el2_get(i).matches_all(ICH_LR_EL2::STATE::Invalid))
    }) else {
        warn!("No free GICv3 list register for virtual interrupt {vector}; deferring injection");
        return Err(AxError::WouldBlock);
    };

    ich_lr_el2_write(
        free_lr,
        ICH_LR_EL2::VINTID.val(vector as u64) + ICH_LR_EL2::STATE::Pending + ICH_LR_EL2::GROUP::SET,
    );

    if !ICH_HCR_EL2.is_set(ICH_HCR_EL2::EN) {
        warn!("Virtual interrupt interface not enabled, enabling now");
        ICH_HCR_EL2.modify(ICH_HCR_EL2::EN::SET);
    }

    debug!("Virtual interrupt {vector} injected successfully in LR{free_lr}");
    Ok(())
}

pub(crate) fn read_gicd_iidr() -> u32 {
    with_gic(|gic| {
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v2::Gic>() {
            return gic.iidr_raw();
        }
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v3::Gic>() {
            return gic.iidr_raw();
        }
        panic!("no GIC driver found");
    })
}

pub(crate) fn read_gicd_typer() -> u32 {
    with_gic(|gic| {
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v2::Gic>() {
            return gic.typer_raw();
        }
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v3::Gic>() {
            return gic.typer_raw();
        }
        panic!("no GIC driver found");
    })
}

pub(crate) fn host_gicd_base() -> PhysAddr {
    with_gic(|gic| {
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v2::Gic>() {
            return default_host().virt_to_phys(VirtAddr::from(usize::from(gic.gicd_addr())));
        }
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v3::Gic>() {
            return default_host().virt_to_phys(VirtAddr::from(usize::from(gic.gicd_addr())));
        }
        panic!("no GIC driver found");
    })
}

pub(crate) fn host_gicr_base() -> PhysAddr {
    with_gic(|gic| {
        if let Some(gic) = gic.typed_mut::<arm_gic_driver::v3::Gic>() {
            return default_host().virt_to_phys(VirtAddr::from(usize::from(gic.gicr_addr())));
        }
        panic!("no GICv3 driver found");
    })
}

pub(crate) fn handle_current_irq() -> Option<usize> {
    // AArch64 ArceOS platform IRQ handlers acknowledge the current IRQ
    // internally. The raw vector argument is ignored by current GIC-backed
    // platforms, so keep the ack/EOI ownership inside the platform handler.
    arceos::handle_host_irq(0)
}

pub(crate) fn fetch_irq() -> usize {
    handle_current_irq().unwrap_or(0)
}
