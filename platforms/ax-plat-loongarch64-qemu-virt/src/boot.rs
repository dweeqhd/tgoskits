use ax_page_table_entry::{GenericPTE, MappingFlags, loongarch64::LA64PTE};
use ax_plat::mem::{Aligned4K, pa, va};

use crate::config::plat::{BOOT_STACK_SIZE, PHYS_BOOT_OFFSET, PHYS_VIRT_OFFSET};

#[unsafe(link_section = ".bss.stack")]
static mut BOOT_STACK: [u8; BOOT_STACK_SIZE] = [0; BOOT_STACK_SIZE];

#[unsafe(link_section = ".data")]
static mut BOOT_PT_L0: Aligned4K<[LA64PTE; 512]> = Aligned4K::new([LA64PTE::empty(); 512]);

#[unsafe(link_section = ".data")]
static mut BOOT_PT_L1: Aligned4K<[LA64PTE; 512]> = Aligned4K::new([LA64PTE::empty(); 512]);

#[unsafe(link_section = ".data")]
static mut BOOT_PT_L2: Aligned4K<[LA64PTE; 512]> = Aligned4K::new([LA64PTE::empty(); 512]);

unsafe fn init_boot_page_table() {
    unsafe {
        let l1_va = va!(&raw const BOOT_PT_L1 as usize);
        // 0x0000_0000_0000 ~ 0x0080_0000_0000, table
        BOOT_PT_L0[0x100] = LA64PTE::new_table(ax_plat::mem::virt_to_phys(l1_va));
        let l2_va = va!(&raw const BOOT_PT_L2 as usize);
        // 0x0000_0000..0x4000_0000, table
        BOOT_PT_L1[0] = LA64PTE::new_table(ax_plat::mem::virt_to_phys(l2_va));
        for i in 0..512 {
            BOOT_PT_L2[i] = LA64PTE::new_page(
                pa!(i << 21),
                MappingFlags::READ
                    | MappingFlags::WRITE
                    | if i < 128 {
                        MappingFlags::EXECUTE
                    } else {
                        MappingFlags::DEVICE
                    },
                true,
            );
        }
        // 0x8000_0000..0xc000_0000, VPWXGD, 1G block
        BOOT_PT_L1[0x2] = LA64PTE::new_page(
            pa!(0x8000_0000),
            MappingFlags::READ | MappingFlags::WRITE | MappingFlags::EXECUTE,
            true,
        );
    }
}

fn enable_fp_simd() {
    // FP/SIMD needs to be enabled early, as the compiler may generate SIMD
    // instructions in the bootstrapping code to speed up the operations
    // like `memset` and `memcpy`.
    #[cfg(feature = "fp-simd")]
    {
        ax_cpu::asm::enable_fp();
        ax_cpu::asm::enable_lsx();
        ax_cpu::asm::enable_lasx();
    }
}

fn init_mmu() {
    ax_cpu::init::init_mmu(
        ax_plat::mem::virt_to_phys(va!(&raw const BOOT_PT_L0 as usize)),
        PHYS_BOOT_OFFSET,
    );
}

const BOOT_TO_VIRT: usize = PHYS_VIRT_OFFSET - PHYS_BOOT_OFFSET;

const EFI_SYSTEM_TABLE_SIGNATURE: u64 = 0x5453_5953_2049_4249;
const DEVICE_TREE_GUID: [u8; 16] = [
    0xd5, 0x21, 0xb6, 0xb1, 0x9c, 0xf1, 0xa5, 0x41, 0x83, 0x0b, 0xd9, 0x15, 0x2c, 0x69, 0xaa, 0xe0,
];

#[repr(C)]
struct EfiTableHeader {
    signature: u64,
    revision: u32,
    header_size: u32,
    crc32: u32,
    reserved: u32,
}

#[repr(C)]
struct EfiSystemTable {
    header: EfiTableHeader,
    firmware_vendor: u64,
    firmware_revision: u32,
    console_in_handle: u64,
    console_in: u64,
    console_out_handle: u64,
    console_out: u64,
    stderr_handle: u64,
    stderr: u64,
    runtime_services: u64,
    boot_services: u64,
    number_of_table_entries: u64,
    configuration_table: u64,
}

#[repr(C)]
struct EfiConfigurationTable {
    vendor_guid: [u8; 16],
    vendor_table: u64,
}

unsafe fn loongarch_qemu_fdt_from_systab(system_table_paddr: usize) -> usize {
    if system_table_paddr == 0 {
        return 0;
    }

    let system_table = (system_table_paddr + PHYS_BOOT_OFFSET) as *const EfiSystemTable;
    let system_table = unsafe { &*system_table };
    if system_table.header.signature != EFI_SYSTEM_TABLE_SIGNATURE {
        return 0;
    }

    let entries = system_table.number_of_table_entries as usize;
    let table_paddr = system_table.configuration_table as usize;
    if entries == 0 || table_paddr == 0 {
        return 0;
    }

    let table = (table_paddr + PHYS_BOOT_OFFSET) as *const EfiConfigurationTable;
    for i in 0..entries {
        let entry = unsafe { &*table.add(i) };
        if entry.vendor_guid == DEVICE_TREE_GUID {
            return entry.vendor_table as usize;
        }
    }

    0
}

/// The earliest entry point for the primary CPU.
///
/// We can't use bl to jump to higher address, so we use jirl to jump to higher address.
#[unsafe(naked)]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".text.boot")]
unsafe extern "C" fn __boot_start() -> ! {
    core::arch::naked_asm!("
        .globl  _linux_image_header
    _linux_image_header:
        .word   0x5a4d              # MZ, MS-DOS header
        .word   0                   # Reserved
        .dword  0x00200040          # Kernel entry point
        .dword  _ekernel - _skernel # Kernel image effective size
        .dword  0x00200000          # Kernel image load offset from start of RAM
        .dword  0                   # Reserved
        .dword  0                   # Reserved
        .dword  0                   # Reserved
        .word   0x818223cd          # Magic number
        .word   0x0                 # Offset to the PE header

        .globl  _start
    _start:
        # Setup DMW
        li.d        $t0, {phys_boot_offset} | 0x11
        csrwr       $t0, 0x180      # DMWIN0

        # Jump to DMW region
        la.local    $t0, 1f
        li.d        $t1, {phys_boot_offset}
        or          $t0, $t0, $t1
        jirl        $zero, $t0, 0

    1:
        move        $s0, $a2            # QEMU LoongArch direct boot passes EFI systab in a2.

        # Setup Stack
        la.local    $sp, {boot_stack}
        li.d        $t0, {boot_stack_size}
        add.d       $sp, $sp, $t0       # setup boot stack

        # Init MMU
        bl          {enable_fp_simd}    # enable FP/SIMD instructions
        bl          {init_boot_page_table}
        bl          {init_mmu}          # setup boot page table and enable MMU

        # Adjust stack pointer
        li.d        $t0, {boot_to_virt}
        add.d       $sp, $sp, $t0

        move        $a0, $s0
        bl          {loongarch_qemu_fdt_from_systab}
        move        $a1, $a0
        csrrd       $a0, 0x20           # cpuid
        la.abs      $t0, {entry}
        li.d        $ra, 0
        jirl        $zero, $t0, 0",

        phys_boot_offset = const PHYS_BOOT_OFFSET,
        boot_to_virt = const BOOT_TO_VIRT,

        boot_stack = sym BOOT_STACK,
        boot_stack_size = const BOOT_STACK_SIZE,
        enable_fp_simd = sym enable_fp_simd,
        init_boot_page_table = sym init_boot_page_table,
        init_mmu = sym init_mmu,
        loongarch_qemu_fdt_from_systab = sym loongarch_qemu_fdt_from_systab,
        entry = sym ax_plat::call_main,
    )
}

/// The earliest entry point for secondary CPUs.
#[cfg(feature = "smp")]
#[unsafe(naked)]
pub(crate) unsafe extern "C" fn _start_secondary() -> ! {
    core::arch::naked_asm!("
        li.w        $t0,  0x1028        # LA_IOCSR_MAIL_BUF1
        iocsrrd.d   $sp,  $t0           # Load stack pointer

        # Setup DMW
        li.d        $t0, {phys_boot_offset} | 0x11
        csrwr       $t0, 0x180          # DMWIN0
        # Already in DMW region

        # Init MMU
        bl          {enable_fp_simd}    # enable FP/SIMD instructions
        bl          {init_mmu}          # setup boot page table and enable MMU

        # Adjust stack pointer
        li.d        $t0, {boot_to_virt}
        add.d       $sp, $sp, $t0

        csrrd       $a0, 0x20           # cpuid
        la.abs      $t0, {entry}
        jirl        $zero, $t0, 0",

        phys_boot_offset = const PHYS_BOOT_OFFSET,
        boot_to_virt = const BOOT_TO_VIRT,

        enable_fp_simd = sym enable_fp_simd,
        init_mmu = sym init_mmu,
        entry = sym ax_plat::call_secondary_main,
    )
}
