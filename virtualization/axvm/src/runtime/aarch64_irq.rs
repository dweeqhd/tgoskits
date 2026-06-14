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

//! AArch64 routed interrupt delivery.

use crate::runtime::VCpuRef;

pub fn drain_routed_irqs(vm: &crate::AxVM, vcpu: &VCpuRef) {
    if let Err(err) = vm
        .interrupt_fabric()
        .drain_pending(vcpu.id(), |irq| vcpu.inject_interrupt(irq.vector))
    {
        if err == ax_errno::AxError::WouldBlock {
            trace!(
                "AArch64 list registers are full for VM[{}] VCpu[{}]; routed IRQ remains pending",
                vm.id(),
                vcpu.id()
            );
        } else {
            warn!(
                "failed to drain routed AArch64 interrupts for VM[{}] VCpu[{}]: {err:?}",
                vm.id(),
                vcpu.id()
            );
        }
    }
}
