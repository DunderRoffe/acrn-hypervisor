/*
 * Copyright (c) 2011 NetApp, Inc.
 * Copyright (c) 2018 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY NETAPP, INC ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NETAPP, INC OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <hypervisor.h>
#include "pci_priv.h"

static uint32_t num_pci_vdev;
static struct pci_vdev sharing_mode_vdev_array[CONFIG_MAX_PCI_DEV_NUM];

struct pci_vdev *sharing_mode_find_vdev(union pci_bdf pbdf)
{
	struct pci_vdev *vdev = NULL;
	uint32_t i;

        pr_fatal("num_pci_vdev: %i", num_pci_vdev);
        pr_fatal("BDF 0x%04x", pbdf);
	/* in SOS_VM, it uses phys BDF */
	for (i = 0U; i < num_pci_vdev; i++) {
                pr_fatal("\nChecking vpci index %i", i);
                if (sharing_mode_vdev_array[i].pdev != NULL ) {
	                pr_fatal("PDEV in question: 0x%x", sharing_mode_vdev_array[i].pdev);
	                pr_fatal("PBDF on index: 0x%04x", sharing_mode_vdev_array[i].pbdf);
	                pr_fatal("VBDF on index: 0x%04x", sharing_mode_vdev_array[i].vbdf);

			if (sharing_mode_vdev_array[i].pdev->bdf.value == pbdf.value) {
				vdev = &sharing_mode_vdev_array[i];
				pr_fatal("Found device matching device: 0x%x", vdev);
			} else {
				pr_fatal("No match on this index");
                        }
		} else {
			pr_fatal("PDEV in vdev array at index %i is NULL", i);
                }
	}
        pr_fatal("Done searching\n");

	return vdev;
}

static void sharing_mode_cfgread(__unused struct acrn_vpci *vpci, union pci_bdf bdf,
	uint32_t offset, uint32_t bytes, uint32_t *val)
{
	struct pci_vdev *vdev;
	bool handled = false;
	uint32_t i;

        pr_fatal("Attempt find vdev from bdf");
	vdev = sharing_mode_find_vdev(bdf);

	/* vdev == NULL: Could be hit for PCI enumeration from guests */
	if ((vdev == NULL) || ((bytes != 1U) && (bytes != 2U) && (bytes != 4U))) {
                pr_fatal("VDEV == NUll or bit 1 2 and 3 are not all set in bytes");
		*val = ~0U;
	} else {
                pr_fatal("Found VDEV, proceeding with reading ops...");
		for (i = 0U; (i < vdev->nr_ops) && !handled; i++) {
			if (vdev->ops[i].cfgread != NULL) {
				if (vdev->ops[i].cfgread(vdev, offset, bytes, val) == 0) {
					handled = true;
				}
			}
		}

		/* Not handled by any handlers. Passthru to physical device */
		if (!handled) {
			*val = pci_pdev_read_cfg(vdev->pdev->bdf, offset, bytes);
		}
	}
}

static void sharing_mode_cfgwrite(__unused struct acrn_vpci *vpci, union pci_bdf bdf,
	uint32_t offset, uint32_t bytes, uint32_t val)
{
	struct pci_vdev *vdev;
	bool handled = false;
	uint32_t i;

	if ((bytes == 1U) || (bytes == 2U) || (bytes == 4U)) {
		vdev = sharing_mode_find_vdev(bdf);
		if (vdev != NULL) {
			for (i = 0U; (i < vdev->nr_ops) && !handled; i++) {
				if (vdev->ops[i].cfgwrite != NULL) {
					if (vdev->ops[i].cfgwrite(vdev, offset, bytes, val) == 0) {
						handled = true;
					}
				}
			}

			/* Not handled by any handlers. Passthru to physical device */
			if (!handled) {
				pci_pdev_write_cfg(vdev->pdev->bdf, offset, bytes, val);
			}
		}
	}
}

static struct pci_vdev *alloc_pci_vdev(const struct acrn_vm *vm, struct pci_pdev *pdev_ref)
{
	struct pci_vdev *vdev = NULL;

	if (num_pci_vdev < CONFIG_MAX_PCI_DEV_NUM) {
	        pr_fatal("\nAllocating vdev %i, vm: 0x%x pdev_ref: 0x%x", num_pci_vdev, vm, pdev_ref);
		vdev = &sharing_mode_vdev_array[num_pci_vdev];
                pr_fatal("vdev ptr %x", vdev);
		num_pci_vdev++;

		if ((vm != NULL) && (vdev != NULL) && (pdev_ref != NULL)) {
			vdev->vpci = &vm->vpci;
			/* vbdf equals to pbdf otherwise remapped */
			vdev->vbdf = pdev_ref->bdf;
			vdev->pdev = pdev_ref;
                        pr_fatal("Allocation DONE! vpci: 0x%x    bdf: 0x%04x", vdev->vpci, vdev->vbdf);
		} else {
			pr_fatal("FAILED ALLOC at index %i", num_pci_vdev - 1);
                }
	}

	return vdev;
}

static void init_vdev_for_pdev(struct pci_pdev *pdev, const void *cb_data)
{
	const struct acrn_vm *vm = (const struct acrn_vm *)cb_data;
	struct pci_vdev *vdev;

	vdev = alloc_pci_vdev(vm, pdev);
	if (vdev != NULL) {
		populate_msi_struct(vdev);
	}
	pr_fatal("After call to populate_msi_struct    vpci: 0x%x    bdf: 0x%04x", vdev->vpci, vdev->vbdf);
}

static int32_t sharing_mode_vpci_init(const struct acrn_vm *vm)
{
	struct pci_vdev *vdev;
	uint32_t i, j;
	int32_t ret;

	/*
	 * Only setup IO bitmap for SOS.
	 * IO/MMIO requests from non-sos_vm guests will be injected to device model.
	 */
	if (!is_sos_vm(vm)) {
		ret = -ENODEV;
	} else {
		/* Build up vdev array for sos_vm */
		pci_pdev_foreach(init_vdev_for_pdev, vm);

		for (i = 0U; i < num_pci_vdev; i++) {
			vdev = &sharing_mode_vdev_array[i];
			pr_fatal("\nInit ops for vdev idx %d", i);
			pr_fatal("Before ops init:  vpci: 0x%x    bdf: 0x%04x    pdev: 0x%x", vdev->vpci, vdev->vbdf, vdev->pdev);
			for (j = 0U; j < vdev->nr_ops; j++) {
				if (vdev->ops[j].init != NULL) {
					(void)vdev->ops[j].init(vdev);
				}
			}
			pr_fatal("After ops init:  vpci: 0x%x    bdf: 0x%04x    pdev: 0x%x", vdev->vpci, vdev->vbdf, vdev->pdev);
		}
		ret = 0;
	}

	return ret;
}

static void sharing_mode_vpci_deinit(__unused const struct acrn_vm *vm)
{
	struct pci_vdev *vdev;
	uint32_t i, j;

	pr_fatal("sharing_mode_vpci_deinit, vm: 0x%x", vm);
	if (is_sos_vm(vm)) {
		for (i = 0U; i < num_pci_vdev; i++) {
			vdev = &sharing_mode_vdev_array[i];
			for (j = 0U; j < vdev->nr_ops; j++) {
				if (vdev->ops[j].deinit != NULL) {
					(void)vdev->ops[j].deinit(vdev);
				}
			}
		}
	}
}

void add_vdev_handler(struct pci_vdev *vdev, const struct pci_vdev_ops *ops)
{
	if (vdev->nr_ops >= (MAX_VPCI_DEV_OPS - 1U)) {
		pr_err("%s, adding too many handlers", __func__);
	} else {
		vdev->ops[vdev->nr_ops] = *ops;
		vdev->nr_ops++;
	}
}

const struct vpci_ops sharing_mode_vpci_ops = {
	.init = sharing_mode_vpci_init,
	.deinit = sharing_mode_vpci_deinit,
	.cfgread = sharing_mode_cfgread,
	.cfgwrite = sharing_mode_cfgwrite,
};

void vpci_set_ptdev_intr_info(const struct acrn_vm *target_vm, uint16_t vbdf, uint16_t pbdf)
{
	struct pci_vdev *vdev;

	vdev = sharing_mode_find_vdev((union pci_bdf)pbdf);
	if (vdev == NULL) {
		pr_err("%s, can't find PCI device for vm%d, vbdf (0x%x) pbdf (0x%x)", __func__,
			target_vm->vm_id, vbdf, pbdf);
	} else {
		/* UOS may do BDF mapping */
		vdev->vpci = (struct acrn_vpci *)&(target_vm->vpci);
		vdev->vbdf.value = vbdf;
		vdev->pdev->bdf.value = pbdf;
	}
}

void vpci_reset_ptdev_intr_info(const struct acrn_vm *target_vm, uint16_t vbdf, uint16_t pbdf)
{
	struct pci_vdev *vdev;
	struct acrn_vm *vm;

	vdev = sharing_mode_find_vdev((union pci_bdf)pbdf);
	if (vdev == NULL) {
		pr_err("%s, can't find PCI device for vm%d, vbdf (0x%x) pbdf (0x%x)", __func__,
			target_vm->vm_id, vbdf, pbdf);
	} else {
		/* Return this PCI device to SOS */
		if (vdev->vpci->vm == target_vm) {
			vm = get_sos_vm();

			if (vm != NULL) {
				vdev->vpci = &vm->vpci;
			}
		}
	}
}
