/*
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_math.h>
#include <sbi/sbi_trap.h>
#include <sbi/sbi_unpriv.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/fdt/fdt_fixup.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/serial/uart8250.h>
#include <sbi_utils/sys/clint.h>
#include "platform.h"

#include <generated/autoconf.h>

#define UART_CLK                    50000000
#define UART_DEFAULT_BAUDRATE       (CONFIG_RTT_CONSOLE_BAUD)
#define UART_ADDR 					(CONFIG_OPENSBI_CONSOLE_UART_REG_ADDR)

#define K230_ATAG_NONE              0x00000000
#define K230_ATAG_BASE_ADDR         (CONFIG_RTSMART_OPENSIB_MEMORY_SIZE - 0x4000UL)
#define K230_ATAG_LIMIT_ADDR        (CONFIG_RTSMART_OPENSIB_MEMORY_SIZE - 0x10UL)

struct k230_atag_header {
	u32 size;
	u32 tag;
};

static size_t plic_base_addr;
static size_t clint_base_addr;

static struct platform_uart_data uart = {
	UART_ADDR,
	UART_CLK,
	UART_DEFAULT_BAUDRATE,
};

static struct plic_data plic = {
	.addr = 0xF00000000,
	.num_src = 200,
};

static struct clint_data clint = {
	.addr = 0xF04000000, /* Updated at cold boot time */
	.first_hartid = 0,
	.hart_count = C908_HART_COUNT,
	.has_64bit_mmio = FALSE,
};

struct c908_pmp_memregion {
	unsigned long addr;
	unsigned long size;
	unsigned long flags;
};

static const struct c908_pmp_memregion c908_root_regions[] = {
	{
		/* PUFS RT CDE */
		.addr = 0x91214000,
		.size = 0x1000,
		.flags = SBI_DOMAIN_MEMREGION_READABLE,
	},
	{
		/* PUFS RT */
		.addr = 0x91213000,
		.size = 0x1000,
		.flags = SBI_DOMAIN_MEMREGION_READABLE,
	},
	{
		/* SRAM 2MiB */
		.addr = 0x80400000,
		.size = 0x200000,
		.flags = SBI_DOMAIN_MEMREGION_READABLE |
			 SBI_DOMAIN_MEMREGION_WRITEABLE,
	},
	{
		/* SRAM 2MiB */
		.addr = 0x80000000,
		.size = 0x200000,
		.flags = SBI_DOMAIN_MEMREGION_READABLE |
			 SBI_DOMAIN_MEMREGION_WRITEABLE, // | SBI_DOMAIN_MEMREGION_EXECUTABLE,
	},
	{
		/* CLINT */
		.addr = 0xf04000000,
		.size = 0x10000,
		.flags = SBI_DOMAIN_MEMREGION_READABLE |
			 SBI_DOMAIN_MEMREGION_WRITEABLE,
	},
};

static int c908_domains_init(void)
{
	struct sbi_domain_memregion reg;
	unsigned int i;
	int rc;

	for (i = 0; i < sizeof(c908_root_regions) / sizeof(c908_root_regions[0]); i++) {
		sbi_domain_memregion_init(c908_root_regions[i].addr,
					 c908_root_regions[i].size,
					 c908_root_regions[i].flags,
					 &reg);
		rc = sbi_domain_root_add_memregion(&reg);
		if (rc) {
			sbi_printf("Failed to add memory region: %d, addr: 0x%lx\n", rc, c908_root_regions[i].addr);
			return rc;
		}
	}

	return 0;
}

static int c908_pmp_normalize_region(unsigned long addr, unsigned long size,
				     unsigned long *base,
				     unsigned long *log2size)
{
	unsigned long order;
	unsigned long region_end;
	unsigned long region_size;

	if (!size || !base || !log2size)
		return SBI_EINVAL;

	order = (size <= (1UL << PMP_SHIFT)) ? PMP_SHIFT : log2roundup(size);
	if (order > __riscv_xlen)
		return SBI_EINVAL;

	if (order == __riscv_xlen) {
		*base = 0;
		*log2size = order;
		return 0;
	}

	region_size = (1UL << order);
	*base = addr & ~(region_size - 1UL);
	region_end = addr + size - 1UL;
	if (region_end < addr || region_end >= (*base + region_size))
		return SBI_EINVAL;

	*log2size = order;
	return 0;
}

static void c908_store_ulong(unsigned long *addr, unsigned long value,
			     struct sbi_trap_info *trap)
{
#if __riscv_xlen == 64
	sbi_store_u64((u64 *)addr, value, trap);
#else
	sbi_store_u32((u32 *)addr, value, trap);
#endif
}

static void c908_store_u8_array(u8 *dst, const u8 *src, unsigned long len,
			struct sbi_trap_info *trap)
{
	unsigned long i;

	for (i = 0; i < len; i++) {
		sbi_store_u8(dst + i, src[i], trap);
		if (trap->cause)
			return;
	}
}

static int c908_atag_total_size(unsigned long *size)
{
	const struct k230_atag_header *hdr;
	unsigned long cursor = K230_ATAG_BASE_ADDR;
	unsigned long step;

	if (!size)
		return SBI_EINVAL;

	while ((cursor + sizeof(*hdr)) <= K230_ATAG_LIMIT_ADDR) {
		hdr = (const struct k230_atag_header *)cursor;
		if (hdr->tag == K230_ATAG_NONE) {
			*size = (cursor - K230_ATAG_BASE_ADDR) + sizeof(*hdr);
			return 0;
		}

		step = ((unsigned long)hdr->size) * sizeof(u32);
		if (step < sizeof(*hdr) || (cursor + step) > K230_ATAG_LIMIT_ADDR)
			return SBI_EFAIL;

		cursor += step;
	}

	return SBI_EFAIL;
}

static int c908_atag_copy(void *dst, unsigned long dst_size,
			  unsigned long *copied_size,
			  struct sbi_trap_info *out_trap)
{
	unsigned long size;
	int rc;

	if (!dst || !dst_size || !copied_size)
		return SBI_EINVAL;

	rc = c908_atag_total_size(&size);
	if (rc)
		return rc;
	if (dst_size < size)
		return SBI_EINVAL;

	c908_store_u8_array((u8 *)dst, (const u8 *)K230_ATAG_BASE_ADDR,
			   size, out_trap);
	if (out_trap->cause)
		return SBI_ETRAP;

	*copied_size = size;
	return 0;
}

static int c908_pmp_find_region(unsigned long base, unsigned long log2size,
			unsigned int *match_index,
			unsigned long *match_prot,
			unsigned int *free_index)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	unsigned int pmp_count = sbi_hart_pmp_count(scratch);
	unsigned long prot;
	unsigned long entry_addr;
	unsigned long entry_log2size;
	int rc;
	int idx;

	if (!match_index || !match_prot || !free_index)
		return SBI_EINVAL;

	*match_index = pmp_count;
	*match_prot = 0;
	*free_index = pmp_count;

	for (idx = 0; idx < (int)pmp_count; idx++) {
		rc = pmp_get(idx, &prot, &entry_addr, &entry_log2size);
		if (rc)
			return rc;
		if ((prot & PMP_A) == 0)
		{
			if (*free_index == pmp_count)
				*free_index = idx;
			continue;
		}
		if (entry_addr == base && entry_log2size == log2size) {
			*match_index = idx;
			*match_prot = prot;
			break;
		}
	}

	return 0;
}

static int c908_pmp_set_region(unsigned long addr, unsigned long size,
			       unsigned long perm,
			       unsigned long *resolved_index)
{
	unsigned long base;
	unsigned long log2size;
	unsigned long match_prot;
	unsigned int match_index;
	unsigned int free_index;
	int rc;

	if (perm & ~(PMP_R | PMP_W | PMP_X))
		return SBI_EINVAL;

	rc = c908_pmp_normalize_region(addr, size, &base, &log2size);
	if (rc)
		return rc;

	rc = c908_pmp_find_region(base, log2size, &match_index,
				  &match_prot, &free_index);
	if (rc)
		return rc;

	if (match_index != sbi_hart_pmp_count(sbi_scratch_thishart_ptr())) {
		if (match_prot & PMP_L)
			return SBI_EDENIED;
		rc = pmp_set(match_index, perm, base, log2size);
		if (rc)
			return rc;
		if (resolved_index)
			*resolved_index = match_index;
		return 0;
	}

	if (free_index == sbi_hart_pmp_count(sbi_scratch_thishart_ptr()))
		return SBI_ENOSPC;

	rc = pmp_set(free_index, perm, base, log2size);
	if (rc)
		return rc;
	if (resolved_index)
		*resolved_index = free_index;

	return 0;
}

static int c908_pmp_get_region(unsigned long addr, unsigned long size,
			       unsigned long *perm,
			       unsigned long *index)
{
	unsigned long base;
	unsigned long log2size;
	unsigned long match_prot;
	unsigned int match_index;
	unsigned int free_index;
	int rc;

	rc = c908_pmp_normalize_region(addr, size, &base, &log2size);
	if (rc)
		return rc;

	rc = c908_pmp_find_region(base, log2size, &match_index,
				  &match_prot, &free_index);
	if (rc)
		return rc;

	if (match_index == sbi_hart_pmp_count(sbi_scratch_thishart_ptr()))
		return SBI_ENOENT;

	if (perm)
		*perm = match_prot & (PMP_R | PMP_W | PMP_X);
	if (index)
		*index = match_index;

	return 0;
}

static int c908_vendor_ext_check(long extid)
{
	return (extid == SBI_EXT_KENDRYTE_PMP) ? 1 : 0;
}

static int c908_vendor_ext_provider(long extid, long funcid,
			    const struct sbi_trap_regs *regs,
			    unsigned long *out_value,
			    struct sbi_trap_info *out_trap)
{
	unsigned long perm;
	unsigned long index;
	int rc;

	if (extid != SBI_EXT_KENDRYTE_PMP)
		return SBI_ENOTSUPP;

	switch (funcid) {
	case SBI_EXT_KENDRYTE_PMP_SET:
		rc = c908_pmp_set_region(regs->a0, regs->a1, regs->a2, &index);
		if (rc)
			return rc;
		*out_value = index;
		return 0;
	case SBI_EXT_KENDRYTE_PMP_GET:
		if (!regs->a2 || !regs->a3)
			return SBI_EINVAL;
		rc = c908_pmp_get_region(regs->a0, regs->a1, &perm, &index);
		if (rc)
			return rc;
		c908_store_ulong((unsigned long *)regs->a2, perm, out_trap);
		if (out_trap->cause)
			return SBI_ETRAP;
		c908_store_ulong((unsigned long *)regs->a3, index, out_trap);
		if (out_trap->cause)
			return SBI_ETRAP;
		*out_value = index;
		return 0;
	case SBI_EXT_KENDRYTE_ATAG_COPY:
		rc = c908_atag_copy((void *)regs->a0, regs->a1, out_value,
				   out_trap);
		return rc;
	default:
		return SBI_ENOTSUPP;
	}
}

static int c908_early_init(bool cold_boot)
{
	void *fdt;
	struct platform_uart_data uart_data;
	struct plic_data plic_data;
	unsigned long clint_addr;
	int rc;

	if (!cold_boot)
		return 0;

	plic_base_addr = csr_read(CSR_PLIC_BASE);
	clint_base_addr = plic_base_addr + C908_PLIC_CLINT_OFFSET;

	fdt = sbi_scratch_thishart_arg1_ptr();

	rc = fdt_parse_uart8250(fdt, &uart_data, "snps,dw-apb-uart");
	if (!rc)
		uart = uart_data;

	rc = fdt_parse_plic(fdt, &plic_data, "riscv,plic0");
	if (!rc)
		plic = plic_data;

	rc = fdt_parse_compat_addr(fdt, &clint_addr, "riscv,clint0");
	if (!rc)
		clint.addr = clint_addr;

	return 0;
}

static int c908_final_init(bool cold_boot)
{
	// void *fdt;
	unsigned long exceptions;

	if (!cold_boot)
		return 0;

	// fdt = sbi_scratch_thishart_arg1_ptr();
	// fdt_fixups(fdt);

	/* Delegate 0 ~ 7 exceptions to S-mode */
	exceptions = csr_read(CSR_MEDELEG);
	exceptions |= ((1U << CAUSE_MISALIGNED_FETCH) | (1U << CAUSE_FETCH_ACCESS) |
		(1U << CAUSE_ILLEGAL_INSTRUCTION) | (1U << CAUSE_BREAKPOINT) |
		(1U << CAUSE_MISALIGNED_LOAD) | (1U << CAUSE_LOAD_ACCESS) |
		(1U << CAUSE_MISALIGNED_STORE) | (1U << CAUSE_STORE_ACCESS));
	csr_write(CSR_MEDELEG, exceptions);

	csr_write(CSR_MIDELEG, 0x222);
	csr_write(CSR_MEDELEG, 0xb1ff);
	csr_write(CSR_MHCR, 0x11ff);
	csr_write(CSR_MCOR, 0x70013);
	csr_write(CSR_MCCR2, 0xe0410009);
	csr_write(CSR_MHINT, 0x16e30c);
	csr_write(0x7f3, 0x1);

	csr_write(0x7d9, csr_read(0x7d9) | 1);

	// enable vs externsion
	csr_write(CSR_MSTATUS, (csr_read(CSR_MSTATUS) & (~(3 << 9))) | (1 << 9));

#ifndef OPENSBI_QUIET
	sbi_printf("pmpcfg0: %lx\r\n", csr_read(CSR_PMPCFG0));
	sbi_printf("pmpaddr0: %lx\r\n", csr_read(CSR_PMPADDR0));
	sbi_printf("pmpaddr1: %lx\r\n", csr_read(CSR_PMPADDR1));
	sbi_printf("pmpaddr2: %lx\r\n", csr_read(CSR_PMPADDR2));
	sbi_printf("pmpaddr3: %lx\r\n", csr_read(CSR_PMPADDR3));
	sbi_printf("pmpaddr4: %lx\r\n", csr_read(CSR_PMPADDR4));
	sbi_printf("pmpaddr5: %lx\r\n", csr_read(CSR_PMPADDR5));
#endif
	return 0;
}

static int c908_irqchip_init(bool cold_boot)
{
	int ret;

	if (cold_boot) {
		/* Delegate plic enable into S-mode */
		writel(C908_PLIC_DELEG_ENABLE, (void *)(plic_base_addr + C908_PLIC_DELEG_OFFSET));
		ret = plic_cold_irqchip_init(&plic);
		if (ret)
			return ret;
	}

	return 0;
}

static int c908_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_ipi_init(&clint);
		if (rc)
			return rc;
	}

	return clint_warm_ipi_init();
}

static int c908_timer_init(bool cold_boot)
{
	int ret;

	if (cold_boot) {
		ret = clint_cold_timer_init(&clint, NULL);
		if (ret)
			return ret;
	}

	return clint_warm_timer_init();
}

int c908_hart_start(u32 hartid, ulong saddr)
{
	csr_write(CSR_MRVBR, saddr);
	csr_write(CSR_MRMR, csr_read(CSR_MRMR) | (1 << hartid));

	return 0;
}

static int c908_console_init(void)
{
	return uart8250_init(UART_ADDR,
						UART_CLK,
						UART_DEFAULT_BAUDRATE,
						2,
						4);
}

const struct sbi_platform_operations platform_ops = {
	.early_init          = c908_early_init,
	.final_init          = c908_final_init,
	.domains_init        = c908_domains_init,
	.irqchip_init        = c908_irqchip_init,
	.console_init        = c908_console_init,
	.ipi_init            = c908_ipi_init,
	.timer_init          = c908_timer_init,
	.vendor_ext_check    = c908_vendor_ext_check,
	.vendor_ext_provider = c908_vendor_ext_provider,
};

const struct sbi_platform platform = {
	.opensbi_version     = OPENSBI_VERSION,
	.platform_version    = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name                = "T-HEAD Xuantie c908",
	.features            = SBI_THEAD_FEATURES,
	.hart_count          = C908_HART_COUNT,
	.hart_stack_size     = SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr   = (unsigned long)&platform_ops
};
