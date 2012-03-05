/* linux/arch/arm/plat-s3c64xx/pm.c
 *
 * Copyright 2008 Openmoko, Inc.
 * Copyright 2008 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *	http://armlinux.simtec.co.uk/
 *
 * S3C64XX CPU PM support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/map.h>
#include <mach/irqs.h>

#include <plat/pm.h>
#include <plat/wakeup-mask.h>
#include <plat/regs-serial.h>

#include <mach/regs-sys.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/regs-syscon-power.h>
#include <mach/regs-gpio-memport.h>
#include <mach/regs-modem.h>

#ifdef CONFIG_S3C_PM_DEBUG_LED_SMDK
void s3c_pm_debug_smdkled(u32 set, u32 clear)
{
	unsigned long flags;
	int i;

	local_irq_save(flags);
	for (i = 0; i < 4; i++) {
		if (clear & (1 << i))
			gpio_set_value(S3C64XX_GPN(12 + i), 0);
		if (set & (1 << i))
			gpio_set_value(S3C64XX_GPN(12 + i), 1);
	}
	local_irq_restore(flags);
}
#endif

static struct sleep_save core_save[] = {
	SAVE_ITEM(S3C_APLL_LOCK),
	SAVE_ITEM(S3C_MPLL_LOCK),
	SAVE_ITEM(S3C_EPLL_LOCK),
	SAVE_ITEM(S3C_CLK_SRC),
	SAVE_ITEM(S3C_CLK_DIV0),
	SAVE_ITEM(S3C_CLK_DIV1),
	SAVE_ITEM(S3C_CLK_DIV2),
	SAVE_ITEM(S3C_CLK_OUT),
	SAVE_ITEM(S3C_HCLK_GATE),
	SAVE_ITEM(S3C_PCLK_GATE),
	SAVE_ITEM(S3C_SCLK_GATE),
	SAVE_ITEM(S3C_MEM0_GATE),

	SAVE_ITEM(S3C_EPLL_CON1),
	SAVE_ITEM(S3C_EPLL_CON0),

	SAVE_ITEM(S3C64XX_MEM0DRVCON),
	SAVE_ITEM(S3C64XX_MEM1DRVCON),

	SAVE_ITEM(S3C_APLL_CON),
	SAVE_ITEM(S3C_MPLL_CON),
};

static struct sleep_save misc_save[] = {
	SAVE_ITEM(S3C64XX_AHB_CON0),
	SAVE_ITEM(S3C64XX_AHB_CON1),
	SAVE_ITEM(S3C64XX_AHB_CON2),
	
	SAVE_ITEM(S3C64XX_SPCON),
	SAVE_ITEM(S3C64XX_SPCONSLP),

	SAVE_ITEM(S3C64XX_MEM0CONSTOP),
	SAVE_ITEM(S3C64XX_MEM1CONSTOP),
	SAVE_ITEM(S3C64XX_MEM0CONSLP0),
	SAVE_ITEM(S3C64XX_MEM0CONSLP1),
	SAVE_ITEM(S3C64XX_MEM1CONSLP),

	SAVE_ITEM(S3C64XX_SDMA_SEL),
	SAVE_ITEM(S3C64XX_MODEM_MIFPCON),
	SAVE_ITEM(S3C64XX_SLEEP_CFG),

	SAVE_ITEM(S3C64XX_QOS_OVERRIDE1),
	SAVE_ITEM(S3C64XX_MISC_CON),	
};

void s3c_pm_configure_extint(void)
{
	__raw_writel(s3c_irqwake_eintmask, S3C64XX_EINT_MASK);
}

static u32 s3c64xx_others_save;

void s3c_pm_restore_core(void)
{
	u32 pclkgate, tmp;
	int i;
	u32 reg, val;

	__raw_writel(0, S3C64XX_EINT_MASK);

	s3c_pm_debug_smdkled(1 << 2, 0);

	__raw_writel(s3c64xx_others_save, S3C64XX_OTHERS);

	if (s3c64xx_others_save & S3C64XX_OTHERS_SYNCMUXSEL)
		val = S3C64XX_OTHERS_SYNCACK_MASK;
	else
		val = 0;

	do {
		reg = __raw_readl(S3C64XX_OTHERS);
		reg &= S3C64XX_OTHERS_SYNCACK_MASK;
	} while (reg != val);

	s3c_pm_do_restore_core(core_save, ARRAY_SIZE(core_save));
	s3c_pm_do_restore(misc_save, ARRAY_SIZE(misc_save));

	tmp = pclkgate = __raw_readl(S3C_PCLK_GATE);

	/* re-start uart clocks */
	tmp |= S3C_CLKCON_PCLK_UART0;
	tmp |= S3C_CLKCON_PCLK_UART1;
	tmp |= S3C_CLKCON_PCLK_UART2;
	tmp |= S3C_CLKCON_PCLK_UART3;

	__raw_writel(tmp, S3C_PCLK_GATE);

	udelay(10);

	for (i = 0; i < 4; ++i) {
		__raw_writel(15, S3C_VA_UARTx(i) + S3C64XX_UINTM);
		__raw_writel(15, S3C_VA_UARTx(i) + S3C64XX_UINTP);
	}

	udelay(10);

	__raw_writel(pclkgate, S3C_PCLK_GATE);

	/* Setup PWRCFG to enter idle mode (for cpuidle) */
	tmp = __raw_readl(S3C64XX_PWR_CFG);
	tmp &= ~S3C64XX_PWRCFG_CFG_WFI_MASK;
	tmp |= S3C64XX_PWRCFG_CFG_WFI_IDLE;
	__raw_writel(tmp, S3C64XX_PWR_CFG);
}

void s3c_pm_save_core(void)
{
	u32 spcon;

	s3c_pm_do_save(misc_save, ARRAY_SIZE(misc_save));
	s3c_pm_do_save(core_save, ARRAY_SIZE(core_save));
	s3c64xx_others_save = __raw_readl(S3C64XX_OTHERS);

	spcon = __raw_readl(S3C64XX_SPCON);
	spcon &= ~0xffec0000;
	__raw_writel(spcon, S3C64XX_SPCON);
}

/* since both s3c6400 and s3c6410 share the same sleep pm calls, we
 * put the per-cpu code in here until any new cpu comes along and changes
 * this.
 */

static void s3c64xx_cpu_suspend(void)
{
	unsigned long tmp;

	/* set our standby method to sleep */

	tmp = __raw_readl(S3C64XX_PWR_CFG);
	tmp &= ~S3C64XX_PWRCFG_CFG_WFI_MASK;
	tmp |= S3C64XX_PWRCFG_CFG_WFI_SLEEP;
	__raw_writel(tmp, S3C64XX_PWR_CFG);

	/* clear any old wakeup */

	__raw_writel(__raw_readl(S3C64XX_WAKEUP_STAT),
		     S3C64XX_WAKEUP_STAT);

	/* set the LED state to 0110 over sleep */
	s3c_pm_debug_smdkled(3 << 1, 0xf);

	/* issue the standby signal into the pm unit. Note, we
	 * issue a write-buffer drain just in case */

	tmp = 0;

	asm("b 1f\n\t"
	    ".align 5\n\t"
	    "1:\n\t"
	    "mcr p15, 0, %0, c7, c10, 5\n\t"
	    "mcr p15, 0, %0, c7, c10, 4\n\t"
	    "mcr p15, 0, %0, c7, c0, 4" :: "r" (tmp));

	/* we should never get past here */

	panic("sleep resumed to originator?");
}

/* mapping of interrupts to parts of the wakeup mask */
static struct samsung_wakeup_mask wake_irqs[] = {
	{ .irq = IRQ_RTC_ALARM,	.bit = S3C64XX_PWRCFG_RTC_ALARM_DISABLE, },
	{ .irq = IRQ_RTC_TIC,	.bit = S3C64XX_PWRCFG_RTC_TICK_DISABLE, },
	{ .irq = IRQ_PENDN,	.bit = S3C64XX_PWRCFG_TS_DISABLE, },
	{ .irq = IRQ_HSMMC0,	.bit = S3C64XX_PWRCFG_MMC0_DISABLE, },
	{ .irq = IRQ_HSMMC1,	.bit = S3C64XX_PWRCFG_MMC1_DISABLE, },
	{ .irq = IRQ_HSMMC2,	.bit = S3C64XX_PWRCFG_MMC2_DISABLE, },
	{ .irq = IRQ_KEYPAD,	.bit = S3C64XX_PWRCFG_KEY_DISABLE, },
	{ .irq = NO_WAKEUP_IRQ,	.bit = S3C64XX_PWRCFG_BATF_DISABLE},
	{ .irq = NO_WAKEUP_IRQ,	.bit = S3C64XX_PWRCFG_MSM_DISABLE },
	{ .irq = NO_WAKEUP_IRQ,	.bit = S3C64XX_PWRCFG_HSI_DISABLE },
	{ .irq = NO_WAKEUP_IRQ,	.bit = S3C64XX_PWRCFG_MSM_DISABLE },
};

static void s3c64xx_pm_prepare(void)
{
	samsung_sync_wakemask(S3C64XX_PWR_CFG,
			      wake_irqs, ARRAY_SIZE(wake_irqs));

	/* store address of resume. */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S3C64XX_INFORM0);

	/* ensure previous wakeup state is cleared before sleeping */
	__raw_writel(__raw_readl(S3C64XX_WAKEUP_STAT), S3C64XX_WAKEUP_STAT);
}

static int s3c64xx_pm_init(void)
{
	pm_cpu_prep = s3c64xx_pm_prepare;
	pm_cpu_sleep = s3c64xx_cpu_suspend;
	pm_uart_udivslot = 1;

#ifdef CONFIG_S3C_PM_DEBUG_LED_SMDK
	gpio_request(S3C64XX_GPN(12), "DEBUG_LED0");
	gpio_request(S3C64XX_GPN(13), "DEBUG_LED1");
	gpio_request(S3C64XX_GPN(14), "DEBUG_LED2");
	gpio_request(S3C64XX_GPN(15), "DEBUG_LED3");
	gpio_direction_output(S3C64XX_GPN(12), 0);
	gpio_direction_output(S3C64XX_GPN(13), 0);
	gpio_direction_output(S3C64XX_GPN(14), 0);
	gpio_direction_output(S3C64XX_GPN(15), 0);
#endif

	return 0;
}

arch_initcall(s3c64xx_pm_init);
