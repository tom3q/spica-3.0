/* linux/arch/arm/mach-s3c64xx/time.c
 *
 * Copyright (c) 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * based on linux/arch/arm/plat-samsung/s5p-time.c
 *
 * S3C64XX generic high resolution time support using PWM 3 and PWM 4 timers.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Notes:
 * - Frequency divisors are currently hardcoded to make both timers operate
 *   at 1/6 the frequency of PCLK, so with the usual PCLK frequency of 66 MHz,
 *   the timers are clocked at 11 MHz giving us the operating range from
 *   90 nsec to 386 sec.
 * - PWM registers, especially TCON, are shared with PWM driver, but since
 *   all the clock event callbacks are run in atomic context synchronization
 *   is not needed.
 */

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/platform_device.h>

#include <asm/sched_clock.h>
#include <mach/map.h>
#include <plat/regs-timer.h>
#include <asm/mach/time.h>

#define PWM_EVENT	(3)
#define PWM_SOURCE	(4)

#define ONESHOT		(0)
#define PERIODIC	(1)

#define TCNT_MAX	(0xffffffff)

static struct clk *timerclk;

/*
 * PWM timers setup code
 */

static inline void s3c64xx_pwm_stop(unsigned int pwm_id)
{
	unsigned long tcon = __raw_readl(S3C2410_TCON);

	switch (pwm_id) {
	case 3:
		tcon &= ~S3C2410_TCON_T3START;
		break;
	case 4:
		tcon &= ~S3C2410_TCON_T4START;
		break;
	}

	__raw_writel(tcon, S3C2410_TCON);
}

static inline void s3c64xx_pwm_init(unsigned int pwm_id, unsigned long tcnt)
{
	unsigned long tcon = __raw_readl(S3C2410_TCON);

	/* timers reload after counting zero, so reduce the count by 1 */
	--tcnt;

	/* stop the timer and ask it to load the new value */
	switch (pwm_id) {
	case 3:
		tcon &= ~(0xf<<16);
		tcon |= S3C2410_TCON_T3MANUALUPD;
		break;
	case 4:
		tcon &= ~(7<<20);
		tcon |= S3C2410_TCON_T4MANUALUPD;
		break;
	}

	__raw_writel(tcnt, S3C2410_TCNTB(pwm_id));
	__raw_writel(tcnt, S3C2410_TCMPB(pwm_id));
	__raw_writel(tcon, S3C2410_TCON);
}

static inline void s3c64xx_pwm_start(unsigned int pwm_id, bool periodic)
{
	unsigned long tcon = __raw_readl(S3C2410_TCON);

	switch (pwm_id) {
	case 3:
		tcon |= S3C2410_TCON_T3START;
		tcon &= ~S3C2410_TCON_T3MANUALUPD;

		if (periodic)
			tcon |= S3C2410_TCON_T3RELOAD;
		else
			tcon &= ~S3C2410_TCON_T3RELOAD;
		break;
	case 4:
		tcon |= S3C2410_TCON_T4START;
		tcon &= ~S3C2410_TCON_T4MANUALUPD;

		if (periodic)
			tcon |= S3C2410_TCON_T4RELOAD;
		else
			tcon &= ~S3C2410_TCON_T4RELOAD;
		break;
	}

	__raw_writel(tcon, S3C2410_TCON);
}

static inline void s3c64xx_pwm_reconfigure(unsigned int pwm_id,
					unsigned long tcnt, bool periodic)
{
	unsigned long tcon = __raw_readl(S3C2410_TCON);

	switch (pwm_id) {
	case 3:
		tcon |= S3C2410_TCON_T3START;
		tcon &= ~S3C2410_TCON_T3MANUALUPD;

		if (periodic)
			tcon |= S3C2410_TCON_T3RELOAD;
		else
			tcon &= ~S3C2410_TCON_T3RELOAD;
		break;
	case 4:
		tcon |= S3C2410_TCON_T4START;
		tcon &= ~S3C2410_TCON_T4MANUALUPD;

		if (periodic)
			tcon |= S3C2410_TCON_T4RELOAD;
		else
			tcon &= ~S3C2410_TCON_T4RELOAD;
		break;
	}

	__raw_writel(tcnt, S3C2410_TCNTB(pwm_id));
	__raw_writel(tcnt, S3C2410_TCMPB(pwm_id));
	__raw_writel(tcon, S3C2410_TCON);
}

/*
 * Clock event
 */

static struct clk *event_in;
static struct clk *event_div;

static unsigned long clock_count_per_tick;

static int s3c64xx_clock_event_set_next_event(unsigned long cycles,
						struct clock_event_device *evt)
{
	s3c64xx_pwm_init(PWM_EVENT, cycles);
	s3c64xx_pwm_start(PWM_EVENT, ONESHOT);
	return 0;
}

static void s3c64xx_clock_event_resume(void)
{
	unsigned long pclk;
	struct clk *tscaler;

	pclk = clk_get_rate(timerclk);
	tscaler = clk_get_parent(event_div);
	clk_set_rate(tscaler, pclk / 3);

	clk_set_rate(event_div, pclk / 6);
	clk_set_parent(event_in, event_div);

	s3c64xx_pwm_init(PWM_EVENT, clock_count_per_tick);
	s3c64xx_pwm_start(PWM_EVENT, PERIODIC);
}

static void s3c64xx_clock_event_set_mode(enum clock_event_mode mode,
						struct clock_event_device *evt)
{
	s3c64xx_pwm_stop(PWM_EVENT);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		s3c64xx_pwm_init(PWM_EVENT, clock_count_per_tick);
		s3c64xx_pwm_start(PWM_EVENT, PERIODIC);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	case CLOCK_EVT_MODE_RESUME:
		s3c64xx_clock_event_resume();
		break;
	}
}

static struct clock_event_device s3c64xx_clock_event_device = {
	.name		= "s3c64xx_clkevt",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 200,
	.shift		= 32,
	.set_next_event	= s3c64xx_clock_event_set_next_event,
	.set_mode	= s3c64xx_clock_event_set_mode,
};

static irqreturn_t s3c64xx_clock_event_isr(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction s3c64xx_clock_event_irq = {
	.name		= "s3c64xx_clkevt_irq",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= s3c64xx_clock_event_isr,
	.dev_id		= &s3c64xx_clock_event_device,
};

static void __init s3c64xx_clock_event_init(void)
{
	unsigned long clock_rate;

	clock_rate = clk_get_rate(event_in);

	clock_count_per_tick = clock_rate / HZ;

	s3c64xx_clock_event_device.mult = div_sc(clock_rate,
			NSEC_PER_SEC, s3c64xx_clock_event_device.shift);
	s3c64xx_clock_event_device.max_delta_ns =
			clockevent_delta2ns(-1, &s3c64xx_clock_event_device);
	s3c64xx_clock_event_device.min_delta_ns =
			clockevent_delta2ns(1, &s3c64xx_clock_event_device);

	s3c64xx_clock_event_device.cpumask = cpumask_of(0);
	clockevents_register_device(&s3c64xx_clock_event_device);

	setup_irq(IRQ_TIMER3, &s3c64xx_clock_event_irq);
}

/*
 * Clock source
 */

static struct clk *source_in;
static struct clk *source_div;

static cycle_t s3c64xx_clocksource_read(struct clocksource *cs)
{
	return (cycle_t) ~__raw_readl(S3C2410_TCNTO(PWM_SOURCE));
}

static u32 saved_tcnt;

static void s3c64xx_clocksource_suspend(struct clocksource *cs)
{
	saved_tcnt = __raw_readl(S3C2410_TCNTO(PWM_SOURCE));
}

static void s3c64xx_clocksource_resume(struct clocksource *cs)
{
	unsigned long pclk;
	struct clk *tscaler;

	pclk = clk_get_rate(timerclk);
	tscaler = clk_get_parent(source_div);
	clk_set_rate(tscaler, pclk / 3);

	clk_set_rate(source_div, pclk / 6);
	clk_set_parent(source_in, source_div);

	s3c64xx_pwm_init(PWM_SOURCE, saved_tcnt);
	s3c64xx_pwm_start(PWM_SOURCE, ONESHOT);
	s3c64xx_pwm_reconfigure(PWM_SOURCE, TCNT_MAX, PERIODIC);
}

/*
 * Override the global weak sched_clock symbol with this
 * local implementation which uses the clocksource to get some
 * better resolution when scheduling the kernel. We accept that
 * this wraps around for now, since it is just a relative time
 * stamp. (Inspired by U300 implementation.)
 */
static DEFINE_CLOCK_DATA(cd);

unsigned long long notrace sched_clock(void)
{
	return cyc_to_sched_clock(&cd,
			~__raw_readl(S3C2410_TCNTO(PWM_SOURCE)), TCNT_MAX);
}

static void notrace s3c64xx_update_sched_clock(void)
{
	update_sched_clock(&cd,
			~__raw_readl(S3C2410_TCNTO(PWM_SOURCE)), (u32)~0);
}

static struct clocksource pwm_clocksource = {
	.name		= "s3c64xx_clksrc",
	.rating		= 250,
	.read		= s3c64xx_clocksource_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend	= s3c64xx_clocksource_suspend,
	.resume		= s3c64xx_clocksource_resume,
};

static void __init s3c64xx_clocksource_init(void)
{
	unsigned long clock_rate;

	clock_rate = clk_get_rate(source_in);

	s3c64xx_pwm_init(PWM_SOURCE, TCNT_MAX);
	s3c64xx_pwm_start(PWM_SOURCE, PERIODIC);

	init_sched_clock(&cd, s3c64xx_update_sched_clock, 32, clock_rate);

	if (clocksource_register_hz(&pwm_clocksource, clock_rate))
		panic("%s: can't register clocksource\n", pwm_clocksource.name);
}

static void __init s3c64xx_timer_init_common(void)
{
	struct platform_device tmpdev;
	unsigned long pclk;
	struct clk *tscaler;

	tmpdev.dev.bus = &platform_bus_type;

	timerclk = clk_get(NULL, "timers");
	if (IS_ERR(timerclk))
		panic("failed to get timers clock for system timer");

	tmpdev.id = PWM_EVENT;
	event_in = clk_get(&tmpdev.dev, "pwm-tin");
	if (IS_ERR(event_in))
		panic("failed to get pwm-event_in clock for system timer");

	event_div = clk_get(&tmpdev.dev, "pwm-tdiv");
	if (IS_ERR(event_div))
		panic("failed to get pwm-event_div clock for system timer");

	tmpdev.id = PWM_SOURCE;
	source_in = clk_get(&tmpdev.dev, "pwm-tin");
	if (IS_ERR(source_in))
		panic("failed to get pwm-source_in clock for system timer");

	source_div = clk_get(&tmpdev.dev, "pwm-tdiv");
	if (IS_ERR(source_div))
		panic("failed to get pwm-source_div clock for system timer");

	pclk = clk_get_rate(timerclk);
	tscaler = clk_get_parent(event_div);
	clk_set_rate(tscaler, pclk / 3);

	clk_set_rate(event_div, pclk / 6);
	clk_set_parent(event_in, event_div);

	clk_set_rate(source_div, pclk / 6);
	clk_set_parent(source_in, source_div);

	clk_enable(timerclk);
	clk_enable(event_in);
	clk_enable(source_in);
}

static void __init s3c64xx_timer_init(void)
{
	s3c64xx_timer_init_common();
	s3c64xx_clock_event_init();
	s3c64xx_clocksource_init();
}

struct sys_timer s3c64xx_timer = {
	.init		= s3c64xx_timer_init,
};
