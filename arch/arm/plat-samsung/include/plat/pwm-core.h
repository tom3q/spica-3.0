/* arch/arm/plat-samsung/include/plat/pwm-core.h
 *
 * Copyright 2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * S3C - PWM Controller core functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_PWM_CORE_H
#define __ASM_ARCH_PWM_CORE_H __FILE__

/* These functions are only for use with the core support code, such as
 * the cpu specific initialisation code
 */

#ifdef CONFIG_SAMSUNG_DEV_PWM
extern void s3c_pwm_setname(const char *name);
#else
static inline void s3c_pwm_setname(const char *name) {}
#endif

#endif /* __ASM_ARCH_PWM_H */
