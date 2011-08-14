#ifndef __LINUX_SPICA_BT_H
#define __LINUX_SPICA_BT_H

struct spica_bt_pdata {
	unsigned int	gpio_host_wake;

	void		(*set_power)(int);
};

#endif /* __LINUX_SPICA_BT_H */
