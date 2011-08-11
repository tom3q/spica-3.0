/*
 * Definitions for akm8973 compass chip.
 */
#ifndef AKM8973_H
#define AKM8973_H

#include <linux/ioctl.h>

typedef signed long long		akm_s64;

#define AKM8973_I2C_NAME "akm8973"

/* Compass device dependent definition */
#define AK8973_MODE_MEASURE		0x00
#define AK8973_MODE_E2P_READ	0x02
#define AK8973_MODE_POWERDOWN	0x03

#define SENSOR_DATA_SIZE		5
#define RWBUF_SIZE				16

#define ORI_DATA_READY			1
#define ACC_DATA_READY			2
#define MAG_DATA_READY			4

/* AK8973 register address */
#define AK8973_REG_ST			0xC0
#define AK8973_REG_TMPS			0xC1
#define AK8973_REG_H1X			0xC2
#define AK8973_REG_H1Y			0xC3
#define AK8973_REG_H1Z			0xC4

#define AK8973_REG_MS1			0xE0
#define AK8973_REG_HXDA			0xE1
#define AK8973_REG_HYDA			0xE2
#define AK8973_REG_HZDA			0xE3
#define AK8973_REG_HXGA			0xE4
#define AK8973_REG_HYGA			0xE5
#define AK8973_REG_HZGA			0xE6

#define AK8973_EEP_ETS			0x62
#define AK8973_EEP_EVIR			0x63
#define AK8973_EEP_EIHE			0x64
#define AK8973_EEP_ETST			0x65
#define AK8973_EEP_EHXGA		0x66
#define AK8973_EEP_EHYGA		0x67
#define AK8973_EEP_EHZGA		0x68

#define AKMIO					0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE             _IOW(AKMIO, 0x01, char*)
#define ECS_IOCTL_READ              _IOWR(AKMIO, 0x02, char*)
#define ECS_IOCTL_RESET             _IO(AKMIO, 0x03)
#define ECS_IOCTL_SET_MODE          _IOW(AKMIO, 0x04, short)
#define ECS_IOCTL_GETDATA           _IOR(AKMIO, 0x05, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR           _IOW(AKMIO, 0x06, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS   _IOR(AKMIO, 0x07, int)
#define ECS_IOCTL_GET_CLOSE_STATUS  _IOR(AKMIO, 0x08, int)
#define ECS_IOCTL_GET_DELAY         _IOR(AKMIO, 0x30, short)
#define ECS_IOCTL_GET_PROJECT_NAME  _IOR(AKMIO, 0x0D, char[64])
#define ECS_IOCTL_GET_MATRIX        _IOR(AKMIO, 0x0E, short [4][3][3])

/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MODE		_IOW(AKMIO, 0x10, short)
#define ECS_IOCTL_APP_SET_MFLAG		_IOW(AKMIO, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG		_IOW(AKMIO, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG		_IOW(AKMIO, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG		_IOR(AKMIO, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG		_IOR(AKMIO, 0x15, short)
#define ECS_IOCTL_APP_GET_TFLAG		_IOR(AKMIO, 0x16, short)
#define ECS_IOCTL_APP_RESET_PEDOMETER   _IO(AKMIO, 0x17)
#define ECS_IOCTL_APP_SET_DELAY		_IOW(AKMIO, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY		ECS_IOCTL_GET_DELAY
#define ECS_IOCTL_APP_SET_MVFLAG	_IOW(AKMIO, 0x19, short)
#define ECS_IOCTL_APP_GET_MVFLAG	_IOR(AKMIO, 0x1A, short)

/*AKM DEBUG Interface*/
#define ECS_IOCTL_GET_NANO_DELAY		_IOR(AKMIO, 0xA0, akm_s64[3])
#define ECS_IOCTL_APP_SET_MDELAY		_IOW(AKMIO, 0xA1, akm_s64)
#define ECS_IOCTL_APP_SET_ADELAY		_IOW(AKMIO, 0xA2, akm_s64)
#define ECS_IOCTL_APP_SET_MVDELAY		_IOW(AKMIO, 0xA3, akm_s64)
#define ECS_IOCTL_SET_YPR_SELECT        _IOW(AKMIO, 0xA4, short[13])
/*DEBUG IF END*/

struct akm8973_platform_data {
	short layouts[4][3][3];
	char project_name[64];
	int gpio_RST;
	int gpio_INT;
};

#endif

