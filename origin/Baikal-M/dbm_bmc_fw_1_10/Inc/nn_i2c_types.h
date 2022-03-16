#ifndef NN_I2C_TYPES_H
#define NN_I2C_TYPES_H

typedef uint16_t i2caddr_t;

typedef enum {
	READY_ON_STANDBY,
	READY_ON_POWERON
}ReadReadyState_t;

// This device have just 2 channels for measurement
typedef enum _channel_t
{
	CHANNEL_NO,
	CHANNEL_1,
	CHANNEL_2
}channel_t;

typedef enum _adgVolatge_t
{
	V_0v8  = 0x01,
	V_1v0  = 0x02,
	V_0v85 = 0x04,
	V_1v05 = 0x08,
	V_0v90 = 0x10,
	V_1v10 = 0x20,
	V_0v95 = 0x40,
	V_0v80 = 0x80
} adgVoltage;

// Definde I2C device info
typedef struct i2c_dev
{
	char *namePCB;
	char *name;
	char *description;
	i2caddr_t addr;
	ReadReadyState_t state;
	channel_t channel;
	uint16_t resValue;
} i2c_dev_t;

#endif //NN_I2C_TYPES_H
