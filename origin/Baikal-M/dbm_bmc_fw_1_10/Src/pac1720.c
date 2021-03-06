/*
 * pac1720.c
 *
 *  Created on: Jun 20, 2017
 *      Author: root
 */

#include "pac1720.h"
#include "cmsis_os.h"

// ---------- ---------- ---------- -------------------- ----------
uint8_t getProductId(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	uint8_t res = readByteBlock(i2cp, addr, PAC1720_REG_ID_PRODUCT);
	return res;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t getSmscId(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	uint8_t res = readByteBlock(i2cp, addr, PAC1720_REG_ID_SMSC);
	return res;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t getRevisionId(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	uint8_t res = readByteBlock(i2cp, addr, PAC1720_REG_ID_REVISION);
	return res;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void initDevice(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	disableAllChannels(i2cp, addr);

	// It is necessary to wait 220ms (this value from data sheet)
	// to let PAC1720 complete write operation
	osDelay(220);

	uint8_t rVal = readByteBlock(i2cp, addr, PAC1720_REG_CONFIG);
	if (rVal != REG_CONFIG_DISABLE)
	{
		return; // In this case, channels where not been turned off
	}

	// Set maximum conversion speed
	setConversionRate(i2cp, addr, REG_CONV_RATE_C_PER_SEC);
	enableAllChannels(i2cp, addr);
}

// ---------- ---------- ---------- ---------- ---------- ----------
float readCurrentValue(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch, uint16_t rSens)
{
	uint8_t hb = 0x00;
	uint8_t lb = 0x00;

	if (ch == CHANNEL_2)
	{
		hb = readByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH2_HB);
		lb = readByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH2_LB);
	}
	else
	{
		hb = readByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH1_HB);
		lb = readByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH1_LB);
	}

	float vSens = getVSensValue(hb, lb);

	// This is resistor that is installed between SENSE- and SENSE+ 16 Milli Omh
	// For DBM in rSens is mili Omh
	float  fsr       = getVSensFSR(i2cp, addr, ch);       // 80
	uint16_t dominator = getVSensDominator(i2cp, addr, ch); // 2047;
	float  result = (fsr / rSens) * (vSens / dominator);

	return result;
}

// ---------- ---------- ---------- ---------- ---------- ----------
float readVoltageValue(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch)
{
	uint8_t hb = 0x00;
	uint8_t lb = 0x00;

	if (ch == CHANNEL_2)
	{
		hb = readByteBlock(i2cp, addr, PAC1720_REG_VSOURCE_VOLTAGE_CH2_HB);
		lb = readByteBlock(i2cp, addr, PAC1720_REG_VSOURCE_VOLTAGE_CH2_LB);
	}
	else
	{
		hb = readByteBlock(i2cp, addr, PAC1720_REG_VSOURCE_VOLTAGE_CH1_HB);
		lb = readByteBlock(i2cp, addr, PAC1720_REG_VSOURCE_VOLTAGE_CH1_LB);
	}

	float vSource = getVSourceValue(hb, lb);

// Old method to get results (read data sheet carefully)
//	uint16_t dominator1 = getVSourceDominator1(i2cp, addr, ch);
//	uint16_t dominator2 = getVSourceDominator2(i2cp, addr, ch);
//	float result = (40 - (40 / dominator1)) * (vSource/dominator2);
	float result = vSource;
	return result;
}

// ---------- ---------- ---------- ---------- ---------- ----------
float readPowerValue(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch, uint16_t rSens)
{
	uint8_t hb = 0x00;
	uint8_t lb = 0x00;

	if (ch == CHANNEL_2)
	{
		hb = readByteBlock(i2cp, addr, PAC1720_REG_POWER_RATIO_CH2_HB);
		lb = readByteBlock(i2cp, addr, PAC1720_REG_POWER_RATIO_CH2_LB);
	}
	else
	{
		hb = readByteBlock(i2cp, addr, PAC1720_REG_POWER_RATIO_CH1_HB);
		lb = readByteBlock(i2cp, addr, PAC1720_REG_POWER_RATIO_CH1_LB);
	}

	uint32_t Pratio = getVPowerValue(hb, lb);

	float  fsr       = getVSensFSR(i2cp, addr, ch);
	float  fsc = fsr / rSens;

	uint16_t dominator1 = getVSourceDominator1(i2cp, addr, ch);
	float  fsv = (40 - (40/dominator1));

	float power = (fsc * fsv * Pratio) / 65535;
	return power;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint16_t getVSensDominator(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch)
{
	uint8_t value = 0x00;
	if (ch == CHANNEL_2)
	{
		value = readByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSENSE_CH2);
	}
	else
	{
		value = readByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSENSE_CH1);
	}

	value = value >> 4;

	value &= 0x07; // Clear all except 3 low bytes
	switch (value)
	{
		case VSENS_DOM_002v5MS_06BIT_63:
			return 63;
		case VSENS_DOM_005v0MS_07BIT_127:
			return 127;
		case VSENS_DOM_010v0MS_08BIT_255:
			return 255;
		case VSENS_DOM_020v0MS_09BIT_511:
			return 511;
		case VSENS_DOM_040v0MS_10BIT_1023:
			return 1023;
		case VSENS_DOM_080v0MS_11BIT_2047:
			return 2047;
		case VSENS_DOM_160v0MS_11BIT_2047:
			return 2047;
		case VSENS_DOM_320v0MS_11BIT_2047:
			return 2047;
	}
	return 0;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t getVSensFSR(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch)
{
	uint8_t value = 0x00;
	if (ch == CHANNEL_2)
	{
		value = readByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSENSE_CH2);
	}
	else
	{
		value = readByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSENSE_CH1);
	}

	value &= 0x03; // Clear all except 3 low bytes
	switch (value)
	{
		case FSR_10MV:
			return 10;
		case FSR_20MV:
			return 20;
		case FSR_40MV:
			return 40;
		case FSR_80MV:
			return 80;
	}
	return 0;
}

// ---------- ---------- ---------- ---------- ---------- ----------
int16_t getVSensValue(uint8_t highByte, uint8_t lowByte)
{
	int16_t hblb = 0x00;

	hblb = hblb +  highByte;
	hblb = hblb << 8;
	hblb = hblb +  lowByte;

	if ((highByte & 0x80) == 0x80)
	{
		// Convert negative 2???s complement format to
		// positive value
		hblb = (~hblb);
		hblb = hblb >> 4; // Four low bytes always zero
		hblb = hblb + 1;
		hblb = hblb * (-1);
	}
	else
	{
		hblb = hblb >> 4; // Four low bytes always zero
	}

	return hblb;
}

// ---------- ---------- ---------- ---------- ---------- ----------
float getVSourceValue(uint8_t highByte, uint8_t lowByte)
{
	uint16_t hblb = 0x00;

	hblb = hblb +  highByte;
	hblb = hblb << 8;
	hblb = hblb +  lowByte;

	hblb = hblb >> 5;

	float   sumResult = 0;
	float   sumWeight = 0.0195;
	uint8_t i = 1;
	for(; i <= 11; ++i)
	{
		if ((hblb & 0x0001) == 0x0001)
		{
			sumResult += sumWeight;
		}

		hblb = hblb >> 1;
		sumWeight = sumWeight * 2;
	}

	return sumResult;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint32_t getVPowerValue(uint8_t highByte, uint8_t lowByte)
{
	uint16_t hblb = 0x00;

	hblb = hblb +  highByte;
	hblb = hblb << 8;
	hblb = hblb +  lowByte;

	uint32_t result = 0x00000000;
	uint32_t sumWeight = 1;
	uint8_t i = 0;
	for(; i < 16; ++i)
	{
		if ((hblb & 0x0001) == 0x0001)
		{
			result += sumWeight;
		}

		hblb = hblb >> 1;
		sumWeight = sumWeight * 2;
	}

	return result;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint16_t getVSourceDominator1(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch)
{
	uint8_t value = readByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSOURSE);

	if (ch == CHANNEL_2)
	{
		value = value >> 6;
	}
	else
	{
		value = value >> 2;
	}

	value = value & 0x03; // Clear all bits except last 2
	switch (value)
	{
		case VSORCE_DOM_002v5MS_08BIT_256_255:
			return 256;
		case VSORCE_DOM_005v0MS_09BIT_512_511:
			return 512;
		case VSORCE_DOM_010v0MS_10BIT_1024_1023:
			return 1024;
		case VSORCE_DOM_020v0MS_11BIT_2048_2047:
			return 2048;
	}
	return 0;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint16_t getVSourceDominator2(I2C_HandleTypeDef* i2cp, i2caddr_t addr, channel_t ch)
{
	uint8_t value = readByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSOURSE);

	if (ch == CHANNEL_2)
	{
		value = value >> 6;
	}
	else
	{
		value = value >> 2;
	}

	value = value & 0x03; // Clear all bits except last 2
	switch (value)
	{
		case VSORCE_DOM_002v5MS_08BIT_256_255:
			return 255;
		case VSORCE_DOM_005v0MS_09BIT_512_511:
			return 512;
		case VSORCE_DOM_010v0MS_10BIT_1024_1023:
			return 1023;
		case VSORCE_DOM_020v0MS_11BIT_2048_2047:
			return 2047;
	}
	return 0;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void disableAllChannels(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_CONFIG, REG_CONFIG_DISABLE);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void enableAllChannels(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_CONFIG, REG_CONFIG_ENABLE_ALL);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void enableAlertPin(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	uint8_t value = readByteBlock(i2cp, addr, PAC1720_REG_CONFIG);
	value |= REG_CONFIG_ENABLE_ALERT_PIN_BIT;
	writeByteBlock(i2cp, addr, PAC1720_REG_CONFIG, value);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setConversionRate(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t rate)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_CONV_RATE, rate);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setVSenceSamplingRegister(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint8_t value)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSENSE_CH1, value);
	writeByteBlock(i2cp, addr, PAC1720_REG_SAMPLING_VSENSE_CH2, value);
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t getHighLimitStatus(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	uint8_t value = readByteBlock(i2cp, addr, PAC1720_REG_STATUS_HL);
	return value;
}

// ---------- ---------- ---------- ---------- ---------- ----------
uint8_t getLowLimitStatus(I2C_HandleTypeDef* i2cp, i2caddr_t addr)
{
	uint8_t value = readByteBlock(i2cp, addr, PAC1720_REG_STATUS_LL);
	return value;
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setCurrentHighLimitCh1(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint16_t rSens, uint8_t value)
{
	// This is resistor that is installed between SENSE- and SENSE+ 16 Milli Omh
	// For DBM in rSens is mili Omh
	uint16_t fsr       = getVSensFSR(i2cp, addr, CHANNEL_1);       // 80
	uint16_t dominator = getVSensDominator(i2cp, addr, CHANNEL_1); // 2047;
	uint16_t vSens = 0x0000;
	uint16_t fsc = fsr / rSens;
	vSens = (value * dominator) / fsc;

	vSens >>= 4;
	// vSens = 0x07;

	writeByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH1_HL, vSens);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setCurrentHighLimitCh2(I2C_HandleTypeDef* i2cp, i2caddr_t addr, uint16_t rSens, uint8_t value)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH2_HL, value);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setCurrentLowLimitCh1(I2C_HandleTypeDef* i2cp, i2caddr_t addr,  uint16_t rSens, uint8_t value)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH1_LL, value);
}

// ---------- ---------- ---------- ---------- ---------- ----------
void setCurrentLowLimitCh2(I2C_HandleTypeDef* i2cp, i2caddr_t addr,  uint16_t rSens, uint8_t value)
{
	writeByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH2_LL, value);
}

// ---------- ---------- ---------- ---------- ---------- ----------
float getCurrentHightLimitCh1(I2C_HandleTypeDef* i2cp, i2caddr_t addr,  uint16_t rSens)
{

	uint8_t hb = 0x00;
	uint8_t lb = 0x00;

	hb = readByteBlock(i2cp, addr, PAC1720_REG_SENCE_VOLTAGE_CH1_HL);

	float vSens = getVSensValue(hb, lb);

	// This is resistor that is installed between SENSE- and SENSE+ 16 Milli Omh
	// For DBM in rSens is mili Omh
	float  fsr       = getVSensFSR(i2cp, addr, CHANNEL_1);       // 80
	uint16_t dominator = getVSensDominator(i2cp, addr, CHANNEL_1); // 2047;
	float  result = (fsr / rSens) * (vSens / dominator);

	return result;
}
