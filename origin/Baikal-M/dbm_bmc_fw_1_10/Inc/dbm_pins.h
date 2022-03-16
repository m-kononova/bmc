#ifndef DBM_PINS_H
#define DBM_PINS_H

#include "stm32f4xx.h"

/*
 * Reset PINS
 * If you see others move them to this section
 */
#define PORT_RESET_SPSTX8       GPIOE
#define PIN_RESET_SPSTX8        GPIO_PIN_12

#define PORT_JTAG_RESET         GPIOC            /* TRSTN PHY JTAG*/
#define PIN_JTAG_RESET          GPIO_PIN_2

#define PORT_PJTAG_RESET        GPIOC            /* Core sight TRSTN*/
#define PIN_PJTAG_RESET         GPIO_PIN_3

#define PORT_CPU_RESET_BMC      GPIOG            /* BAIKAL-M RESET_N*/
#define PIN_CPU_RESET_BMC       GPIO_PIN_9

#define PORT_RESET_MEZ10G       GPIOE
#define PIN_RESET_MEZ10G        GPIO_PIN_5

#define PORT_RESET_MEZ_LVDS     GPIOE
#define PIN_RESET_MEZ_LVDS      GPIO_PIN_6

/*
 * SPI multiplexer switchers
 * BMC is connected to each flash simultaneously
 * Eache time, only one MUX must be connected to BMC, the other must
 * be connected to BAIKAL.
 * BAIKAL connected to flash by default. Two differnt spi lines from BAIKAL
 * connected to eache flash.
 */
#define PORT_SPI1_MUX           GPIOC           /* DD55 Secure flash */
#define PIN_SPI1_MUX            GPIO_PIN_15

#define PORT_BOOT_PROG          GPIOF           /* DD57 Boot flash */
#define PIN_BOOT_PROG           GPIO_PIN_4

#define ENABLE_BOOT_FLASH() do {\
	                      	    HAL_GPIO_WritePin(PORT_SPI1_MUX, PIN_SPI1_MUX, GPIO_PIN_RESET);\
							    HAL_GPIO_WritePin(PORT_BOOT_PROG, PIN_BOOT_PROG, GPIO_PIN_SET);\
                             } while(0U)

#define ENABLE_SECURE_FLASH() do {\
								  HAL_GPIO_WritePin(PORT_BOOT_PROG, PIN_BOOT_PROG, GPIO_PIN_RESET);\
	                      	      HAL_GPIO_WritePin(PORT_SPI1_MUX, PIN_SPI1_MUX, GPIO_PIN_SET);\
                              } while(0U)

#define DISABLE_FLASH() do {\
						    HAL_GPIO_WritePin(PORT_BOOT_PROG, PIN_BOOT_PROG, GPIO_PIN_RESET);\
	                      	HAL_GPIO_WritePin(PORT_SPI1_MUX, PIN_SPI1_MUX, GPIO_PIN_RESET);\
                        } while(0U)
/*
 * DC/DC Enable pins
 */
#define PORT_PS_ON              GPIOC
#define PIN_PS_ON               GPIO_PIN_6

#define PORT_EN_VDD_CORE        GPIOE
#define PIN_EN_VDD_CORE         GPIO_PIN_7

#define PORT_EN_VDD1V5          GPIOE
#define PIN_EN_VDD1V5           GPIO_PIN_9

#define PORT_EN_VDD1V8          GPIOE
#define PIN_EN_VDD1V8           GPIO_PIN_10

#define PORT_EN_VDD_RESERVE     GPIOE
#define PIN_EN_VDD_RESERVE      GPIO_PIN_0

#define PORT_EN_VDD3V3_CLK      GPIOE
#define PIN_EN_VDD3V3_CLK       GPIO_PIN_11

#define PORT_EN_VDD_PLL         GPIOC
#define PIN_EN_VDD_PLL          GPIO_PIN_8

#define PORT_PLL_PLUS_50        GPIOF
#define PIN_PLL_PLUS_50         GPIO_PIN_7

#define PORT_PLL_PLUS_100       GPIOF
#define PIN_PLL_PLUS_100        GPIO_PIN_8

#define PORT_PLL_PLUS_200       GPIOF
#define PIN_PLL_PLUS_200        GPIO_PIN_9

#define PORT_PLL_PLUS_400       GPIOF
#define PIN_PLL_PLUS_400        GPIO_PIN_10

#define PORT_VDQ_S3_CH1         GPIOE
#define PIN_VDQ_S3_CH1          GPIO_PIN_2

#define PORT_VDQ_S5_CH1         GPIOE
#define PIN_VDQ_S5_CH1          GPIO_PIN_3

#define PORT_EN_VPP_CH1         GPIOE
#define PIN_EN_VPP_CH1          GPIO_PIN_4

#define PORT_VDQ_S3_CH2         GPIOE
#define PIN_VDQ_S3_CH2          GPIO_PIN_13

#define PORT_VDQ_S5_CH2         GPIOE
#define PIN_VDQ_S5_CH2          GPIO_PIN_14

#define PORT_EN_VPP_CH2         GPIOE
#define PIN_EN_VPP_CH2          GPIO_PIN_15

#define PORT_EN_HDMI_PWR        GPIOF
#define PIN_EN_HDMI_PWR         GPIO_PIN_3

#define PORT_SMBUS_CLK_CURSE    GPIOF
#define PIN_SMBUS_CLK_CURSE     GPIO_PIN_1

#define PORT_SMBUS_SDA_CURSE    GPIOF
#define PIN_SMBUS_SDA_CURSE     GPIO_PIN_0

#define PORT_FAB_PWM_BAIKAL     GPIOB
#define PIN_FAB_PWM_BAIKAL      GPIO_PIN_7

#define PORT_PWR_OK             GPIOC
#define PIN_PWR_OK              GPIO_PIN_7

#define PORT_PGOOD_CLK          GPIOD
#define PIN_PGOOD_CLK           GPIO_PIN_15

#define PORT_PGOOD_PLL          GPIOD
#define PIN_PGOOD_PLL           GPIO_PIN_6

#define PORT_PGOOD_VDQ_CH2      GPIOD
#define PIN_PGOOD_VDQ_CH2       GPIO_PIN_8

#define PORT_PGOOD_VPP_CH2      GPIOD
#define PIN_PGOOD_VPP_CH2       GPIO_PIN_9

#define PORT_PGOOD_VDQ_CH1      GPIOC
#define PIN_PGOOD_VDQ_CH1       GPIO_PIN_10

#define PORT_PGOOD_VPP_CH1      GPIOC
#define PIN_PGOOD_VPP_CH1       GPIO_PIN_11

#define PORT_ALERT_CURSE        GPIOF
#define PIN_ALERT_CURSE         GPIO_PIN_2

#define PORT_FAN_SENS_BMC       GPIOF
#define PIN_FAN_SENS_BMC        GPIO_PIN_5

#ifdef DISCO_BOARD
	#define PORT_SYS_RESET_BTN      GPIOA
	#define PIN_SYS_RESET_BTN       GPIO_PIN_0
#else
	#define PORT_SYS_RESET_BTN      GPIOB
	#define PIN_SYS_RESET_BTN       GPIO_PIN_0
#endif

#define PORT_PWR_RESET_BTN      GPIOB
#define PIN_PWR_RESET_BTN       GPIO_PIN_1

#ifdef DISCO_BOARD
	#define PORT_LED_GREEN          GPIOG
	#define PIN_LED_GREEN           GPIO_PIN_13

	#define PORT_LED_RED            GPIOG
	#define PIN_LED_RED             GPIO_PIN_14
#else
	#define PORT_LED_GREEN          GPIOC
	#define PIN_LED_GREEN           GPIO_PIN_12

	#define PORT_LED_RED            GPIOC
	#define PIN_LED_RED             GPIO_PIN_14
#endif

#define PORT_EN_CLK1_XG         GPIOG
#define PIN_EN_CLK1_XG          GPIO_PIN_12

#define PORT_EN_CLK0_XG         GPIOG
#define PIN_EN_CLK0_XG          GPIO_PIN_13

#define PORT_EN_CLK27MHZ_3V3    GPIOG
#define PIN_EN_CLK27MHZ_3V3     GPIO_PIN_14

#define PORT_EN_CLK25MHZ_3V3    GPIOG
#define PIN_EN_CLK25MHZ_3V3     GPIO_PIN_15

#define PORT_EN_LVDS_VPLL_27M   GPIOC
#define PIN_EN_LVDS_VPLL_27M    GPIO_PIN_0

/*
 * PCI Express present and reset pins
 */

#define PORT_PE_PRSNT0          GPIOD
#define PIN_PE_PRSNT0           GPIO_PIN_11

#define PORT_PE_PRSNT1          GPIOD
#define PIN_PE_PRSNT1           GPIO_PIN_13

#define PORT_PE_PRSNT2          GPIOD
#define PIN_PE_PRSNT2           GPIO_PIN_14

#define PORT_PE_RST_0           GPIOF
#define PIN_PE_RST_0            GPIO_PIN_11

#define PORT_PE_RST_1           GPIOF
#define PIN_PE_RST_1            GPIO_PIN_13

#define PORT_PE_RST_2           GPIOF
#define PIN_PE_RST_2            GPIO_PIN_15

/*
 * I2C pins
 * Add here all I2C pins
 */

#define PORT_SMBUS_CLK_CURSE   GPIOF
#define PIN_SMBUS_CLK_CURSE    GPIO_PIN_1

#define PORT_SMBUS_SDA_CURSE   GPIOF
#define PIN_SMBUS_SDA_CURSE    GPIO_PIN_0

#define PORT_I2C0_SDA_3V3      GPIOC
#define PIN_I2C0_SDA_3V3       GPIO_PIN_9

#define PORT_I2C0_SCL_3V3      GPIOA
#define PIN_I2C0_SCL_3V3       GPIO_PIN_8

/*
 * User button is used on test board it is not exists
 * on DBM board
 */
#define PORT_USER_BTN           GPIOA
#define PIN_USER_BTN            GPIO_PIN_0

typedef struct dbm_pin
{
	GPIO_TypeDef  *port;
	uint16_t       pin;
	GPIO_PinState  defaultState;
	GPIO_PinState  enableState;
	GPIO_TypeDef  *actionPort;
	uint16_t       actionPin;

} dbm_pin_t;


#endif // DBM_PINS_H
