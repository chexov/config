/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

//#define DEBUG_MODE DEBUG_STACK

#define DEFAULT_RX_FEATURE            FEATURE_RX_SERIAL
#define DEFAULT_FEATURES              (FEATURE_TELEMETRY | FEATURE_OSD )


#define USE_GYRO
#define USE_ACC
#define USE_BARO
#define USE_FLASH

#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_BMP280
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define FC_TARGET_MCU STM32F7X2
#define BOARD_NAME KIWIF7
#define MANUFACTURER_ID KAPO

#define TARGET_BOARD_IDENTIFIER "KIWI"
#define USBD_PRODUCT_STRING     "KIWIF7x"


#define MOTOR1_PIN PA9
#define MOTOR2_PIN PA8
#define MOTOR3_PIN PC9
#define MOTOR4_PIN PC8

#define SERVO1_PIN PA0
#define SERVO2_PIN PA1

#define UART1_TX_PIN PB6
#define UART1_RX_PIN PB7
#define UART2_TX_PIN PA2
#define UART2_RX_PIN PA3
#define UART3_TX_PIN PB10
#define UART3_RX_PIN PB11
#define UART4_TX_PIN PC10
#define UART4_RX_PIN PC11
#define UART5_TX_PIN PC12
#define UART5_RX_PIN PD2
#define UART6_TX_PIN PC6
#define UART6_RX_PIN PC7

#define I2C1_SCL_PIN PB8
#define I2C1_SDA_PIN PB9

#define LED0_PIN PB1
#define LED_STRIP_PIN PB0

#define SPI1_SCK_PIN PA5    // SPI1_CLK
#define SPI1_SDI_PIN PA6    // SPI1_MISO
#define SPI1_SDO_PIN PA7    // SPI1_MOSI

#define SPI2_SCK_PIN PB13   // SPI2_CLK
#define SPI2_SDI_PIN PB14   // SPI2_MISO
#define SPI2_SDO_PIN PB15   // SPI2_MOSI

#define SPI3_SCK_PIN PB3    // SPI3_CLK
#define SPI3_SDI_PIN PB4    // SPI3_MISO
#define SPI3_SDO_PIN PB5    // SPI3_MOSI

#define ADC_VBAT_PIN PC2
#define ADC_CURR_PIN PC0

//PB2   PIN_DE_CHECK_OUT
//PB12  PIN_WIS
//PC4   PIN_DE_CHECK_IN
#define VTX1_ENABLE_PIN PC14   //   PIN_VTX_PWR_CTR1
#define VTX2_ENABLE_PIN PC3    //   PIN_VTX_PWR_CTRL2

#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN PA4  //  PIN_F
#define PINIO2_PIN PC15 //  PIN_SAFETY_CHECK
#define PINIO3_PIN PA13 //  PIN_CAMERA_SWITCH
#define PINIO4_PIN VTX1_ENABLE_PIN

#define GYRO_1_CS_PIN PA14     // IMU_CS
#define GYRO_1_EXTI_PIN PA10   // IMU_INT
#define MAX7456_SPI_CS_PIN PC1 // OSD_CS
#define FLASH_CS_PIN PC13      //  FLASH_CS

//#define USE_GYRO_CLKIN
//#define GYRO_1_CLKIN_PIN PA15 // IMU_EXT_CLK
//#define GYRO_1_ALIGN CW180_DEG


#define ADC1_DMA_OPT 1

#define PINIO1_BOX 40
#define PINIO1_CONFIG 1
#define PINIO2_BOX 41
#define PINIO2_CONFIG 1

#define USE_RACE_PRO

#define USE_OSD

#define OSD_DISPLAYPORT_DEVICE MAX7456

#define GYRO_1_SPI_INSTANCE SPI1
#define MAX7456_SPI_INSTANCE SPI2  // PC1  // OSD_CS
#define FLASH_SPI_INSTANCE SPI3


#define MAG_I2C_INSTANCE (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define ADC_INSTANCE ADC1
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 110
#define DEFAULT_CURRENT_METER_SCALE 182


#define TIMER_PIN_MAPPING   \
    TIMER_PIN_MAP( 0,  MOTOR1_PIN,  2,  0 ) \
    TIMER_PIN_MAP( 1,  MOTOR2_PIN ,  2,  1 ) \
    TIMER_PIN_MAP( 2,  MOTOR3_PIN,  2,  1 ) \
    TIMER_PIN_MAP( 3,  MOTOR4_PIN,  2,  1 ) \
    TIMER_PIN_MAP( 4,  SERVO1_PIN,  1, -1 ) \
    TIMER_PIN_MAP( 5,  SERVO2_PIN,  1, -1 )


#define MSP_DISPLAYPORT_UART    SERIAL_PORT_USART1
#define SERIALRX_UART           SERIAL_PORT_USART2
#define MSP_UART                SERIAL_PORT_UART4
#define ESC_SENSOR_UART         SERIAL_PORT_UART5
