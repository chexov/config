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

#define FC_TARGET_MCU STM32F7X2
#define SYSTEM_HSE_MHZ 16

#define BOARD_NAME KAPONGA_KIWIF71
#define MANUFACTURER_ID KPNG

#define USE_RACE_PRO
#define DEFAULT_SMALL_ANGLE 180
#define DEFAULT_FAILSAFE_RECOVERY_DELAY 1            // 100ms of valid rx data needed to allow recovery from failsafe and arming block



//#define USE_WING
// #define USE_ESAD
#define USE_GPS
#define USE_POSITION_HOLD
#define USE_ALTITUDE_HOLD

#define USE_GYRO
#define USE_ACC
#define USE_BARO
#define USE_FLASH

#define USE_RX_MSP
#define USE_RX_MSP_OVERRIDE
#define USE_RX_RSSI_DBM
#define USE_RX_RSNR
#define USE_RX_LINK_QUALITY_INFO
#define USE_RX_LINK_UPLINK_POWER

#define USE_OSD
#define USE_LED_STRIP
#define USE_SERVOS
//#define USE_CHIRP


#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_BMP388
#define USE_FLASH_W25Q128FV
#define USE_MAX7456


#define MOTOR1_PIN PA9
#define MOTOR2_PIN PA8
#define MOTOR3_PIN PC9
#define MOTOR4_PIN PC8

#define SERVO1_PIN PA1
#define SERVO2_PIN PA0

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

#define LED0_PIN PB1 // LED_STATUS
#define LED1_PIN PB0 // RGB

#define SPI1_SCK_PIN PA5    // SPI1_CLK
#define SPI1_SDI_PIN PA6    // SPI1_MISO
#define SPI1_SDO_PIN PA7    // SPI1_MOSI

#define SPI2_SCK_PIN PB13   // SPI2_CLK
#define SPI2_SDI_PIN PB14   // SPI2_MISO
#define SPI2_SDO_PIN PB15   // SPI2_MOSI

#define SPI3_SCK_PIN PB3    // SPI3_CLK
#define SPI3_SDI_PIN PB4    // SPI3_MISO
#define SPI3_SDO_PIN PB5    // SPI3_MOSI

#define ADC_VBAT_PIN PC4
#define ADC_CURR_PIN PC0

#define RELAY1_PIN PC14   //   PIN_VTX_PWR_CTRL1
#define RELAY2_PIN PC13   //   PIN_VTX_PWR_CTRL2
#define CAM_SWITCH_PIN PA13 //  PIN_CAMERA_SWITCH

#define USE_PINIO
#define USE_PINIOBOX
#define USE_CUSTOM_BOX_NAMES

#define PINIO1_PIN NONE //
// #define PINIO2_PIN PC2 //  PIN_PARACHUTE_SAFETY
// #define PINIO3_PIN CAM_SWITCH_PIN
// #define PINIO4_PIN RELAY1_PIN

#define PINIO2_PIN CAM_SWITCH_PIN
#define PINIO3_PIN RELAY1_PIN
#define PINIO4_PIN RELAY2_PIN


//#define PINIO_COUNT 10
//#define PINIO5_PIN RELAY2_PIN
//#define PINIO6_PIN PA4  // PIN_PARACHUTE_DEPLOY
//#define PINIO7_PIN PB12 // PIN_WIS
//PC3   PIN_DE_CHECK_IN
//PC4   PIN_DE_CHECK_OUT
//PC2   PIN_REMOVE_BEFORE_FLIGHT_SAFETY_CHECK


#define PINIO1_BOX 40 // user
#define PINIO2_BOX 41 // User 2; Parachute safety
#define PINIO3_BOX 42 // User 3; Camera switch
#define PINIO4_BOX 43 // User 4; Turn VTX1 on/off
#define PINIO5_BOX 44 // User 5; Turn VTX2 on/off
#define PINIO6_BOX 45 // User 6; Deploy Parachute
#define PINIO7_BOX 46 //
#define PINIO8_BOX 47 //
#define PINIO9_BOX 48 //

#define PINIO1_CONFIG PINIO_CONFIG_MODE_OUT_PP
#define PINIO2_CONFIG PINIO_CONFIG_MODE_OUT_PP
#define PINIO3_CONFIG PINIO_CONFIG_MODE_OUT_PP
#define PINIO4_CONFIG PINIO_CONFIG_MODE_OUT_PP // VBAT enable, active low
//#define PINIO4_CONFIG (PINIO_CONFIG_MODE_OUT_PP | PINIO_CONFIG_OUT_INVERTED) // VBAT enable, active low

#define PINIO5_CONFIG PINIO_CONFIG_MODE_OUT_PP | PINIO_CONFIG_OUT_INVERTED // VTX2 VBAT enable, active low
#define PINIO6_CONFIG PINIO_CONFIG_MODE_OUT_PP | PINIO_CONFIG_OUT_INVERTED // VBAT enable, active low
#define PINIO7_CONFIG 0 //PINIO_CONFIG_MODE_OUT_PP | PINIO_CONFIG_OUT_INVERTED // VBAT enable, active low

#define GYRO_1_CS_PIN PA10     // IMU_CS
#define GYRO_1_EXTI_PIN PC15   // IMU_INT
#define GYRO_1_CLKIN_PIN PA15 // IMU_EXT_CLK

#define MAX7456_SPI_CS_PIN PC1 // OSD_CS
#define FLASH_CS_PIN PA14      // FLASH_CS


#define USE_GYRO_CLKIN
#define GYRO_1_ALIGN CW90_DEG
#define ADC1_DMA_OPT 1
#define OSD_DISPLAYPORT_DEVICE MAX7456
#define MAX7456_SPI_INSTANCE SPI1  // PC1  // OSD_CS
#define GYRO_1_SPI_INSTANCE SPI2
#define FLASH_SPI_INSTANCE SPI3


#define MAG_I2C_INSTANCE (I2CDEV_1)
#define BARO_I2C_INSTANCE (I2CDEV_1)
#define ADC_INSTANCE ADC1


#define TIMER_PIN_MAPPING   \
    TIMER_PIN_MAP( 0,  MOTOR1_PIN,  2,  0 ) \
    TIMER_PIN_MAP( 1,  MOTOR2_PIN,  2,  1 ) \
    TIMER_PIN_MAP( 2,  MOTOR3_PIN,  2,  1 ) \
    TIMER_PIN_MAP( 3,  MOTOR4_PIN,  2,  1 ) \
    TIMER_PIN_MAP( 4,  SERVO1_PIN,  1, -1 ) \
    TIMER_PIN_MAP( 5,  SERVO2_PIN,  1, -1 )


#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 1052
#define DEFAULT_VOLTAGE_METER_SCALE 110

#define DEFAULT_FEATURES              (FEATURE_RX_SERIAL |FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_GPS)

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_CRSF

#define VTX_TRAMP_UART     SERIAL_PORT_USART1
#define SERIALRX_UART           SERIAL_PORT_USART2
#define GPS_UART               SERIAL_PORT_UART3
// #define ESAD_UART               SERIAL_PORT_UART4
#define MSP_UART                SERIAL_PORT_UART5
#define ESC_SENSOR_UART         SERIAL_PORT_USART6
