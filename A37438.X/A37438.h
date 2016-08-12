/* 
 * File:   A37438.h
 * Author: hwanetick
 *
 * Created on August 3, 2016, 6:44 PM
 */

#ifndef A37438_H
#define	A37438_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <spi.h>
#include <uart.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"
//#include "faults.h"

#define FCY_CLK                    10000000

#define __noModbusLibrary
// --------- Resource Summary  -----------------
/*
  Hardware Module Resource Usage

  CAN2   - Used/Configured by ETM CAN (optical CAN) 
  CAN1   - Used/Configured by ETM CAN (hardware CAN) - Not implimented/tested
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used for communicating with Converter Logic Board
  SPI2   - Used for communicating with on board DAC

  Timer2 - Used for 10msTicToc 

  ADC Module - AN3,AN4,AN5,AN6,AN7,VREF+,VREF-,AN13,AN14,AN15

  I2C    - Used to communicate with on board EEPROM (not used at this time)
  
  EEPROM - The internal EEPROM is used at this time
  
*/





// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set


// Pins to be configured as inputs


/*
  RA9  - ADC VREF-
  RA10 - ADC VREF+
  RA14 - PIN_CUSTOMER_HV_ON  <----------Trigger Input Interrupt
 
  RB0  - ICD  - PROGRAM
  RB1  - ICD  - PROGRAM
  RB3  - AN3  - POT EK
  RB4  - AN4  - POT VTOP
  RB5  - AN5  - POT HTR
  RB6  - AN6  - REF HTR        <-----------SW Bit 1
  RB7  - AN7  - REF VTOP       <-----------SW Bit 3
  RB13 - AN13 - REF EK         <-----------SW Bit 2
  RB14 - AN14 - PIC ADC +15V MON
  RB15 - AN15 - PIC ADC -15V MON

  RC1  - DAC LDAC  (Configured by DAC module)

  RD8  - PIN_CUSTOMER_BEAM_ENABLE  <-----------SW Bit 0


  RF0  - CAN 1 (Configured By Pic Module)
  RF1  - CAN 1 (Configured By Pic Module)
  RF2  - UART 1 (Configured By Pic Module)
  RF3  - UART 1 (Configured By Pic Module)
  RF6  - SPI 1 (Configured By Pic Module)
  RF7  - SPI 1 (Configured By Pic Module)
  RF8  - SPI 1 (Configured By Pic Module)


  RG0  - CAN 2 (Configured By Pic Module)
  RG1  - CAN 2 (Configured By Pic Module)
  RG2  - I2C   (Configured By Pic Module)
  RG3  - I2C   (Configured By Pic Module)
  RG6  - SPI2 CLK
  RG7  - SPI2 DI
  RG8  - SPI2 DO
  RG14 - Reset Detect
  RG15 - DAC CS/LD (Configured by DAC module)

*/

#define A36772_TRISA_VALUE 0b0100011000000000 
#define A36772_TRISB_VALUE 0b1110000011111011 
#define A36772_TRISC_VALUE 0b0000000000000010 
#define A36772_TRISD_VALUE 0b0000000100000000 
#define A36772_TRISF_VALUE 0b0000000111001111 
#define A36772_TRISG_VALUE 0b1100000111001111 


/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9.0uS
  8 Samples per Interrupt, use alternating buffers
  Conversion rate of 111KHz (13.875 Khz per Channel), 138 Samples per 10mS interrupt
  Scan Through Selected Inputs (8 selected at any point in time)

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN3 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)
#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13)




// Digital Inputs
#define PIN_TRIGGER_INPUT                            _RA14
#define ILL_PIN_TRIGGER_INPUT                        1


#define PIN_SW_BIT0_STATUS                           _RD8  
#define ILL_PIN_SW_BIT0_ON                           1

#define PIN_SW_BIT1_STATUS                           _RB6
#define PIN_SW_BIT2_STATUS                           _RB13
#define PIN_SW_BIT3_STATUS                           _RB7
#define ILL_PIN_SW_BIT1_ON                           0


//------------------- GUN Driver Interface I/O ------------------------- //
#define PIN_CS_DAC                                   _LATD13
#define OLL_PIN_CS_DAC_SELECTED                      1

#define PIN_CS_ADC                                   _LATD14
#define OLL_PIN_CS_ADC_SELECTED                      1

#define PIN_CS_FPGA                                  _LATD15
#define OLL_PIN_CS_FPGA_SELECTED                     1



// Digital Outputs
#define PIN_CPU_SWITCH_BIT0_ENABLE                   _LATD0
#define PIN_CPU_SWITCH_BIT1_ENABLE                   _LATD11
#define PIN_CPU_SWITCH_BIT2_ENABLE                   _LATD10
#define PIN_CPU_SWITCH_BIT3_ENABLE                   _LATA15

#define PIN_CPU_BEAM_ENABLE_STATUS                   _LATF5  // DPARKER THERE IS ERROR ON SCHEMATIC
#define PIN_CPU_EXTRA_STATUS                         _LATD3
#define OLL_STATUS_ACTIVE                            1

#define PIN_CPU_HV_ENABLE                            _LATD2
#define PIN_CPU_BEAM_ENABLE                          _LATD1
#define OLL_PIN_CPU_BEAM_ENABLE_BEAM_ENABLED         1
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED             1

#define PIN_RS485_ENABLE                             _LATF4  // DPARKER THERE IS ERROR ON SCHEMATIC


// LED Indicator Output Pins
#define OLL_LED_ON                                   0

#define PIN_LED_I1B                                  _LATG12
#define PIN_LED_I1C                                  _LATG13
#define PIN_LED_I1D                                  _LATA7

#define PIN_LED_I2A                                  _LATC2
#define PIN_LED_I2B                                  _LATC3
#define PIN_LED_I2C                                  _LATC4
#define PIN_LED_I2D                                  _LATA6

#define PIN_RESET_DETECT_OUTPUT                      _LATG14
#define PIN_RESET_DETECT_INPUT                       _RG14

#define PIN_TEST_POINT_B                             _LATF5
#define PIN_TEST_POINT_E                             _LATB8
#define PIN_TEST_POINT_F                             _LATB9



#define PIN_LED_BEAM_ENABLE                          PIN_LED_I2A
#define PIN_LED_I2C_OPERATION                        PIN_LED_I2B  // This pin is controlled by the CAN Module
#define PIN_LED_OPERATIONAL                          PIN_LED_I2C
#define PIN_LED_SYSTEM_OK                            PIN_LED_I2D

// THIS PIN IS ALWAYS ON WHEN POWERED                PIN_LED_I1A
#define PIN_LED_WARMUP                               PIN_LED_I1B
#define PIN_LED_STANDBY                              PIN_LED_I1C
#define PIN_LED_HV_ON                                PIN_LED_I1D



// -----------------------  END IO PIN CONFIGURATION ------------------------ //



// -------------------------------------------- INTERNAL MODULE CONFIGURATION --------------------------------------------------//

/*
  --- SPI1 Port --- 
  This SPI port is used to connect with the gun driver
  This must be slower to compensate for the 2x delay across the optocoupler 200ns with filtering in one direction, 80ns (without filtering) in the other direction
  Minimum clock period is therefore 280ns + holdtime + margins
*/
#define A36772_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A36772_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   


/*
  --- Timer2 Setup ---
  Period of 10mS
*/
#define A36772_T2CON_VALUE     (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define A36772_PR2_VALUE_US    10000   // 10mS
#define A36772_PR2_VALUE       ((FCY_CLK/1000000)*A36772_PR2_VALUE_US/8)

/*
  --- Timer3 Setup ---
  Period of 10uS
*/
#define A36772_T3CON_VALUE     (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_1 & T3_SOURCE_INT)
#define A36772_PR3_VALUE_US    100   // 100us
#define A36772_PR3_VALUE       ((FCY_CLK/1000000)*A36772_PR3_VALUE_US)

 
// ---- Transmit start delay after trigger received in 100us ---- //

#define TRANSMIT_DELAY_TIME     20    // 2ms


  
// MAX1230 Control Words
#define MAX1230_CONVERSION_BYTE                      0b10000011
#define MAX1230_SETUP_BYTE                           0b01101000
#define MAX1230_AVERAGE_BYTE                         0b00111000
#define MAX1230_RESET_BYTE                           0b00010000



typedef struct {
  
  volatile unsigned char control_config;        // This indicates when all set values from the CAN interface have been received

  unsigned int run_time_counter;                // This counts how long the unit has been running for.  It wraps every 11 minutes
  
  unsigned int last_period;
  unsigned int trigger_complete;
  unsigned int this_pulse_level_energy_command;
  unsigned int next_pulse_level_energy_command;
  
  unsigned int trigger_received;
  unsigned int delay_time;
  unsigned int waiting_to_transmit;
  
  unsigned char dose_switch_value;
  
  unsigned char message0_dose;
  unsigned char message1_energy;
  unsigned char message2_blank;
  unsigned char message3_blank;
  unsigned char message4_crc_low;
  unsigned char message5_crc_high;

  
  TYPE_DIGITAL_INPUT switch_bit_0;
  TYPE_DIGITAL_INPUT switch_bit_1;
  TYPE_DIGITAL_INPUT switch_bit_2;
  TYPE_DIGITAL_INPUT switch_bit_3;
  
} TYPE_GLOBAL_DATA_A36772;

extern TYPE_GLOBAL_DATA_A36772 global_data_A36772;

#define  DOSE_LOOKUP_VALUES             0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0,0xB0,0xC0,0xD0,0xE0,0xF0,0xFF
#define  CRC_HIGH_ENERGY_LOOKUP_VALUES  0x2455,0x245A,0xE45E,0x2444,0xE440,0xE44F,0x244B,0x2478,0xE47C,0xE473,0x2477,0xE46D,0x2469,0x2466,0xE462,0xF061
#define  CRC_LOW_ENERGY_LOOKUP_VALUES   0xE404,0xE40B,0x240F,0xE415,0x2411,0x241E,0xE41A,0xE429,0x242D,0x2422,0xE426,0x243C,0xE438,0xE437,0x2433,0x3030

#define  DOSE_0      0x00
#define  DOSE_1      0x01
#define  DOSE_2      0x02
#define  DOSE_3      0x03
#define  DOSE_4      0x04
#define  DOSE_5      0x05
#define  DOSE_6      0x06
#define  DOSE_7      0x07
#define  DOSE_8      0x08
#define  DOSE_9      0x09                            
#define  DOSE_A      0x0A
#define  DOSE_B      0x0B
#define  DOSE_C      0x0C
#define  DOSE_D      0x0D
#define  DOSE_E      0x0E    
#define  DOSE_F      0x0F



// ---------------------- FAULT & STATUS   CONFIGURATION ---------------------------- //


#define UART1_BAUDRATE             625000        // U1 Baud Rate

#define MODBUS_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_2STOPBITS)
#define MODBUS_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define MODBUS_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)

#define UART1TX_ON_TRIS		(TRISDbits.TRISD7)
#define UART1TX_ON_IO		(PORTDbits.RD7)

///*
//  --- Timer1 Setup ---
//   Used to measure the PRF
//   With 10Mhz Clock, x64 multiplier will yield max period of 419mS, 6.4 uS per Tick
//*/
//#define A36772_T1CON_VALUE     (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT)
//#define A36772_PR1_VALUE_US    400000   // 400ms
//#define A36772_PR1_VALUE       ((FCY_CLK/1000000)*A36772_PR1_VALUE_US/64)



#define CRC_POLY 0xA001				// Reverse CR16 polynomial





#endif	/* A37438_H */

