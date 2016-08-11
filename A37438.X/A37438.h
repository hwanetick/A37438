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
  Period of 1S
*/
#define A36772_T3CON_VALUE     (T3_ON & T3_IDLE_CON & T3_GATE_OFF & T3_PS_1_256 & T3_SOURCE_INT)
#define A36772_PR3_VALUE_US    1000000   // 1s
#define A36772_PR3_VALUE       ((FCY_CLK/1000000)*A36772_PR3_VALUE_US/256)

 
// ---- Hard Coded Delays ---- //
#define DELAY_FPGA_CABLE_DELAY 10

// ---------------------- Converter Logic Board Interface Control ------------------------- //
//#define WATCHDOG_HIGH                                48000
//#define WATCHDOG_LOW                                 16000

#define WATCHDOG_MODE_0         10
#define WATCHDOG_MODE_1         20
#define WATCHDOG_VALUE_0        0x3201   // 12801 - Allowed range from 8706 to 16896   //0x1000
#define WATCHDOG_VALUE_1        0xCE3E   // 52798 - Allowed range from 48703 to 56893  //0xEFF0

#define WATCHDOG_MAX_COUNT      80                           //800ms
   
#define MIN_WD_VALUE_0          0x2202
#define MAX_WD_VALUE_0          0x4200
#define MIN_WD_VALUE_1          0xBE3F  //0xBFF0
#define MAX_WD_VALUE_1          0xDE3D

#define DAC_DIGITAL_OFF                              0x0000
#define DAC_DIGITAL_ON                               0xFFFF

#define ADC_DATA_DIGITAL_HIGH                        0x0800

#define TARGET_CUSTOMER_HARDWARE_REV                 0b000100
#define TARGET_FPGA_FIRMWARE_MAJOR_REV               0b0001
#define TARGET_FPGA_FIRMWARE_MINOR_REV               0b000010
  
// MAX1230 Control Words
#define MAX1230_CONVERSION_BYTE                      0b10000011
#define MAX1230_SETUP_BYTE                           0b01101000
#define MAX1230_AVERAGE_BYTE                         0b00111000
#define MAX1230_RESET_BYTE                           0b00010000




typedef struct {
  //unsigned int watchdog_count_error;          // 
  unsigned int control_state;                   // This stores the state of the state machine
  unsigned int request_hv_enable;               // This indicates that hv_enable has been requested (either from CAN module or from discrete inputs depending upon configuration)
  unsigned int request_beam_enable;             // This indicates that beam_enable has been requested (either from CAN module or from discrete inputs depending upon configuration)
  unsigned int reset_active;                    // This indicates that reset has been requested (either from CAN module or from discrete inputs depending upon configuration)

  unsigned int heater_start_up_attempts;        // This counts the number of times the heater has started up without successfully completing it's ramp up.

  unsigned int run_time_counter;                // This counts how long the unit has been running for.  It wraps every 11 minutes
  unsigned int fault_restart_remaining;         // This counts down the delay of the heater automatic restart
  unsigned int power_supply_startup_remaining;  // This counts down the ramp up time of the HV supply
  unsigned int heater_warm_up_time_remaining;   // This counts down the heater warm up
  unsigned int heater_ramp_up_time;             // This counts the time it takes the heater to ramp up
  unsigned int watchdog_counter;                // This counts when to updated the watchdog DAC output on the converter logic board
  unsigned int watchdog_state_change;           // This flag is so the DAC isn't rewritten to for at least 80 ms
  unsigned int watchdog_set_mode;               // This is the DAC/ADC test setting for the SPI watchdog
  unsigned int heater_ramp_interval;            // This counts the interval between heater ramp voltage changes
  unsigned int heater_voltage_target;           // This is the targeted heater voltage set point
  unsigned int fault_holdoff_state;             // This is whether to hold off current limit fault during htr warmup period
  unsigned int fault_holdoff_count;             // This is a counter for the current limit fault holdoff
  
  volatile unsigned char control_config;        // This indicates when all set values from the CAN interface have been received

  unsigned int state_message;                   // This is a state message for the modbus module
  unsigned int current_state_msg;               // This stores the preliminary state message

  unsigned int can_high_voltage_set_point;      // This is the high voltage set point set over the can interface (it is only used if can mode is selected)
  unsigned int can_pulse_top_set_point;         // This is the pulse top set point set over the can interface (it is only used if can mode is selected)
  unsigned int can_heater_voltage_set_point;    // This is the heater voltage set point set over the can interface (it is only used if can mode is selected)


  unsigned int accumulator_counter;             // This counts the number of converstion on the internal ADC (used for averaging)
  unsigned int adc_read_error_count;            // This counts the total number of errors on reads from the adc on the converter logic board
  unsigned int adc_read_error_test;             // This increments when there is an adc read error and decrements when there is not.  If it exceeds a certain value a fault is generated
  unsigned int adc_read_ok;                     // This indicates if the previous adc read was successful or not

  unsigned int dac_write_error_count;           // This counts the total number of dac write errors
  unsigned int dac_write_failure_count;         // This counts the total number of unsessful dac transmissions (After N write errors it gives us)
  unsigned int dac_write_failure;               // This indicates that the previous attempt to write to the dac failed

  unsigned int heater_voltage_current_limited;  // This counter is used to track how long the heater is opperating in current limited mode. 
  unsigned int previous_state_pin_customer_hv_on;  // This stores the previous state of customer HV on input.  An On -> Off transion of this pin is used to generate a reset in discrete control mode
  
  unsigned int last_period;
  unsigned int trigger_complete;
  unsigned int this_pulse_level_energy_command;
  unsigned int next_pulse_level_energy_command;
  
  unsigned int trigger_received;
  
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
  
  
  // These are the Data Structures for the DAC outputs on the converter logic board
  AnalogOutput analog_output_high_voltage;
  AnalogOutput analog_output_top_voltage;
  AnalogOutput analog_output_heater_voltage;
  unsigned int dac_digital_hv_enable;
  unsigned int dac_digital_heater_enable;
  unsigned int dac_digital_top_enable;
  unsigned int dac_digital_trigger_enable;
  unsigned int dac_digital_watchdog_oscillator; //

  // These are the Data Structures for the on board DAC outputs
  AnalogOutput monitor_heater_voltage;
  AnalogOutput monitor_heater_current;
  AnalogOutput monitor_cathode_voltage;
  AnalogOutput monitor_grid_voltage;
  


  // These are the Data Structures for the Digital Data from the FPGA on the Converter Logic board
  TYPE_DIGITAL_INPUT fpga_coverter_logic_pcb_rev_mismatch;
  TYPE_DIGITAL_INPUT fpga_firmware_major_rev_mismatch;
  TYPE_DIGITAL_INPUT fpga_firmware_minor_rev_mismatch;
  TYPE_DIGITAL_INPUT fpga_arc;
  TYPE_DIGITAL_INPUT fpga_arc_high_voltage_inihibit_active;
  TYPE_DIGITAL_INPUT fpga_heater_voltage_less_than_4_5_volts;
  TYPE_DIGITAL_INPUT fpga_module_temp_greater_than_65_C;
  TYPE_DIGITAL_INPUT fpga_module_temp_greater_than_75_C;
  TYPE_DIGITAL_INPUT fpga_pulse_width_limiting_active;
  TYPE_DIGITAL_INPUT fpga_prf_fault;
  TYPE_DIGITAL_INPUT fpga_current_monitor_pulse_width_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_hardware_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_over_voltage_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_under_voltage_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_bias_voltage_fault;
  TYPE_DIGITAL_INPUT fpga_hv_regulation_warning;
  TYPE_DIGITAL_INPUT fpga_dipswitch_1_on;
  TYPE_DIGITAL_INPUT fpga_test_mode_toggle_switch_set_to_test;
  TYPE_DIGITAL_INPUT fpga_local_mode_toggle_switch_set_to_local;


  // These are Data Structures for the ADC input from the converter logic board
  AnalogInput  input_adc_temperature;
  AnalogInput  input_hv_v_mon;
  AnalogInput  input_hv_i_mon;
  AnalogInput  input_gun_i_peak;
  AnalogInput  input_htr_v_mon;
  AnalogInput  input_htr_i_mon;
  AnalogInput  input_top_v_mon;
  AnalogInput  input_bias_v_mon;
  AnalogInput  input_24_v_mon;
  AnalogInput  input_temperature_mon;
  TYPE_DIGITAL_INPUT adc_digital_warmup_flt;
  TYPE_DIGITAL_INPUT adc_digital_watchdog_flt;
  TYPE_DIGITAL_INPUT adc_digital_arc_flt;
  TYPE_DIGITAL_INPUT adc_digital_over_temp_flt;
  TYPE_DIGITAL_INPUT adc_digital_pulse_width_duty_flt;
  TYPE_DIGITAL_INPUT adc_digital_grid_flt;
  AnalogInput  input_dac_monitor;

  // These are the anlog input from the PICs internal DAC
  AnalogInput  pot_htr;     // an3
  AnalogInput  pot_vtop;    // an4
  AnalogInput  pot_ek;      // an5
  AnalogInput  ref_htr;     // an6
  AnalogInput  ref_vtop;    // an7
  AnalogInput  ref_ek;      // an13
  AnalogInput  pos_15v_mon; // an14
  AnalogInput  neg_15v_mon; // an15
  
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




#define _FAULT_FPGA_FIRMWARE_MAJOR_REV_MISMATCH        _FAULT_0 // CHECKED_DP// Heater Fault
#define _FAULT_ADC_HV_V_MON_OVER_RELATIVE              _FAULT_1 // CHECKED_DP
#define _FAULT_ADC_HV_V_MON_UNDER_RELATIVE             _FAULT_1 // CHECKED_DP
#define _FAULT_ADC_HTR_V_MON_OVER_RELATIVE             _FAULT_2 // CHECKED_DP// Heater Fault
#define _FAULT_ADC_HTR_V_MON_UNDER_RELATIVE            _FAULT_2 // CHECKED_DP// Heater Fault
#define _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE             _FAULT_3 // CHECKED_DP// Heater Fault
#define _FAULT_ADC_HTR_I_MON_UNDER_ABSOLUTE            _FAULT_4 // CHECKED_DP// Heater Fault
#define _FAULT_ADC_TOP_V_MON_OVER_RELATIVE             _FAULT_5 // CHECKED_DP
#define _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE            _FAULT_5 // CHECKED_DP
#define _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE            _FAULT_6 // CHECKED_DP 
#define _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE           _FAULT_6 // CHECKED_DP
#define _FAULT_CAN_COMMUNICATION                       _FAULT_7 // CHECKED_DP// Heater Fault
#define _FAULT_SPI_COMMUNICATION                       _FAULT_8
#define _FAULT_ADC_DIGITAL_ARC                         _FAULT_9  // CHECKED_DP// This requires HV OFF
#define _FAULT_ADC_DIGITAL_OVER_TEMP                   _FAULT_A  // CHECKED_DP// This requires a FPGA Reset (Goto Heater Off State)
//#define _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE        _FAULT_B // CHECKED_DP// Heater Fault
#define _FAULT_ADC_DIGITAL_GRID                        _FAULT_C  // CHECKED_DP// This requires a FPGA Reset (Goto Heater Off State)
#define _FAULT_HEATER_VOLTAGE_CURRENT_LIMITED          _FAULT_D  // CHECKED_DP// Heater Fault
#define _FAULT_HEATER_RAMP_TIMEOUT                     _FAULT_E  // CHECKED_DP// Heater Fault
#define _FAULT_HEATER_STARTUP_FAILURE                  _FAULT_F


#define _STATUS_CUSTOMER_HV_ON                         _WARNING_0
#define _STATUS_CUSTOMER_BEAM_ENABLE                   _WARNING_1
#define _STATUS_ADC_DIGITAL_HEATER_NOT_READY           _WARNING_2
#define _STATUS_DAC_WRITE_FAILURE                      _WARNING_3
//#define _STATUS_INTERLOCK_INHIBITING_HV                _WARNING_4
//#define _STATUS_HEATER_AT_OPERATING_CURRENT            _WARNING_5
//#define _FPGA_CUSTOMER_HARDWARE_REV_MISMATCH           _WARNING_6
#define _FPGA_FIRMWARE_MINOR_REV_MISMATCH              _WARNING_6
#define _FPGA_ARC_COUNTER_GREATER_ZERO                 _WARNING_7
#define _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE          _WARNING_7
//#define _FPGA_MODULE_TEMP_GREATER_THAN_65_C            _WARNING_8
#define _FPGA_MODULE_TEMP_GREATER_THAN_75_C            _WARNING_8
//#define _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT        _WARNING_9
#define _FPGA_GRID_MODULE_HARDWARE_FAULT               _WARNING_B
#define _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT           _WARNING_B
#define _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT          _WARNING_B
#define _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT           _WARNING_B
#define _FPGA_HV_REGULATION_WARNING                    _WARNING_C
#define _FPGA_DIPSWITCH_1_ON                           _WARNING_D
#define _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE        _WARNING_E
#define _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE      _WARNING_F
//#define _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS       _WARNING_9


#define ETM_CAN_REGISTER_GUN_DRIVER_RESET_FPGA        0x8202


#define STATE_FAULT_HEATER_FAILURE           00
#define STATE_FAULT_WARMUP_HEATER_OFF        10
#define STATE_FAULT_HEATER_OFF               20
#define STATE_START_UP                       30
#define STATE_WAIT_FOR_CONFIG                40
#define STATE_RESET_FPGA                     50
#define STATE_HEATER_RAMP_UP                 60
#define STATE_HEATER_WARM_UP                 70
#define STATE_FAULT_HEATER_ON                80
#define STATE_HEATER_WARM_UP_DONE            90
#define STATE_POWER_SUPPLY_RAMP_UP           100
#define STATE_HV_ON                          110
#define STATE_TOP_ON                         120
#define STATE_TOP_READY                      130
#define STATE_BEAM_ENABLE                    140


#define STATE_MESSAGE_FAULT_HEATER_OFF         0x0101
#define STATE_MESSAGE_START_UP                 0x0001
#define STATE_MESSAGE_HEATER_RAMP_UP           0x0003
#define STATE_MESSAGE_HEATER_WARM_UP           0x0007
#define STATE_MESSAGE_FAULT_HEATER_ON          0x010B
#define STATE_MESSAGE_HEATER_WARM_UP_DONE      0x000B
#define STATE_MESSAGE_HV_ON                    0x002B
#define STATE_MESSAGE_BEAM_ENABLE              0x00AB



#define UART1_BAUDRATE             625000        // U1 Baud Rate

#define MODBUS_U1MODE_VALUE        (UART_EN & UART_IDLE_STOP & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_2STOPBITS)
#define MODBUS_U1STA_VALUE         (UART_INT_TX & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS)
#define MODBUS_U1BRG_VALUE         (((FCY_CLK/UART1_BAUDRATE)/16)-1)

#define UART1TX_ON_TRIS		(TRISDbits.TRISD7)
#define UART1TX_ON_IO		(PORTDbits.RD7)

/*
  --- Timer1 Setup ---
   Used to measure the PRF
   With 10Mhz Clock, x64 multiplier will yield max period of 419mS, 6.4 uS per Tick
*/
#define A36772_T1CON_VALUE     (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT)
#define A36772_PR1_VALUE_US    400000   // 400ms
#define A36772_PR1_VALUE       ((FCY_CLK/1000000)*A36772_PR1_VALUE_US/64)

#define SLAVE_ADDRESS 0x07  //Slave address

#define CRC_POLY 0xA001				// Reverse CR16 polynomial


// Modbus states
#define MODBUS_STATE_IDLE           0x01
#define MODBUS_STATE_RECEIVING      0x02
#define MODBUS_STATE_PROCESSING     0x03
#define MODBUS_STATE_TRANSMITTING   0x04

// PLC slave address
#define MODBUS_SLAVE_ADDR       0x07

// Modbus exception codes
#define ILLEGAL_FUNCTION           0x01
#define ILLEGAL_ADDRESS            0x02
#define ILLEGAL_VALUE              0x03
#define DEVICE_FAILURE             0x04

//Other Errors
#define ETMMODBUS_ERROR_CRC          10
#define ETMMODBUS_ERROR_SLAVE_ADDR   20
#define ETMMODBUS_ERROR_FUNCTION     30

#define ETMMODBUS_COMMAND_OK         40


// Modbus functions
#define FUNCTION_READ_BITS              0x01
#define FUNCTION_READ_REGISTERS         0x03
#define FUNCTION_READ_INPUT_REGISTERS   0x04
#define FUNCTION_WRITE_BIT              0x05
#define FUNCTION_WRITE_REGISTER         0x06

#define EXCEPTION_FLAGGED               0x09
 
//#define ETMMODBUS_CMD_QUEUE_SIZE   16

typedef struct {
  unsigned char function_code;
  unsigned char received_function_code;
  unsigned char data_length_bytes;
  unsigned char exception_code;
  unsigned char done;
  unsigned int  output_address;
  unsigned int  data_address;
  unsigned int  qty_bits;
  unsigned int  qty_reg;
  unsigned int  write_value;
  unsigned int  data[125];
  unsigned char bit_data[125];
} MODBUS_MESSAGE;

//extern MODBUS_MESSAGE  current_command_ptr;

unsigned char modbus_cmd_byte[8];

#define ETMMODBUS_COMMAND_SIZE_MIN    8

#define SLAVE_BIT_ARRAY_SIZE          64
#define SLAVE_HOLD_REG_ARRAY_SIZE     64
#define SLAVE_INPUT_REG_ARRAY_SIZE    64

#define MODBUS_200ms_DELAY           20

unsigned int ETM_modbus_state;

unsigned int ModbusTimer;
unsigned int ModbusTest;

unsigned int ModbusSlaveHoldingRegister[SLAVE_HOLD_REG_ARRAY_SIZE];
unsigned int ModbusSlaveInputRegister[SLAVE_INPUT_REG_ARRAY_SIZE];
unsigned int ModbusSlaveBit[SLAVE_BIT_ARRAY_SIZE];




#endif	/* A37438_H */

