
// This is firmware for the Gun Driver Board


#include "A37438.h"
#include "A37438_CONFIG.h"

_FOSC(EC & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_64 & PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



void InitializeA36772(void); // Initialize the A36772 for operation
void DoStartupLEDs(void); // Used to flash the LEDs at startup

// Helper functions for DoA36772
void DoA36772(void);
/*
  DoA36772 is called every time the processor cycles throuh it's control loop
  If _T2IF is set (indicateds 10mS has passed) it executes everything that happens on 10mS time scale
*/


static unsigned char  ETMmodbus_put_index;
static unsigned char  ETMmodbus_get_index;


BUFFERBYTE64 uart1_input_buffer;   
BUFFERBYTE64 uart1_output_buffer; 

unsigned int checkCRC(unsigned char * ptr, unsigned int size);


// -------------------------- GLOBAL VARIABLES --------------------------- //
TYPE_GLOBAL_DATA_A36772 global_data_A36772;
LTC265X U32_LTC2654;

const unsigned char Dose_Array[16] = {DOSE_LOOKUP_VALUES};
const unsigned int CRC_High_Energy[16] = {CRC_HIGH_ENERGY_LOOKUP_VALUES};
const unsigned int CRC_Low_Energy[16] = {CRC_LOW_ENERGY_LOOKUP_VALUES};


int main(void) {
  InitializeA36772();
  _CONTROL_NOT_CONFIGURED = 0;
  _CONTROL_NOT_READY = 1;
//  global_data_A36772.run_time_counter = 0;
//  _T2IF = 0;
//  while (global_data_A36772.run_time_counter < 300) {
//    DoStartupLEDs();
//    if (_T2IF) {
//      _T2IF = 0;
//      global_data_A36772.run_time_counter++;
//    } 
//  }
 
  while (1) {
    DoA36772();
  }
}


void InitializeA36772(void) {
 
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;

  
  global_data_A36772.message1_energy = 0x00;
  global_data_A36772.message2_blank = 0x00;
  global_data_A36772.message3_blank = 0x00;
  PIN_RS485_ENABLE = 1;
  global_data_A36772.dose_switch_value = 0;
  global_data_A36772.trigger_received = 0;
  
  
  // --------- BEGIN IO PIN CONFIGURATION ------------------
  
    // Initialize Ouput Pin Latches BEFORE setting the pins to Output
  PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
	  
	  
  // ---- Configure the dsPIC ADC Module Analog Inputs------------ //
  ADPCFG = 0xFFFF;             // all are digital I/O
 
  // Initialize all I/O Registers
  TRISA = A36772_TRISA_VALUE;
  TRISB = A36772_TRISB_VALUE;
  TRISC = A36772_TRISC_VALUE;
  TRISD = A36772_TRISD_VALUE;
  TRISF = A36772_TRISF_VALUE;
  TRISG = A36772_TRISG_VALUE;

//  // Config SPI1 for Gun Driver
  ConfigureSPI(ETM_SPI_PORT_1, A36772_SPI1CON_VALUE, 0, A36772_SPI1STAT_VALUE, SPI_CLK_1_MBIT, FCY_CLK);  
//  

  	  // Initialize application specific hardware
  UART1TX_ON_TRIS = 0;
  UART1TX_ON_IO = 1;    // always enable TX1

    // Configure UART Interrupts
  _U1RXIE = 0;
  _U1RXIP = 5;
  
  _U1TXIE = 0;
  _U1TXIP = 5;

  
    
  // Set up external INT3 */
  // This is the trigger interrupt
  _INT3IF = 0;		// Clear Interrupt flag
  _INT3IE = 1;		// Enable INT3 Interrupt
  _INT3EP = 1; 	        // Interrupt on falling edge
  _INT3IP = 7;		// Set interrupt to highest priority
      
  
  // ---------- Configure Timers ----------------- //
//          // Initialize TMR1
//  PR1   = A36772_PR1_VALUE;
//  TMR1  = 0;
//  _T1IF = 0;
//  _T1IP = 2;
//  T1CON = A36772_T1CON_VALUE;

  // Initialize TMR2
  PR2   = A36772_PR2_VALUE;
  TMR2  = 0;
  _T2IF = 0;
//  _T2IP = 5;
  _T2IP = 2;
  T2CON = A36772_T2CON_VALUE;
  
//      // Initialize TMR3
//  PR3   = A36772_PR3_VALUE;
//  TMR3  = 0;
//  _T3IF = 0;
////  _T3IP = 5;
//  _T3IP = 2;
//  T3CON = A36772_T3CON_VALUE;
  
  

//  // Configure on-board DAC
  SetupLTC265X(&U32_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);
//
//  //Configure EEPROM
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);
//
//  // ------------- Configure Internal ADC --------- //
//  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
//  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
//  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
//  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
//  
//  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
//  ADCSSL = ADCSSL_SETTING;             // Set which analog pins are scanned
//
//  _ADIF = 0;
//  _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
//  _ADIE = 1;
//  _ADON = 1;
//  
  
  // ----------------- UART #1 Setup and Data Buffer -------------------------//
  // Setup the UART input and output buffers
  uart1_input_buffer.write_location = 0;  
  uart1_input_buffer.read_location = 0;
  uart1_output_buffer.write_location = 0;
  uart1_output_buffer.read_location = 0;
           
  ETMmodbus_put_index = 0;
  ETMmodbus_get_index = 0;
  
  U1MODE = MODBUS_U1MODE_VALUE;
  U1BRG = MODBUS_U1BRG_VALUE;
  U1STA = MODBUS_U1STA_VALUE;
  
  _U1TXIF = 0;	// Clear the Transmit Interrupt Flag
  _U1TXIE = 1;	// Enable Transmit Interrupts
  _U1RXIF = 0;	// Clear the Recieve Interrupt Flag
  _U1RXIE = 1;	// Enable Recieve Interrupts
  
  
  U1MODEbits.UARTEN = 1;	// And turn the peripheral on
  
  PIN_RS485_ENABLE = 1;
  
  
//#ifdef __CAN_ENABLED
  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_2, FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RC4, 4, _PIN_RC3, _PIN_RC3);
  ETMCanSlaveLoadConfiguration(36772, BOARD_DASH_NUMBER, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);
//#endif


  ETMDigitalInitializeInput(&global_data_A36772.switch_bit_0, 0, 50);
  ETMDigitalInitializeInput(&global_data_A36772.switch_bit_1, 0, 50);
  ETMDigitalInitializeInput(&global_data_A36772.switch_bit_2, 0, 50);
  ETMDigitalInitializeInput(&global_data_A36772.switch_bit_3, 0, 50);
  
  
   // Turn on switch bit pullups
  PIN_CPU_SWITCH_BIT0_ENABLE = OLL_STATUS_ACTIVE;
  PIN_CPU_SWITCH_BIT1_ENABLE = OLL_STATUS_ACTIVE;
  PIN_CPU_SWITCH_BIT2_ENABLE = OLL_STATUS_ACTIVE;
  PIN_CPU_SWITCH_BIT3_ENABLE = OLL_STATUS_ACTIVE;
 
}


void DoStartupLEDs(void) {
  switch (((global_data_A36772.run_time_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_LED_I2A = OLL_LED_ON;
    PIN_LED_I2B = !OLL_LED_ON;
    PIN_LED_I2C = !OLL_LED_ON;
    PIN_LED_I2D = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_LED_I2A = !OLL_LED_ON;
    PIN_LED_I2B = OLL_LED_ON;
    PIN_LED_I2C = !OLL_LED_ON;
    PIN_LED_I2D = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_LED_I2A = !OLL_LED_ON;
    PIN_LED_I2B = !OLL_LED_ON;
    PIN_LED_I2C = OLL_LED_ON;
    PIN_LED_I2D = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_LED_I2A = !OLL_LED_ON;
    PIN_LED_I2B = !OLL_LED_ON;
    PIN_LED_I2C = !OLL_LED_ON;
    PIN_LED_I2D = OLL_LED_ON;
    break;
  }
}


void DoA36772(void) {
  
//#ifdef __CAN_ENABLED
  ETMCanSlaveDoCan();
//#endif

//#ifndef __CAN_REQUIRED
  ClrWdt();
//#endif
  
  
  unsigned int crc_int;
  unsigned int crc_16_msb;

  if (global_data_A36772.trigger_received){
    global_data_A36772.trigger_received = 0;
    global_data_A36772.message1_energy ^= 0x01;
    global_data_A36772.message0_dose = Dose_Array[global_data_A36772.dose_switch_value];
    if (global_data_A36772.message1_energy) {
      crc_int = CRC_High_Energy[global_data_A36772.dose_switch_value];
    } else {
      crc_int = CRC_Low_Energy[global_data_A36772.dose_switch_value];
    }
    
    crc_16_msb = crc_int >> 8;
    global_data_A36772.message5_crc_high = (unsigned char)crc_16_msb & 0xff;
    global_data_A36772.message4_crc_low  = (unsigned char)crc_int & 0xff;     
    
    BufferByte64WriteByte(&uart1_output_buffer, global_data_A36772.message0_dose);
    BufferByte64WriteByte(&uart1_output_buffer, global_data_A36772.message1_energy);
    BufferByte64WriteByte(&uart1_output_buffer, global_data_A36772.message2_blank);
    BufferByte64WriteByte(&uart1_output_buffer, global_data_A36772.message3_blank);
    BufferByte64WriteByte(&uart1_output_buffer, global_data_A36772.message4_crc_low);
    BufferByte64WriteByte(&uart1_output_buffer, global_data_A36772.message5_crc_high);

    if (!U1STAbits.UTXBF) {
      U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
    }
  }
  

  if (_T2IF) {
    // Run once every 10ms
    _T2IF = 0;

    
    if (PIN_SW_BIT0_STATUS == ILL_PIN_SW_BIT0_ON) {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_0, 1);   
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_0, 0);
    }
  
    if (PIN_SW_BIT1_STATUS == ILL_PIN_SW_BIT1_ON) {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_1, 1);   
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_1, 0);
    }
  
    if (PIN_SW_BIT2_STATUS == ILL_PIN_SW_BIT1_ON) {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_2, 1);   
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_2, 0);
    }
  
    if (PIN_SW_BIT3_STATUS == ILL_PIN_SW_BIT1_ON) {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_3, 1);   
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.switch_bit_3, 0);
    }

  
  
    if (ETMDigitalFilteredOutput(&global_data_A36772.switch_bit_0)) {
      global_data_A36772.dose_switch_value |= 0x01;
    } else { 
      global_data_A36772.dose_switch_value &= ~0x01;
    }
  
    if (ETMDigitalFilteredOutput(&global_data_A36772.switch_bit_1)) {
      global_data_A36772.dose_switch_value |= 0x02;
    } else { 
      global_data_A36772.dose_switch_value &= ~0x02;
    }
  
    if (ETMDigitalFilteredOutput(&global_data_A36772.switch_bit_2)) {
      global_data_A36772.dose_switch_value |= 0x04;
    } else { 
      global_data_A36772.dose_switch_value &= ~0x04;
    }
  
    if (ETMDigitalFilteredOutput(&global_data_A36772.switch_bit_3)) {
      global_data_A36772.dose_switch_value |= 0x08;
    } else { 
      global_data_A36772.dose_switch_value &= ~0x08;
    }

    if (global_data_A36772.dose_switch_value > 15) {
      global_data_A36772.dose_switch_value = 0;
    }

    global_data_A36772.run_time_counter++;

    if (global_data_A36772.run_time_counter & 0x0010) {
      PIN_LED_OPERATIONAL = 1;
    } else {
      PIN_LED_OPERATIONAL = 0;
    }
 

    ETMCanSlaveSetDebugRegister(0xA, global_data_A36772.dose_switch_value);
    ETMCanSlaveSetDebugRegister(0xB, global_data_A36772.message0_dose);
    ETMCanSlaveSetDebugRegister(0xC, PIN_SW_BIT0_STATUS);
    ETMCanSlaveSetDebugRegister(0xD, PIN_SW_BIT1_STATUS);
    //ETMCanSlaveSetDebugRegister(0xE, global_data_A36772.ref_vtop.reading_scaled_and_calibrated);//
    //ETMCanSlaveSetDebugRegister(0xF, global_data_A36772.ref_ek.reading_scaled_and_calibrated);
//    ETMCanSlaveSetDebugRegister(0xA, global_data_A36772.monitor_grid_voltage.set_point); //run_time_counter);
//    ETMCanSlaveSetDebugRegister(0xB, global_data_A36772.monitor_grid_voltage.dac_setting_scaled_and_calibrated); //fault_restart_remaining);
//    ETMCanSlaveSetDebugRegister(0xC, global_data_A36772.ref_vtop.reading_scaled_and_calibrated); //power_supply_startup_remaining);
//    ETMCanSlaveSetDebugRegister(0xD, global_data_A36772.heater_warm_up_time_remaining);
//    ETMCanSlaveSetDebugRegister(0xE, global_data_A36772.heater_ramp_up_time);
    ETMCanSlaveSetDebugRegister(0xF, global_data_A36772.run_time_counter);
    
//    ETMCanSlaveSetDebugRegister(0xA, _FPGA_FIRMWARE_MINOR_REV_MISMATCH);
//    ETMCanSlaveSetDebugRegister(0xB, fpga_bits.fpga_firmware_minor_rev);
//    ETMCanSlaveSetDebugRegister(0xC, _WARNING_REGISTER);
    
//    
//    slave_board_data.log_data[0] = global_data_A36772.input_gun_i_peak.reading_scaled_and_calibrated;
//    slave_board_data.log_data[1] = global_data_A36772.input_hv_v_mon.reading_scaled_and_calibrated;
//    slave_board_data.log_data[2] = global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated;    //gdoc says low energy
//    slave_board_data.log_data[3] = global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated;    //gdoc says high energy
//    slave_board_data.log_data[4] = global_data_A36772.input_temperature_mon.reading_scaled_and_calibrated;
//    slave_board_data.log_data[5] = global_data_A36772.heater_warm_up_time_remaining;
//    slave_board_data.log_data[6] = global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated;
//    slave_board_data.log_data[7] = global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated;
//    slave_board_data.log_data[8] = global_data_A36772.analog_output_high_voltage.set_point;
//    slave_board_data.log_data[9] = global_data_A36772.heater_voltage_target;
//    slave_board_data.log_data[10] = global_data_A36772.analog_output_top_voltage.set_point;       //gdoc says low energy
//    slave_board_data.log_data[11] = global_data_A36772.analog_output_top_voltage.set_point;       //gdoc says high energy
//    slave_board_data.log_data[12] = global_data_A36772.input_bias_v_mon.reading_scaled_and_calibrated;
//    slave_board_data.log_data[13] = global_data_A36772.control_state;
//    slave_board_data.log_data[14] = global_data_A36772.adc_read_error_count;
//    slave_board_data.log_data[15] = GUN_DRIVER_LOAD_TYPE;



//    ETMCanSlaveSetDebugRegister(7, global_data_A36772.dac_write_failure_count);


    // Send out Data to local DAC and offboard.  Each channel will be updated once every 40mS
    // Do not send out while in state "STATE_WAIT_FOR_CONFIG" because the module is not ready to receive data and
    // you will just get data transfer errors
 
    
    // Turn on switch bit pullups

    PIN_CPU_SWITCH_BIT0_ENABLE = OLL_STATUS_ACTIVE;
    PIN_CPU_SWITCH_BIT1_ENABLE = OLL_STATUS_ACTIVE;
    PIN_CPU_SWITCH_BIT2_ENABLE = OLL_STATUS_ACTIVE;
    PIN_CPU_SWITCH_BIT3_ENABLE = OLL_STATUS_ACTIVE;
    
  }
}



void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;

}





//void ETMModbusSlaveDoModbus(void) {
//  if (!modbus_transmission_needed) {
//    if (LookForMessage()) {
//      //Execute command with following functions
//      PIN_RS485_ENABLE = 1;
//      ReceiveCommand(&current_command_ptr);
//      ProcessCommand(&current_command_ptr);
//      SendResponse(&current_command_ptr);
//      modbus_transmission_needed = 1;
////      while ((!U1STAbits.UTXBF) && (BufferByte64BytesInBuffer(&uart1_output_buffer))) {
////          U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
////      }
//      if (!U1STAbits.UTXBF) {
//        U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
//      }
//    }
//  } else if ((U1STAbits.TRMT == 1) && (!BufferByte64BytesInBuffer(&uart1_output_buffer))) {
//    PIN_RS485_ENABLE = 0;
//    modbus_transmission_needed = 0;
//  }   
//}


//-----------------------------------------------------------------------------
// CRC_Check
//-----------------------------------------------------------------------------
//
// Return Value : accum -- the end result of the CRC value.
// Parameter    : *ptr -- pointer to the array for CRC calculation.
//				: size -- size of the array
//
// This function calculates the 16-bit CRC value for the input array.
//
//-----------------------------------------------------------------------------
unsigned int checkCRC(unsigned char * ptr, unsigned int size)
{
    unsigned int i, j;
    unsigned int accum, element;

    accum = 0xffff;

    for (j = 0; j < size; j++)
    {
        element = ptr[j];

        for (i = 8; i > 0; i--)		
        {
            if (((element ^ accum) & 0x0001) > 0)
                accum = (unsigned int)((accum >> 1) ^ ((unsigned int)CRC_POLY));
            else
                accum >>= 1;

            element >>= 1;
        }
    }

    return (accum);
}


//-----------------------------------------------------------------------------
//   UART Interrupts
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
        
//void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
//  _U1RXIF = 0;
//
//  modbus_receiving_flag = 1;
//
//  while (U1STAbits.URXDA) {
//    BufferByte64WriteByte(&uart1_input_buffer, U1RXREG);
//  }
//  
//}


void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
  _U1TXIF = 0;
  while ((!U1STAbits.UTXBF) && (BufferByte64BytesInBuffer(&uart1_output_buffer))) {
    /*
      There is at least one byte available for writing in the output buffer and the transmit buffer is not full.
      Move a byte from the output buffer into the transmit buffer
    */
    U1TXREG = BufferByte64ReadByte(&uart1_output_buffer);
  }

}

//
//
#define MIN_PERIOD 150 // 960uS 1041 Hz// 
void __attribute__((interrupt(__save__(CORCON,SR)), no_auto_psv)) _INT3Interrupt(void) {
  // A trigger was recieved.
  // THIS DOES NOT MEAN THAT A PULSE WAS GENERATED
  // If (PIN_CPU_XRAY_ENABLE_OUT == OLL_CPU_XRAY_ENABLE)  && (PIN_CUSTOMER_XRAY_ON_IN == ILL_CUSTOMER_BEAM_ENABLE) then we "Probably" generated a pulse

    
  global_data_A36772.trigger_received = 1;
    
//  if ((TMR1 > MIN_PERIOD) || _T1IF) {
//    // Calculate the Trigger PRF
//    // TMR1 is used to time the time between INT3 interrupts
//    global_data_A36772.last_period = TMR1;
//    TMR1 = 0;
//    if (_T1IF) {
//      // The timer exceed it's period of 400mS - (Will happen if the PRF is less than 2.5Hz)
//      global_data_A36772.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
//    }
//    /*
//      if (ETMCanSlaveGetSyncMsgPulseSyncDisableHV() || ETMCanSlaveGetSyncMsgPulseSyncDisableXray()) {
//      // We are not pulsing so set the PRF to the minimum
//      global_data_A36772.last_period = 62501;  // This will indicate that the PRF is Less than 2.5Hz
//      }
//    */
//    _T1IF = 0;
//    
//    global_data_A36772.trigger_complete = 1;
//    global_data_A36772.this_pulse_level_energy_command = global_data_A36772.next_pulse_level_energy_command;
//    uart2_next_byte = 0;
//  }
  
  _INT3IF = 0;		// Clear Interrupt flag
}  





//  switch(global_data_A36772.dose_switch_value){ 
//    case DOSE_0:
//      message0_dose = 0x10;
//      break;
//    
//    case DOSE_1:
//      message0_dose = 0x20;
//      break;
//      
//    case DOSE_2:
//      message0_dose = 0x30;
//      break;
//      
//    case DOSE_3:
//      message0_dose = 0x40;
//      break;
//      
//    case DOSE_4:
//      message0_dose = 0x50;
//      break;
//      
//    case DOSE_5:
//      message0_dose = 0x60;
//      break;
//      
//    case DOSE_6:
//      message0_dose = 0x70;
//      break;
//      
//    case DOSE_7:
//      message0_dose = 0x80;
//      break;
//      
//    case DOSE_8:
//      message0_dose = 0x90;
//      break;
//      
//    case DOSE_9:
//      message0_dose = 0xA0;
//      break;
//      
//    case DOSE_A:
//      message0_dose = 0xB0;
//      break;
//      
//    case DOSE_B:
//      message0_dose = 0xC0;
//      break;
//      
//    case DOSE_C:
//      message0_dose = 0xD0;
//      break;
//      
//    case DOSE_D:
//      message0_dose = 0xE0;
//      break;
//      
//    case DOSE_E:
//      message0_dose = 0xF0;
//      break;
//      
//    case DOSE_F:
//      message0_dose = 0xFF;
//      break;
//      
//    default:
//      
//  } 