// Name: Alkinoos Sarioglou
// ID: 10136315
// Data Networking Assignment - YEAR 3

// Libraries to include
#include "main.h"
#include "Time_Delays.h"
#include "Clk_Config.h"

#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

#include "stm32f4xx_ll_crc.h"

/*
This program samples the accelerometer, stores it in a packet of a protocol, which also includes dest,src,length and checksum fields.
This packet can be written and retrieved from the external EEPROM memory.
*/

//Temperature Sensor I2C Address
#define TEMPADR 0x90

//EEPROM I2C WRITE Address
#define EEPROMADR 0xA0

//Accelerometer address
#define ACCELADR 0x98

// Data retrieved from a Single Byte EEPROM read
uint8_t eepromdata = 0;

// Variable to determine whether the EEPROM memory contents are displayed or the microcontroller memory contents
int viewing_eeprom = 0;

// Array to store the packet in MCU memory
uint8_t packet_byte[66] = {0};

// Array to store the packet retrieved from EEPROM memory
uint8_t eeprom_packet_byte[66] = {0};

// Sum of the whole packet (in 16-bit numbers)
uint16_t sum = 0;
uint16_t sum_recalculation = 0;

// Sum of the whole packet in EEPROM (in 16-bit numbers)
uint16_t eeprom_sum = 0;
uint16_t eeprom_sum_recalculation = 0;

// Checksum of the whole packet
uint16_t checksum_result = 0;
uint16_t checksum_recalculation = 0;

// Checksum of the whole packet in EEPROM
uint16_t eeprom_checksum_result = 0;
uint16_t eeprom_checksum_recalculation = 0;

// What is currently viewed on the LCD
enum field {none,dest,src,length,payload,checksum,checksum_check};
enum field displaying_field;


// Configure centre joystick
void config_centre_joystick(void) {
 LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB); // enable peripheral clock
 LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); // set B5 as Input
 LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); //set B5 as NO pull
}

// returns 1 if the joystick is pressed in the centre, or 0 otherwise
uint32_t pressed_joystick_centre(void) {
 return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_5));
}

// Configure left joystick
void config_left_joystick(void) {
 LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC); // enable peripheral clock
 LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); // set C1 as Input
 LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO); //set C1 as NO pull
}

// returns 1 if the joystick is pressed to the left, or 0 otherwise
uint32_t pressed_joystick_left(void) {
 return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_1));
}

// Configure right joystick
void config_right_joystick(void) {
 LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC); // enable peripheral clock
 LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set C0 as Input
 LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); //set C0 as NO pull
}

// returns 1 if the joystick is pressed to the right, or 0 otherwise
uint32_t pressed_joystick_right(void) {
 return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_0));
}

// Configure down joystick
void config_down_joystick(void) {
 LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB); // enable peripheral clock
 LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set B0 as Input
 LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); //set B0 as NO pull
}

// returns 1 if the joystick is pressed downwards, or 0 otherwise
uint32_t pressed_joystick_down(void) {
 return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_0));
}

// Configure up joystick
void config_up_joystick(void) {
 LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOA); // enable peripheral clock
 LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT); // set A4 as Input
 LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO); //set A4 as NO pull
}

// returns 1 if the joystick is pressed upwards, or 0 otherwise
uint32_t pressed_joystick_up(void) {
 return (LL_GPIO_IsInputPinSet (GPIOA, LL_GPIO_PIN_4));
}

// Configure the LCD
void config_lcd_screen(void) {
	
	Configure_LCD_Pins ();
	Configure_SPI1 ();
	Activate_SPI1 ();
	Clear_Screen ();
	Initialise_LCD_Controller ();
	set_font ((unsigned char*) Arial_12);

}

// Configure SDA (data) and SCL (clock) in I2C1 peripheral
void configure_sda_and_scl_pins (void) {
	
	// configure SCL as Alternate function, Open Drain, Pull Up:
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
	// configure SDA as: Alternate, Open Drain, Pull Up:
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
	
}

// Configure the I2C1 peripheral
void configure_i2c (void) {
	
	// Enable the I2C1 Peripheral:
	LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_I2C1);
	LL_I2C_Disable (I2C1); // disable I2C1 prior to configuration
	LL_I2C_SetMode (I2C1, LL_I2C_MODE_I2C);
	LL_I2C_ConfigSpeed (I2C1, 84000000, 100000, LL_I2C_DUTYCYCLE_2); // set speed to 100 kHz
	LL_I2C_Enable (I2C1); // re-enable I2C1	
	
}

// Set up the accelerometer on the Nucleo Board
void set_up_accelerometer(void) {
	
	//Sets accelerometer to active mode
	LL_I2C_GenerateStartCondition (I2C1); //START
	while (!LL_I2C_IsActiveFlag_SB(I2C1));
	
	LL_I2C_TransmitData8 (I2C1, ACCELADR); //ADDRESS + WRITE
	while (!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x07); //Set pointer register to mode register
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x01); //Set accelerometer to active mode
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
}

// Take a sample from the accelerometer 
uint8_t read_accelerometer_value (void) {
	LL_I2C_GenerateStartCondition (I2C1); //START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, ACCELADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x03); //Set pointer register to tilt register
	while (!LL_I2C_IsActiveFlag_TXE (I2C1));

	LL_I2C_GenerateStartCondition (I2C1); //RE-START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, ACCELADR+1); //ADDRESS + READ
	while (!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while (!LL_I2C_IsActiveFlag_RXNE (I2C1));
	
	packet_byte[2] = LL_I2C_ReceiveData8 (I2C1); // Data saved in the THIRD BYTE of the packet (First two are used for checksum)
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP

	return packet_byte[2]; //Returns sample
}

// Calculate checksum for the sample taken
void calculate_checksum (uint16_t tiltregister) {
	// SUM OF ALL PREVIOUS 16-BIT NUMBERS IN THE FIELD EQUALS 0x558B = 0xCCCC + 0xCCCC + 0xDDDD + 0xDDDD + 0x0036 + 0x0000(other payload bytes)
	sum = 0x558B + tiltregister; // Sum all the 16-bit numbers in the packet
	checksum_result = 0xFFFF - sum; // Invert bits in the sum
	packet_byte[0] = (uint8_t) checksum_result; // Save first byte of checksum in memory - BYTE 0
	packet_byte[1] = (uint8_t) (checksum_result >> 8); // Save second byte of checksum in memory - BYTE 1
}

// Calculate checksum in EEPROM
void calculate_eeprom_checksum (void) {
	// SUM OF ALL PREVIOUS 16-BIT NUMBERS IN THE FIELD EQUALS 0x558B = 0xCCCC + 0xCCCC + 0xDDDD + 0xDDDD + 0x0036 + 0x0000(other payload bytes)
	eeprom_sum = 0x558B + eeprom_packet_byte[2]; // Sum of all 16-bit numbers in the packet saved
	eeprom_checksum_result = 0xFFFF - eeprom_sum; // Invert bits in the sum
}

// Display the sample in a binary format
void display_payload(uint8_t sample) {
	char outputString[18]; //Buffer to store text in for LCD
	char data_bin[8] = {0};
	//Converts 8 bits into array for display purposes:
	for(int j = 0; j < 8; j++){
	 data_bin[j] = (sample & (0x80 >> j)) > 0;
	}
	put_string(0,0,"Tilt Reg");
	sprintf (outputString,"%x%x%x%x%x%x%x%x", data_bin[0], data_bin[1], data_bin[2], data_bin[3], data_bin[4], data_bin[5], data_bin[6], data_bin[7]); //  Add bits to a string
	put_string (0,15,outputString); // Output to LCD
	
}

// Display checksum result on screen
void display_checksum(void) {
	char outputString1[20]; //Buffer to store text in for LCD
	put_string(0,0,"Checksum:");
	sprintf (outputString1,"%x",checksum_result);
	put_string (0,15,outputString1);
	
}

// Display the checksum result from EEPROM
void display_eeprom_checksum(void) {
	char outputString1[20]; //Buffer to store text in for LCD
	put_string(0,0,"Checksum:");
	sprintf (outputString1,"%x%x",eeprom_packet_byte[1],eeprom_packet_byte[0]); // Display checksum field from EEPROM - BYTES 0 and 1
	put_string (0,15,outputString1);
	
}

// Display the check for integrity in Checksum
void display_checksum_check (void) {
	char outputString2[20]; //Buffer to store text in for LCD
	sum_recalculation = 0x558B + packet_byte[2]; // Sum of the entire packet recalculation
	checksum_recalculation = 0xFFFF - sum_recalculation; // Inverse of the sum recalculation
	if (checksum_recalculation == checksum_result) { // Check for integrity
		put_string(0,0,"Checksum:OK"); // Display OK message
	}
	else { // For invalid case
		put_string(0,0,"Checksum:ERROR"); // Display ERROR message
	}
	sprintf(outputString2,"%x",checksum_result); // Display checksum saved in the packet
	put_string (0,15,outputString2);
	
}

// Display the check for integrity in Checksum from EEPROM
void display_eeprom_checksum_check (void) {
	char outputString2[20]; //Buffer to store text in for LCD
	eeprom_sum_recalculation = 0x558B + eeprom_packet_byte[2]; // Sum of the entire packet recalculation
	eeprom_checksum_recalculation = 0xFFFF - eeprom_sum_recalculation; // Inverse of the sum recalculation
	if (eeprom_checksum_recalculation == eeprom_checksum_result) { // Check for integrity
		put_string(0,0,"Checksum:OK"); // Display OK message
	}
	else {
		put_string(0,0,"Checksum:ERROR"); // Display ERROR message
	}
	sprintf(outputString2,"%x",eeprom_checksum_result); // Display checksum saved in the packet
	put_string (0,15,outputString2);
	
}

//Display of dest field
void display_dest(void) {
	char outputString1[20]; //Buffer to store text in for LCD
	put_string(0,0,"dest:");
	sprintf (outputString1,"%x%x%x%x",packet_byte[65],packet_byte[64],packet_byte[63],packet_byte[62]); // Dest saved in BYTES 62-65 in MCU memory -- considering FIRST BYTE -> BYTE 0
	put_string (0,15,outputString1);
	
}

//Display of dest field from EEPROM
void display_eeprom_dest(void) {
	char outputString1[20]; //Buffer to store text in for LCD
	put_string(0,0,"dest:");
	sprintf (outputString1,"%x%x%x%x",eeprom_packet_byte[65],eeprom_packet_byte[64],eeprom_packet_byte[63],eeprom_packet_byte[62]); // Dest saved in BYTES 62-65 in EEPROM memory
	put_string (0,15,outputString1);
	
}

//Display of src field
void display_src(void) {
	char outputString2[20]; //Buffer to store text in for LCD
	put_string(0,0,"src:");
	sprintf (outputString2,"%x%x%x%x",packet_byte[61],packet_byte[60],packet_byte[59],packet_byte[58]); // Dest saved in BYTES 58-61 in MCU memory
	put_string (0,15,outputString2);
	
}

//Display of src field from EEPROM
void display_eeprom_src(void) {
	char outputString2[20]; //Buffer to store text in for LCD
	put_string(0,0,"src:");
	sprintf (outputString2,"%x%x%x%x",eeprom_packet_byte[61],eeprom_packet_byte[60],eeprom_packet_byte[59],eeprom_packet_byte[58]); // Dest saved in BYTES 58-61 in EEPROM memory
	put_string (0,15,outputString2);
	
}

//Display of length field
void display_length(void) {
	char outputString3[20]; //Buffer to store text in for LCD
	put_string(0,0,"length:");
	sprintf (outputString3,"%x%x",packet_byte[57],packet_byte[56]); // Dest saved in BYTES 56-57 in MCU memory
	put_string (0,15,outputString3);
	
}

//Display of length field from EEPROM
void display_eeprom_length(void) {
	char outputString3[20]; //Buffer to store text in for LCD
	put_string(0,0,"length:");
	sprintf (outputString3,"%x%x",eeprom_packet_byte[57],eeprom_packet_byte[56]); // Dest saved in BYTES 56-57 in EEPROM memory
	put_string (0,15,outputString3);
	
}

// Single Byte Write to EEPROM
void single_write_tilt_byte_to_eeprom(uint8_t sample) {
	
	LL_I2C_GenerateStartCondition (I2C1); //START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, EEPROMADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address High Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address Low Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, sample); // Tilt Register Data
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
}

// Page Write of Packet to EEPROM
void page_write_tilt_byte_to_eeprom(uint8_t checksum_byte_1, uint8_t checksum_byte_2, uint8_t sample) {
	
	LL_I2C_GenerateStartCondition (I2C1); //START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, EEPROMADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address High Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address Low Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, checksum_byte_1); // Checksum Write - BYTE 0
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, checksum_byte_2); // Checksum Write - BYTE 1
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, sample); // Payload Write - BYTE 0  (Tilt Register Data) [BYTE 2 of the packet]
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	for (int i=1; i<30; i++) {
		LL_I2C_TransmitData8 (I2C1, 0x00); // Payload Write - BYTES 1-29 [BYTES 3-31 of the packet]
		while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	}
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
	// End of first page
	
	// Acknowledge Polling

	do {
		
		// Clear the ACK Failure Flag
		LL_I2C_ClearFlag_AF(I2C1);
		
		LL_I2C_GenerateStartCondition (I2C1); //START
		while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
		LL_I2C_TransmitData8 (I2C1, EEPROMADR); //ADDRESS + WRITE
		
		LL_mDelay(150);
		
	} while (LL_I2C_IsActiveFlag_AF(I2C1)); // Repeat the loop until EEPROM returns ACK
	
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1)); // Checking if the device acknowledged the address
	LL_I2C_ClearFlag_ADDR (I2C1); // Clear the ADDR flag
	
	// End of Acknowledge Polling
	
	// Second Page Write
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address High Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x20); // Address Low Byte (Second Page)
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	for (int k=0; k<24; k++) {
		LL_I2C_TransmitData8 (I2C1, 0x00); // Payload Write - BYTES 30-53 [BYTES 32-55 of the packet]
		while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	}
	
	LL_I2C_TransmitData8 (I2C1, 0x36); // Length Write - FIRST BYTE [BYTE 56 of the packet]
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Length Write - SECOND BYTE [BYTE 57 of the packet]
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	for (int l=0; l<4; l++) {
		LL_I2C_TransmitData8 (I2C1, 0xDD); // SRC Write - BYTES 0-3 [BYTES 58-61 of the packet]
		while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	}
	
	for (int n=0; n<2; n++) {
		LL_I2C_TransmitData8 (I2C1, 0xCC); // DEST Write - BYTES 0-1 [BYTES 62-63 of the packet]
		while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	}
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
	// End of Second Page
	
	// Acknowledge Polling

	do {
		
		// Clear the ACK Failure Flag
		LL_I2C_ClearFlag_AF(I2C1);
		
		LL_I2C_GenerateStartCondition (I2C1); //START
		while (!LL_I2C_IsActiveFlag_SB (I2C1));
	
		LL_I2C_TransmitData8 (I2C1, EEPROMADR); //ADDRESS + WRITE
		
		LL_mDelay(150);
		
	} while (LL_I2C_IsActiveFlag_AF(I2C1)); // Repeat the loop until EEPROM returns ACK
	
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));  // Checking if the device acknowledged the address
	LL_I2C_ClearFlag_ADDR (I2C1); // Clear the ADDR flag
	
	// End of Acknowledge Polling
	
	// Third Page
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address High Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x40); // Address Low Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	for (int j=0; j<2; j++) {
		LL_I2C_TransmitData8 (I2C1, 0xCC); // DEST Write - BYTES 2-3 [BYTES 64-65 of the packet] -- considering the first byte as BYTE 0
		while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	}
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
	// End of third page
	
}


// Single Byte Read from EEPROM
uint8_t read_from_eeprom(void) {
	
	LL_I2C_GenerateStartCondition (I2C1); //START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));

	LL_I2C_TransmitData8 (I2C1, EEPROMADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address High Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x02); // Address Low Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));

	LL_I2C_GenerateStartCondition (I2C1); //RE-START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));

	LL_I2C_TransmitData8 (I2C1, EEPROMADR+1); //ADDRESS + READ
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while (!LL_I2C_IsActiveFlag_RXNE (I2C1));
	
	eepromdata = LL_I2C_ReceiveData8 (I2C1); //DATA BYTE
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
	return eepromdata;
}

// Page Mode Read from EEPROM
void packet_page_read_from_eeprom(void) {
	
	LL_I2C_GenerateStartCondition (I2C1); //START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));

	LL_I2C_TransmitData8 (I2C1, EEPROMADR); //ADDRESS + WRITE
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address High Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));
	
	LL_I2C_TransmitData8 (I2C1, 0x00); // Address Low Byte
	while(!LL_I2C_IsActiveFlag_TXE (I2C1));

	LL_I2C_GenerateStartCondition (I2C1); //RE-START
	while (!LL_I2C_IsActiveFlag_SB (I2C1));

	LL_I2C_TransmitData8 (I2C1, EEPROMADR+1); //ADDRESS + READ
	while(!LL_I2C_IsActiveFlag_ADDR (I2C1));
	LL_I2C_ClearFlag_ADDR (I2C1);
	
	for (int m=0; m<65; m++) { // READ BYTES 0-64
		
		LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); //ACK INCOMING DATA
		while (!LL_I2C_IsActiveFlag_RXNE (I2C1));
	
		eeprom_packet_byte[m] = LL_I2C_ReceiveData8 (I2C1); //DATA BYTE
		
	}
	
	LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while (!LL_I2C_IsActiveFlag_RXNE (I2C1));
	
	eeprom_packet_byte[65] = LL_I2C_ReceiveData8 (I2C1); // READ BYTE 65
	
	LL_I2C_GenerateStopCondition (I2C1); //STOP
	
}

// START OF MAIN

int main(void){
	
	//Initialise
  SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);	
	
	//Configure joystick buttons
	config_centre_joystick();
	config_left_joystick();
	config_right_joystick();
	config_down_joystick();
	config_up_joystick();
	
	//Configure the LCD screen
	config_lcd_screen();
	
	//Configure SDA and SCL pins
	configure_sda_and_scl_pins();
	
	//Configure I2C
	configure_i2c();
	
	//Set up accelerometer
	set_up_accelerometer();
	
	// Dest field specification
	packet_byte[65]=0xcc;
	packet_byte[64]=0xcc;
	packet_byte[63]=0xcc;
	packet_byte[62]=0xcc;
	
	// Src field specification
	packet_byte[61]=0xdd;
	packet_byte[60]=0xdd;
	packet_byte[59]=0xdd;
	packet_byte[58]=0xdd;
	
	// Length field specification
	packet_byte[57]=0x00;
	packet_byte[56]=0x36;
	
	while (1) {
			
		if (pressed_joystick_centre()) { // Centre button press
			
			viewing_eeprom = 0; // Not viewing EEPROM
			Clear_Screen();
			read_accelerometer_value(); // Take sample of accelerometer
			put_string(0,0,"Sampled"); // Display message for completion
			calculate_checksum(packet_byte[2]); // Checksum with sample taken
			LL_mDelay(500000);
			Clear_Screen();
			display_payload(packet_byte[2]); // Display sample
			displaying_field = payload; // Payload being displayed
			
		}
		
		else if (pressed_joystick_right()) { // Right button press
			
			Clear_Screen();
			page_write_tilt_byte_to_eeprom(packet_byte[0],packet_byte[1],packet_byte[2]); // Page Write to EEPROM
			put_string(0,0,"Packet Write"); // Display message for completion
			LL_mDelay(500000);
			Clear_Screen();
			display_payload(packet_byte[2]); // Display sample
			displaying_field = payload; // Payload being displayed
			
		}
		
		else if (pressed_joystick_left()) { // Left button press
			
			viewing_eeprom = 1; // Display of EEPROM contents 
			Clear_Screen();
			packet_page_read_from_eeprom(); // Page Read from EEPROM
			put_string(0,0,"Packet Read"); // Display message for completion
			LL_mDelay(500000);
			Clear_Screen();
			display_payload(eeprom_packet_byte[2]); // Display packet saved in EEPROM
			displaying_field = payload; // Payload being displayed
			
		}
		
		else if (pressed_joystick_down()) { // Down button press
			
			Clear_Screen();
			LL_mDelay(100000);

			if (viewing_eeprom == 0) { // Displaying current sample contents - NOT EEPROM contents
				if (displaying_field == dest) { // Dest -> Src
					display_src();
					displaying_field = src;
				}
				
				else if (displaying_field == none) { // If blank, show Payload
					display_payload(packet_byte[2]);
					displaying_field = payload;
				}
				
				else if (displaying_field == src) { // Src -> Length
					display_length();
					displaying_field = length;
				}
				
				else if (displaying_field == length) { // Length -> Payload
					display_payload(packet_byte[2]);
					displaying_field = payload;
				}
				
				else if (displaying_field == payload) { // Payload -> Checksum
					display_checksum();
					displaying_field = checksum;
				}
				
				else if (displaying_field == checksum) { // Checksum -> Checksum check
					display_checksum_check();
					displaying_field = checksum_check;
				}
				
				else if (displaying_field == checksum_check) { // Checksum check stays ON
					display_checksum_check();
					displaying_field = checksum_check;
					
				}
			}
			
			else if (viewing_eeprom == 1) { // Displaying EEPROM contents
				
				if (displaying_field == dest) { // Dest -> Src
					display_eeprom_src();
					displaying_field = src;
				}
				
				else if (displaying_field == none) { // If blank, show Payload
					display_payload(eeprom_packet_byte[2]);
					displaying_field = payload;
				}
				
				else if (displaying_field == src) { // Src -> Length
					display_eeprom_length();
					displaying_field = length;
				}
				
				else if (displaying_field == length) { // Length -> Payload
					display_payload(eeprom_packet_byte[2]);
					displaying_field = payload;
				}
				
				else if (displaying_field == payload) { // Payload -> Checksum
					display_eeprom_checksum();
					displaying_field = checksum;
				}
				
				else if (displaying_field == checksum) { // Checksum -> Checksum check
					calculate_eeprom_checksum(); 
					display_eeprom_checksum_check();
					displaying_field = checksum_check;
				}
				
				else if (displaying_field == checksum_check) { // Checksum check stays ON
					calculate_eeprom_checksum();
					display_eeprom_checksum_check();
					displaying_field = checksum_check;
					
				}
			}
				
		}
		
		else if (pressed_joystick_up()) {  // Up button press
			
			Clear_Screen();
			LL_mDelay(100000);
			
			if (viewing_eeprom == 0) { // Displaying current sample contents - NOT EEPROM contents
				
				if (displaying_field == dest) { // Dest stays ON
					display_dest();
					displaying_field = dest;
				}
				
				else if (displaying_field == none) { // If blank, show Dest
					display_dest();
					displaying_field = dest;
				}
				
				else if (displaying_field == src) { // Src -> Dest
					display_dest();
					displaying_field = dest;
				}
				
				else if (displaying_field == length) { // Length -> Src
					display_src();
					displaying_field = src;
				}
				
				else if (displaying_field == payload) { // Payload -> Length
					display_length();
					displaying_field = length;
				}
				
				else if (displaying_field == checksum) { // Checksum -> Payload
					display_payload(packet_byte[2]);
					displaying_field = payload;
				}
				
				else if (displaying_field == checksum_check) { // Checksum check -> Checksum
					display_checksum();
					displaying_field = checksum;
					
				}
				
			}
			
			else if (viewing_eeprom == 1) { // Displaying EEPROM contents
				
				if (displaying_field == dest) {  // Dest stays ON
					display_eeprom_dest();
					displaying_field = dest;
				}
				
				else if (displaying_field == none) {  // If blank, show Dest
					display_eeprom_dest();
					displaying_field = dest;
				}
				
				else if (displaying_field == src) {  // Src -> Dest
					display_eeprom_dest();
					displaying_field = dest;
				}
				
				else if (displaying_field == length) {  // Length -> Src
					display_eeprom_src();
					displaying_field = src;
				}
				
				else if (displaying_field == payload) {  // Payload -> Length
					display_eeprom_length();
					displaying_field = length;
				}
				
				else if (displaying_field == checksum) { // Checksum -> Payload
					display_payload(eeprom_packet_byte[2]);
					displaying_field = payload;
				}
				
				else if (displaying_field == checksum_check) { // Checksum check -> Checksum 
					display_eeprom_checksum();
					displaying_field = checksum;
					
				}
				
			}
			
		}
		
	}

}

// END OF MAIN
