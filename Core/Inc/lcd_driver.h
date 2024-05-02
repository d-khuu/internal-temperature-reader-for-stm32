/*
 * lcd_driver.h
 *
 *  Created on: Mar 31, 2024
 *      Author: Danny
 *
 *      Device: LCD1602 16x2 LCD I2C Module
 *      I have no intention to include every feature of this LCD, I just want the
 *      basic functionality of this LCD screen.
 *
 *      In terms of operation times, all instructions take around 40 us, except for the
 *      return home and clear display operations which use 1.53 ms.
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define LCD_SLAVE_ADDRESS 0x3E << 1

#define LCD_MAX_CHAR_LENGTH 32

// I2C Register Addresses
// Main Address
#define LCD_CONTROL_BYTE	0x00
// CGRAM Address
#define LCD_CGRAM_ADDRESS	0x40
// DDRAM Address
#define LCD_DDRAM_ADDRESS	0x80


// Data Byte/Command
// Clears the display and returns the cursor to the beginning
#define LCD_CLEAR_DISPLAY 	0x01
// Return Cursor to the beginning
#define LCD_RETURN_HOME		0x02
// Entry Mode Set
#define	LCD_ENTRY_MODE_SET	0x04
// Display On/Off Control
#define LCD_DISPLAY_ONOFF_CONTROL	0x08
// Cursor or Display Shift
#define LCD_CURSOR_DISPLAY_SHIFT	0x10
// Function Set
#define	LCD_FUNCTION_SET	0x20


// Modifiers for Data Byte/Command

// Entry Mode Set Modifiers
// Increment DDRAM Address
#define LCD_INC_DDRAM	0x02
// Decrement DDRAM Address
#define LCD_DEC_DDRAM	0x00
// Shift Entire Display
#define	LCD_SHIFT_DISP	0x01
// Not Shift Entire Display
#define LCD_NO_SHIFT_DISP	0x00

// Display On/Off Control Modifier
// Display On Control Bit
#define LCD_DISP_CTRL_ON	0x04
// Display Off Control Bit
#define	LCD_DISP_CTRL_OFF	0x00
// Cursor On Control Bit
#define LCD_CURSOR_ON	0x02
// Cursor Off Control Bit
#define LCD_CURSOR_OFF	0x00
// Cursor Blink On Control Bit
#define LCD_CURSOR_BLINK_ON	0x01
// Cursor Blink Off Control Bit
#define LCD_CURSOR_BLINK_OFF	0x00

// Cursor or Display Shift Modifiers
// Shift cursor to the left, AC is decreased by 1
#define LCD_CURSOR_LEFT_DECREASE	0x00
// Shift cursor to the right, AC is increased by 1
#define LCD_CURSOR_RIGHT_INCREASE	0x04
// Shift all the display to the left, cursor moves according to the display
#define LCD_CURSOR_WITH_DISPLAY_LEFT	0x08
// Shift all the display to the right, cursor moves according to the display
#define LCD_CURSOR_WITH_DISPLAY_RIGHT	0x012

// Function Set
// 8-bit Bus Mode with MPU
#define LCD_DL_8BIT	0x10
// 4-bit Bus Mode with MPU
#define LCD_DL_4BIT	0x00
// 2-line Display Mode
#define	LCD_2_LINE	0x08
// 1-line Display Mode
#define	LCD_1_LINE	0x00
// 11 Dot Format Display Format
#define LCD_11_DOT	0x04
// 8 Dot Format Display Format
#define LCD_8_DOT	0x00

// DDRAM Miscellaneous Bits
#define LCD_ROW_1_FIRST_ADDRESS	0x00
#define LCD_ROW_2_FIRST_ADDRESS 0x40

// Struct Definitions
struct lcd_current_state
{
	int cursor_row;
	int cursor_col;

	char chars_on_screen[LCD_MAX_CHAR_LENGTH];

};

// Function Definitions

int LCD_Setup(I2C_HandleTypeDef* i2c_handler);

int LCD_Reset(I2C_HandleTypeDef* i2c_handler);

int LCD_Write_Char(I2C_HandleTypeDef* i2c_handler, char output_char);

int LCD_Write_String(I2C_HandleTypeDef* i2c_handler, char* output_string);

int LCD_Set_Cursor_Position(I2C_HandleTypeDef* i2c_handler, int row, int col);

int LCD_Return_Home(I2C_HandleTypeDef* i2c_handler);



int LCD_Show_Debug_Message(I2C_HandleTypeDef* i2c_handler, char* output_string);

// Wrapper functions for LCD Debugging Messages
// and returning the text back to the original state
int LCD_Write_String_Non_Debug(I2C_HandleTypeDef* i2c_handler, char* output_string);

int LCD_Set_Cursor_Position_Non_Debug(I2C_HandleTypeDef* i2c_handler, int row, int col);

int LCD_Initialise_State_Struct(I2C_HandleTypeDef* i2c_handler);
int __LCD_State_Update_Cursor_Position(int row, int col, int update_absolute_position);
int __LCD_State_Update_LCD_Screen(I2C_HandleTypeDef* i2c_handler, char output_char);
int __LCD_Redraw_Current_State(I2C_HandleTypeDef* i2c_handler);


#endif /* INC_LCD_DRIVER_H_ */
