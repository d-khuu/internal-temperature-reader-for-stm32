/*
 * lcd_driver.c
 *
 *  Created on: Mar 31, 2024
 *      Author: Danny
 *
 *      TODO: The LCD screen innately cannot handle strings
 *      that will overflow from the first row to the second
 *      row. So there needs to be a internal cursor tracker
 *      so that the write function can write the rest of the
 *      string to the second line. Issue is where to place
 *      this logic.
 */

#include <lcd_driver.h>

struct lcd_current_state lcd_current_state_g;

int LCD_Setup(I2C_HandleTypeDef* i2c_handler)
{
	HAL_StatusTypeDef ret;
	uint8_t buffer[12];
	buffer[0] = LCD_CONTROL_BYTE;

	// 15ms initialisation delay
	HAL_Delay(15);

	// Set function set
	// 2-line mode, 8-bit Bus Mode, display on
	buffer[1] = LCD_FUNCTION_SET | LCD_DL_8BIT | LCD_2_LINE;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);

	// Delay of at least 39 us
	HAL_Delay(1);

	// Set display mode
	// display on, cursor on, cursor blink on
	buffer[1] = LCD_DISPLAY_ONOFF_CONTROL | LCD_DISP_CTRL_ON | LCD_CURSOR_ON | LCD_CURSOR_BLINK_ON;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);

	// Delay of at least 39 us
	HAL_Delay(1);

	// Clear display
	buffer[1] = LCD_CLEAR_DISPLAY;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);


	// Delay of at least 1.53 ms
	HAL_Delay(2);

	// Entry Mode Set
	// Cursor moves to the right, no shift (no idea what this is)
	buffer[1] = LCD_ENTRY_MODE_SET | LCD_INC_DDRAM | LCD_NO_SHIFT_DISP;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);

	LCD_Initialise_State_Struct(i2c_handler);

	return ret;
}

// TODO: Needs to reset the state struct
int LCD_Reset(I2C_HandleTypeDef* i2c_handler)
{
	HAL_StatusTypeDef ret;

	uint8_t buffer[12];
	buffer[0] = LCD_CONTROL_BYTE;
	buffer[1] = LCD_CLEAR_DISPLAY;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);


	// Delay of at least 1.53 ms
	HAL_Delay(2);

	return ret;
}

int LCD_Write_Char(I2C_HandleTypeDef* i2c_handler, char output_char)
{
	HAL_StatusTypeDef ret;

	uint8_t buffer[12];
	buffer[0] = LCD_CGRAM_ADDRESS;
	buffer[1] = output_char;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);

	return ret;
}


int LCD_Write_String(I2C_HandleTypeDef* i2c_handler, char* output_string)
{
	HAL_StatusTypeDef ret;
	int string_count = strlen(output_string);

	for(int i = 0; i<string_count; i++)
	{
		if(output_string[i] != '\0') ret = LCD_Write_Char(i2c_handler, output_string[i]);
	}

	return ret;
}


int LCD_Set_Cursor_Position(I2C_HandleTypeDef* i2c_handler, int row, int col)
{
	HAL_StatusTypeDef ret;

	uint8_t buffer[12];
	buffer[0] = LCD_DDRAM_ADDRESS;
	if(row == 1)
	{
		buffer[1] = LCD_DDRAM_ADDRESS | LCD_ROW_1_FIRST_ADDRESS | col;
	}
	else if(row == 2)
	{
		buffer[1] = LCD_DDRAM_ADDRESS | LCD_ROW_2_FIRST_ADDRESS | col;
	}
	else
	{
		return HAL_ERROR;
	}

	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);

	return ret;

}

int LCD_Return_Home(I2C_HandleTypeDef* i2c_handler)
{
	HAL_StatusTypeDef ret;

	uint8_t buffer[12];
	buffer[0] = LCD_CONTROL_BYTE;
	buffer[1] = LCD_RETURN_HOME;
	ret = HAL_I2C_Master_Transmit(i2c_handler, LCD_SLAVE_ADDRESS, buffer, 2, HAL_MAX_DELAY);


	// Delay of at least 1.53 ms
	HAL_Delay(2);

	return ret;
}

int LCD_Initialise_State_Struct(I2C_HandleTypeDef* i2c_handler)
{
	lcd_current_state_g.cursor_col = 0;
	lcd_current_state_g.cursor_row = 1;
	memset(lcd_current_state_g.chars_on_screen, 0, sizeof(lcd_current_state_g.chars_on_screen));
//	memset(lcd_current_state_g.chars_on_screen, 0, sizeof(LCD_MAX_CHAR_LENGTH * lcd_current_state_g.chars_on_screen));

	return 1;
}


int __LCD_Redraw_Current_State(I2C_HandleTypeDef* i2c_handler)
{
	LCD_Reset(i2c_handler);
	LCD_Write_String(i2c_handler, lcd_current_state_g.chars_on_screen);
	LCD_Set_Cursor_Position(i2c_handler, lcd_current_state_g.cursor_row, lcd_current_state_g.cursor_col);

	return 1;
}


int LCD_Show_Debug_Message(I2C_HandleTypeDef* i2c_handler, char* output_string)
{
	LCD_Reset(i2c_handler);
	LCD_Write_String(i2c_handler, output_string);
	HAL_Delay(4000);
	__LCD_Redraw_Current_State(i2c_handler);
	return 1;
}

int LCD_Write_String_Non_Debug(I2C_HandleTypeDef* i2c_handler, char* output_string)
{
	//Update the internal current state struct
	int string_count = strlen(output_string);

	for(int i = 0; i<string_count; i++)
	{
		if(output_string[i] != '\0')
		{
			__LCD_State_Update_LCD_Screen(i2c_handler, output_string[i]);
			LCD_Write_Char(i2c_handler, output_string[i]);
			__LCD_State_Update_Cursor_Position(0,0,0);
		}
	}
	// LCD_Write_String(i2c_handler, output_string);
	return 1;
}

int LCD_Set_Cursor_Position_Non_Debug(I2C_HandleTypeDef* i2c_handler, int row, int col)
{
	//Update the internal current state struct
	__LCD_State_Update_Cursor_Position(row, col, 1);


	LCD_Set_Cursor_Position(i2c_handler, row, col);

	return 1;
}

int __LCD_State_Update_Cursor_Position(int row, int col, int update_absolute_position)
{
	int overflows_to_row_2 = (lcd_current_state_g.cursor_row == 1 
							  && lcd_current_state_g.cursor_col == 15);

	// update to an absolute position
	if(update_absolute_position == 1)
	{
		lcd_current_state_g.cursor_row = row;
		lcd_current_state_g.cursor_col = col;
	}
	// If it overflows to the second row
	else if(overflows_to_row_2)
	{
		lcd_current_state_g.cursor_row = 2;
		lcd_current_state_g.cursor_col = 0;
	}
	// The LCD screen's cursor can overflow out of the screen, so no need to account for that
	else
	{
		lcd_current_state_g.cursor_col++;
	}

	return 1;
}

int __LCD_State_Update_LCD_Screen(I2C_HandleTypeDef* i2c_handler, char output_char)
{
	int index = (lcd_current_state_g.cursor_row-1)*16 + lcd_current_state_g.cursor_col;

	char * debug_message = malloc(LCD_MAX_CHAR_LENGTH * sizeof(char));
	if(index >= LCD_MAX_CHAR_LENGTH)
	{
		sprintf(debug_message, "The index exceeded for %c",output_char);
		LCD_Show_Debug_Message(i2c_handler, debug_message);
		return 0;
	}
	lcd_current_state_g.chars_on_screen[index] = output_char;

	return 1;
}
