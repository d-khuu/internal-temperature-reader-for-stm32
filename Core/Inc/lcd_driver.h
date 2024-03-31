/*
 * lcd_driver.h
 *
 *  Created on: Mar 31, 2024
 *      Author: Danny
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_


#define LCD_SLAVE_ADDRESS 0b0111110

int lcd_setup();

int lcd_write_string(char* output_string);

int lcd_reset();





#endif /* INC_LCD_DRIVER_H_ */
