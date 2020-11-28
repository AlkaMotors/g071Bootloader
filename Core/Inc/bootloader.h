/*
 * bootloader.h
 *
 *  Created on: Sep 3, 2020
 *      Author: Alka
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_
#include "main.h"


#endif /* INC_BOOTLOADER_H_ */

//void save_to_flash(uint8_t *data);
//void read_flash(uint8_t* data, uint32_t address);
//void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
void read_flash_bin(uint8_t*  data , uint32_t add ,int  out_buff_len);
void save_flash_nolib(uint8_t *data, int length, uint32_t add);
void erase_flash();
