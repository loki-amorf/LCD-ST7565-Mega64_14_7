/** \file sedit.h Simple Editors header
 *
 *  Created on: 10.11.2009
 *      Author: Pоман
 */

#ifndef SEDIT_H_
#define SEDIT_H_

#include <avr/pgmspace.h>

void se_init_spec_symbols(uint8_t symbol_count, uint8_t *symbols);
void init_h_scale_sym(void);
void init_hbar_scale_sym(void);
void init_v_scale_sym(void);

void v_bar(uint8_t val, uint8_t base);
void v2_bar(uint8_t pos, uint8_t val, uint8_t base);

void se_scale_percent(uint8_t percent);
void se_percent(uint8_t x, uint8_t y, uint8_t percent);

// uint16_t edit_uint(uint8_t y, prog_char *msg, uint16_t val, uint16_t min, uint16_t max);
// int16_t edit_sint(uint8_t y, prog_char *msg, int16_t val, int16_t min, int16_t max);

uint16_t edit_uint(uint8_t y, const char *msg, uint16_t val, uint16_t min, uint16_t max);
int16_t edit_sint(uint8_t y, const char *msg, int16_t val, int16_t min, int16_t max);


#endif /* SEDIT_H_ */
