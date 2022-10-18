/*
 * Implementation of dust sensor DSM501A.
 * 
 * (c)2022 Victor Moura
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *    
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DSM501A_SETUP_H
#define DSM501A_SETUP_H

#include "global_data.h"

#define DSM501a_RED_PIN         GPIO_NUM_34 //DSM501A Vout2 sensibilidade em 1 micrometro 
#define DSM501a_YELLOW_PIN      GPIO_NUM_35 //DSM501A Vout1 sensibilidade em 2.5 micrometro, se pino Controle (preto) aberto
#define GPIO_INPUT_PIN_SEL      (uint64_t)((1ULL<<DSM501a_RED_PIN) | (1ULL<<DSM501a_YELLOW_PIN))
#define ESP_INTR_FLAG_DEFAULT   0

void IRAM_ATTR change_v2_isr ( void *arg );
void IRAM_ATTR change_v1_isr ( void *arg );
void dsm501a_task ( void *pvParameters );

#endif /* DSM501A_SETUP_H */
