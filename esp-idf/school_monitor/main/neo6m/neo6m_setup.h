/*
 * Implementation for NEO-6M GPS using UART.
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

#ifndef NEO6M_SETUP_H
#define NEO6M_SETUP_H

#include "global_data.h"

#define TX_PIN          32
#define RX_PIN          33
#define RD_BUF_SIZE     1024
#define PATTERN_CHR_NUM (1)

void neo6m_task ( void *pvParameters );

#endif /* NEO6M_SETUP_H */
