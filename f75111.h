/* Copyright (c) 2012, Daniel Toussaint ,  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#define CHIPID1 0x5a
#define CHIPID2 0x5b

#define GPIO1_OCR 0x10
#define GPIO1_ODR 0x11 
#define GPIO1_ISR 0x12 

#define GPIO2_OCR 0x20
#define GPIO2_ODR 0x21
#define GPIO2_ISR 0x22


#define CONFIG_FUNCTION 0x03
#define SMI_ENABLE 0x2a
#define EDGE_DETECT_ENABLE 0x28
#define EDGE_DETECT_STATUS 0x29 


#define CLEAR_INPUTS 0x29
