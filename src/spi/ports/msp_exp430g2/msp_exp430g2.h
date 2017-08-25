/*******************************************************************************
* Copyright (c) 2017, Alan Barr
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __BOARD_H__
#define __BOARD_H__

#define PIN_RED_LED_PORT1   BIT0    /* Port 1 Pin 0 */
#define PIN_UARTTX_PORT1    BIT1    /* Port 1 Pin 1 */
#define PIN_UARTRX_PORT1    BIT2    /* Port 1 Pin 2 */
#define PIN_SWITCH_PORT1    BIT3    /* Port 1 Pin 3 */
#define PIN_4_PORT1         BIT4    /* Port 1 Pin 4 */
#define PIN_CLOCK_PORT1     BIT5    /* Port 1 Pin 5 */
#define PIN_SLAVE_OUT_PORT1 BIT6    /* Port 1 Pin 6 */  /* Green LED */
#define PIN_SLAVE_IN_PORT1  BIT7    /* Port 1 Pin 7 */

#define PIN_0_PORT2         BIT0    /* Port 2 Pin 0 */
#define PIN_1_PORT2         BIT1    /* Port 2 Pin 1 */
#define PIN_2_PORT2         BIT2    /* Port 2 Pin 2 */
#define PIN_3_PORT2         BIT3    /* Port 2 Pin 3 */
#define PIN_4_PORT2         BIT4    /* Port 2 Pin 4 */
#define PIN_5_PORT2         BIT5    /* Port 2 Pin 5 */
#define PIN_GDO_PORT2       BIT6    /* Port 2 Pin 6 */
#define PIN_SELECT_PORT2    BIT7    /* Port 2 Pin 7 */

#endif
