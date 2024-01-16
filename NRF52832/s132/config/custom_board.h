/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef PCA10040_H
#define PCA10040_H

#include "nrf_gpio.h"


                                                          ;
// PORT12 - IRQ (INTERUPT REQUEST from TRF7970A)               ;
// PORT22 - TRF7970A ENABLE                                    ;
// PORT29 - ISO14443B LED                                      ;
// PORT27 - ISO14443A LED                                      ;
// PORT26 - ISO15693  LED                                      ;

// LEDs definitions for Board
#define LEDS_NUMBER    1

#define LED_1          17
#define LEDS_ACTIVE_STATE 1
#define LEDS_LIST { LED_1 }
//#define BSP_LED_0      LED_1

#define BUTTONS_NUMBER 1
#define BUTTON_1       13
#define BUTTON_STOP    13
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
#define BUTTONS_ACTIVE_STATE 0
#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1


#define MCU_EN          4
#define MCU_KEY         5
//#define CW_SDA         15
//#define CW_SCL         17
#define LED1           20
#define LED2           30
#define LED3           3
#define LED4           28

//Sensor Pin Contact
#define USE_IIC

#define IIC_SDA   15
#define IIC_SCL   17

// PORT23 - SLAVE SELECT                                       ;
// PORT24 - SPI DATACLK                                        ;
// PORT30 - SPI MISO                                           ;
// PORT25 - SPI MOSI                                           ;

//#define USE_SPI

//#define SPI_PIN_SS        23
//#define SPI_PIN_SCK				24
//#define SPI_PIN_MOSI			25
//#define SPI_PIN_MISO			30

//#define USE_UART

//#define UART_SENSOR_TX 29
//#define UART_SENSOR_RX 28

//#define SENSOR_NAME_CS 10

#endif // PCA10040_H
