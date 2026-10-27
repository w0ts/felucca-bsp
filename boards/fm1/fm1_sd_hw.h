/* SPDX-License-Identifier: MPL-2.0
 * Copyright (C) 2026 Henri David */
/* A microSD card on the LCD's SPI1 (a hardware mod; docs/SD.md): the card's CLK and DI on the LCD's PC9 and PC10,
 * its DO on PC8 (the LCD's D/C, which is SPI1's data in in pin group B) through a series resistor, its CS the LCD's
 * PC7 through an inverter. So the card is selected whenever the LCD is not: with the card fitted the LCD idles
 * selected (CS low, D/C high, no clock; the ST7789 allows CS held low) and the card is off the bus, and a card
 * session raises the LCD's CS to select the card. The bytes clocked while the card is deselected reach the LCD as
 * data (D/C high): a NOP to the LCD before a session ends any memory write, so the LCD ignores them.
 * Main loop only, the LCD's transfer finished (fm1_lcd_wait) before fm1_sd_hw_take.
 *
 *   fm1_lcd_idle()              the LCD selected and idle: D/C high, then CS low (in place of fm1_lcd_deselect)
 *   fm1_lcd_cs_pulse()          CS high for a moment, then low (before each LCD command: the panel counts its bits
 *                               from a CS edge; no clock meanwhile, so the card, selected for that moment, sees none)
 *   fm1_sd_hw_take(baud, mode)  the bus to the card: PC8 readable and pulled up (no card: 0xFF, as an idle card),
 *                               PC10 pulled up, SPI1's clock lsb / (baud + 1); returns SPI1's CON to give back
 *   fm1_sd_hw_give(con, mode)   the card deselected, the LCD selected and idle, PC8 driven, SPI1's CON as it was; the
 *                               caller sets the LCD's clock again (fm1_lcd_baud)
 *   fm1_sd_hw_baud(baud)        the clock, as above
 *   fm1_sd_hw_sel() / _desel()  the card selected (LCD CS high, PC8 an input) / deselected (PC8 driven high, LCD CS low)
 *   fm1_sd_hw_xfer(b, mode)     one byte out, the byte in
 *   fm1_sd_hw_dma(p, n, rx)     n bytes (RAM, 4-byte aligned) by SPI1's DMA, out of p or (rx) into p; returns at once
 *   fm1_sd_hw_dma_end()         its end waited for (bounded), SPI1 back to sending; 0 on a timeout
 * Modes (which way SPI1 reads its data in is the card's driver's to find out, from the card's answers):
 *   FM1_SD_HW_RX      a byte in with the direction bit set (CON b12), as fm1_flash.h reads SPI0
 *   FM1_SD_HW_DUPLEX  a byte out and the byte shifted in read back from BUF
 *   FM1_SD_HW_GPIO    PC9 / PC10 / PC8 as plain pins (SPI1 off them for the session), SPI mode 0, MSB first; no DMA */
#pragma once
#include <stdint.h>
#include "fm1_gpio.h"
#include "fm1_lcd_hw.h"

enum { FM1_SD_HW_RX = 0, FM1_SD_HW_DUPLEX = 1, FM1_SD_HW_GPIO = 2 };
#define FM1_SD_HW_BAUD_INIT 149u       /* lsb 60 MHz / 150 = 400 kHz: a card's start-up (400 kHz at most) */
#define FM1_SD_HW_BAUD_RUN 2u          /* 20 MHz (a card's default speed: 25 MHz at most) */

static uint32_t fm1_sd_hw_half;        /* GPIO mode: spin loops per half clock */

FM1_INLINE void fm1_lcd_idle(void)
{
    FM1_LCD_PC_OUT |= FM1_LCD_DC;
    FM1_LCD_PC_OUT &= ~FM1_LCD_CS;
}

FM1_INLINE void fm1_lcd_cs_pulse(void)
{
    FM1_LCD_PC_OUT |= FM1_LCD_CS;
    for (volatile uint32_t i = 0; i < 8u; i++)       /* tCHW 40 ns (ST7789V): a few register reads */
        (void)FM1_LCD_PC_OUT;
    FM1_LCD_PC_OUT &= ~FM1_LCD_CS;
}

FM1_INLINE void fm1_sd_hw_baud(uint32_t baud)
{
    FM1_LCD_SPI_BAUD = baud;
    fm1_sd_hw_half = baud > 20u ? 40u : 0u;
}

FM1_INLINE void fm1_sd_hw_sel(void)
{
    FM1_LCD_PC_OUT |= FM1_LCD_CS;
    FM1_LCD_PC_DIR |= FM1_LCD_DC;
}

FM1_INLINE void fm1_sd_hw_desel(void)
{
    FM1_LCD_PC_OUT |= FM1_LCD_DC;
    FM1_LCD_PC_DIR &= ~FM1_LCD_DC;
    FM1_LCD_PC_OUT &= ~FM1_LCD_CS;
}

static uint32_t fm1_sd_hw_take(uint32_t baud, uint32_t mode)
{
    uint32_t con = FM1_LCD_SPI_CON & ~0xC000u;
    FM1_PR(FM1_PC, FM1_DIE) |= FM1_LCD_DC;
    FM1_PR(FM1_PC, FM1_PU) |= FM1_LCD_DC | FM1_LCD_DO;
    if (mode == FM1_SD_HW_GPIO) {
        FM1_LCD_IOMAP_CON1 &= ~0x10u;                /* SPI1 off PC9 / PC10 */
        FM1_LCD_PC_OUT &= ~FM1_LCD_CLK;
    }
    fm1_sd_hw_baud(baud);
    return con;
}

static void fm1_sd_hw_give(uint32_t con, uint32_t mode)
{
    fm1_sd_hw_desel();
    FM1_PR(FM1_PC, FM1_PU) &= ~(FM1_LCD_DC | FM1_LCD_DO);
    if (mode == FM1_SD_HW_GPIO)
        FM1_LCD_IOMAP_CON1 |= 0x10u;
    FM1_LCD_SPI_CON = con | 0x4000u;
}

static uint32_t fm1_sd_hw_wait(void)
{
    uint32_t n;
    for (n = 0; n < 100000u && !(FM1_LCD_SPI_CON & 0x8000u); n++)
        ;
    FM1_LCD_SPI_CON |= 0x4000u;
    return n < 100000u;
}

static uint8_t fm1_sd_hw_gpio(uint8_t b)
{
    uint32_t i, in = 0;
    for (i = 0; i < 8u; i++, b = (uint8_t)(b << 1)) {
        if (b & 0x80u)
            FM1_LCD_PC_OUT |= FM1_LCD_DO;
        else
            FM1_LCD_PC_OUT &= ~FM1_LCD_DO;
        for (volatile uint32_t k = 0; k < fm1_sd_hw_half; k++)
            ;
        FM1_LCD_PC_OUT |= FM1_LCD_CLK;
        in = in << 1 | ((FM1_PR(FM1_PC, FM1_IN) >> 8) & 1u);
        for (volatile uint32_t k = 0; k < fm1_sd_hw_half; k++)
            ;
        FM1_LCD_PC_OUT &= ~FM1_LCD_CLK;
    }
    return (uint8_t)in;
}

static uint8_t fm1_sd_hw_xfer(uint8_t b, uint32_t mode)
{
    uint8_t v;
    if (mode == FM1_SD_HW_GPIO)
        return fm1_sd_hw_gpio(b);
    FM1_LCD_SPI_CON |= 8u;
    if (mode == FM1_SD_HW_RX && b == 0xFFu)
        FM1_LCD_SPI_CON |= 0x1000u;
    else
        FM1_LCD_SPI_CON &= ~0x1000u;
    FM1_LCD_SPI_BUF = b;
    (void)fm1_sd_hw_wait();
    v = (uint8_t)FM1_LCD_SPI_BUF;
    FM1_LCD_SPI_CON &= ~(8u | 0x1000u);
    return v;
}

FM1_INLINE void fm1_sd_hw_dma(const void *p, uint32_t n, uint32_t rx)   /* n > 0 */
{
    FM1_LCD_SPI_CON |= 8u;
    if (rx)
        FM1_LCD_SPI_CON |= 0x1000u;
    else
        FM1_LCD_SPI_CON &= ~0x1000u;
    FM1_LCD_SPI_CON |= 0x4000u;
    FM1_LCD_SPI_ADR = (uint32_t)(uintptr_t)p;
    FM1_LCD_SPI_CNT = n;                             /* starts it */
}

static uint32_t fm1_sd_hw_dma_end(void)
{
    uint32_t n;
    for (n = 0; n < 1000000u && !(FM1_LCD_SPI_CON & 0x8000u); n++)
        ;
    FM1_LCD_SPI_CON |= 0x4000u;
    FM1_LCD_SPI_CON &= ~(8u | 0x1000u);
    return n < 1000000u;
}
