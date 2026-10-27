# SD card on the LCD's bus

A microSD card can share the FM-1's LCD bus (SPI1). It is a hardware mod: nothing on a stock FM-1 changes, and a
firmware built without it drives the LCD as before. `boards/fm1/fm1_sd_hw.h` has the bus helpers; a card driver on top
of them (SPI mode, CMD0 .. CMD59) is the application's, for example fm1-base's `device/sd.c` (`plat_sd_*`).

## Wiring

| Card (SPI mode) | FM-1 | Notes |
|---|---|---|
| CLK | PC9, the LCD's clock | shared |
| DI / CMD | PC10, the LCD's data out | shared |
| DO / DAT0 | PC8, the LCD's D/C | PC8 is SPI1's data in in pin group B; a series resistor (220 Ohm to 1 kOhm) limits the current if both ends ever drive it |
| CS / DAT3 | PC7, the LCD's CS, through an inverter (74LVC1G04) | the card is selected whenever the LCD is not |
| VDD, VSS | 3.3 V, ground | a card board without a regulator or level shifters |

Where to take the four lines from on the FM-1's board (the LCD's flat-cable connector or its traces) is not settled
here: check each against the main chip's pins with a meter before soldering.

## The rules this wiring sets

- **The LCD idles selected** (CS low, D/C high, no clock): the ST7789 allows CS held low, and with CS low the card's CS
  is high, so the card is off the bus and lets go of PC8. `fm1_lcd_idle()` replaces `fm1_lcd_deselect()` in a build
  with the card; `fm1_lcd_cs_pulse()` before each LCD command gives the panel a CS edge to count its bits from (no
  clock runs meanwhile, so the card, selected for that moment, sees nothing).
- **A card session raises the LCD's CS.** Between the card's commands the card is deselected and the LCD selected:
  the bytes clocked then (the 8 release clocks after a command) reach the LCD as data with D/C high. Send the LCD a
  NOP before a session so that no memory write is open: the panel ignores data with no command waiting.
- **Main loop only**, the LCD's DMA finished (`fm1_lcd_wait`) before `fm1_sd_hw_take`, and the LCD's clock set again
  (`fm1_lcd_baud`) after `fm1_sd_hw_give`.
- A firmware that idles the LCD's CS high (any firmware without the card: the stock one, its update loader) keeps the
  card selected. That is harmless while nothing reads the card, but take the card out before a stock update.

## Helpers (`fm1_sd_hw.h`)

| | |
|---|---|
| `fm1_sd_hw_take(baud, mode)` | the bus to the card: PC8 readable and pulled up (an absent card reads 0xFF), SPI1 at lsb / (baud + 1); returns SPI1's CON |
| `fm1_sd_hw_give(con, mode)` | the card deselected, the LCD selected and idle, PC8 driven, SPI1's CON back |
| `fm1_sd_hw_sel()`, `fm1_sd_hw_desel()` | the card on (LCD CS high, PC8 an input) / off (PC8 driven high, then LCD CS low) |
| `fm1_sd_hw_xfer(b, mode)` | a byte out, the byte in |
| `fm1_sd_hw_dma(p, n, rx)`, `fm1_sd_hw_dma_end()` | n bytes by SPI1's DMA (RAM, aligned), out or in; its end waited for |
| `FM1_SD_HW_BAUD_INIT`, `FM1_SD_HW_BAUD_RUN` | 400 kHz (a card's start-up), 20 MHz (default speed) |

Three ways to read a byte, because which one SPI1 does on PC8 is not yet seen on an FM-1:
`FM1_SD_HW_RX` (the direction bit set while 0xFF is clocked, as `fm1_flash.h` reads SPI0), `FM1_SD_HW_DUPLEX` (the
byte shifted in while one is sent, read back from BUF) and `FM1_SD_HW_GPIO` (the three lines as plain pins, SPI1 off
them for the session; no DMA). A driver tries them in turn until the card answers CMD0.

## Not verified on an FM-1

No FM-1 has the mod yet. Unverified: the read modes above, SPI1's DMA receiving as master (a driver should check it
against a byte-by-byte read of the same block before using it), 20 MHz over the added wiring, and the panel ignoring
the data bytes after a NOP. The helpers are host-tested only, through fm1-base's driver test (a card in SPI mode,
byte by byte).
