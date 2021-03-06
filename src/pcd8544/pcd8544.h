//
// Created by sl on 18.02.17.
//

#ifndef STM32_MH_Z19_PCD8544_H
#define STM32_MH_Z19_PCD8544_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <wchar.h>

#define PCD8544_RST GPIO1
#define PCD8544_RST_PORT GPIOB

#define PCD8544_DC GPIO1
#define PCD8544_DC_PORT GPIOA
#define PCD8544_SCE GPIO12
#define PCD8544_SCE_PORT GPIOB

#define PCD8544_SPI SPI2
#define PCD8544_SPI_PORT GPIOB
#define PCD8544_SPI_MOSI GPIO15
#define PCD8544_SPI_SCK  GPIO13
#define PCD8544_SPI_SS  GPIO12

#ifndef _BV
#define _BV(bit) (1<<(bit))
#endif

//--- cut here --- cut here --- cut here --- :=)

#define BLACK 1
#define WHITE 0

#define LCDWIDTH 84
#define LCDHEIGHT 48


// H = 0
#define PCD8544_COMMAND_MODE 0x20
#define PCD8544_EXTENDED_MODE 0x21
#define PCD8544_DISPLAY_MODE 0x0c
#define PCD8544_SET_PAGE 0x40
#define PCD8544_SET_ADDRESS 0x80

// H = 1
#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

//--- cut here --- cut here --- cut here --- :=)

#define SCE_LOW gpio_clear(PCD8544_SCE_PORT, PCD8544_SCE);
#define SCE_HIGH gpio_set(PCD8544_SCE_PORT, PCD8544_SCE);
#define DC_HIGH  gpio_set(PCD8544_DC_PORT, PCD8544_DC);
#define DC_LOW  gpio_clear(PCD8544_DC_PORT, PCD8544_DC);

#ifndef PCD8544_BIAS
#define PCD8544_BIAS 4
#endif //PCD8544_BIAS

void pcd8544_init(void);
void pcd8544_display(void);
void pcd8544_setContrast(uint8_t contrast); // maximum value is 0x7f
void pcd8544_drawPixel(int16_t x, int16_t y, uint16_t color);
uint8_t pcd8544_getPixel(int8_t x, int8_t y);
uint8_t pcd8544_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void pcd8544_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void pcd8544_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void pcd8544_drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void pcd8544_drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void pcd8544_clearDisplay();
void pcd8544_drawText(uint8_t x, uint8_t y, uint8_t color, wchar_t *text);

#endif //STM32_MH_Z19_PCD8544_H
