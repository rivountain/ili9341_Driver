/**
 ******************************************************************************
 * @file    ili9341.h
 * @author  Adapted from Adafruit Industries
 * @version V1.0
 * @date    7-August-2025
 * @brief   Header file for ILI9341 TFT LCD driver.
 *
 * This file is adapted from the Adafruit ILI9341 Arduino library for use with
 * STM32CubeIDE and HAL libraries.
 *
 ******************************************************************************
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "stm32u5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

//
//  --- PIN CONFIGURATION ---
//
#define ILI9341_CS_PORT     GPIOD
#define ILI9341_CS_PIN      GPIO_PIN_14
#define ILI9341_DC_PORT     GPIOD
#define ILI9341_DC_PIN      GPIO_PIN_15
#define ILI9341_RST_PORT    GPIOF
#define ILI9341_RST_PIN     GPIO_PIN_12

//
//  --- DISPLAY CONFIGURATION ---
//
#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

//
//  --- ILI9341 COMMANDS ---
//
#define ILI9341_NOP        0x00     ///< No-op register
#define ILI9341_SWRESET    0x01     ///< Software reset register
#define ILI9341_RDDID      0x04     ///< Read display identification information
#define ILI9341_RDDST      0x09     ///< Read Display Status
#define ILI9341_SLPIN      0x10     ///< Enter Sleep Mode
#define ILI9341_SLPOUT     0x11     ///< Sleep Out
#define ILI9341_PTLON      0x12     ///< Partial Mode ON
#define ILI9341_NORON      0x13     ///< Normal Display Mode ON
#define ILI9341_INVOFF     0x20     ///< Display Inversion OFF
#define ILI9341_INVON      0x21     ///< Display Inversion ON
#define ILI9341_GAMMASET   0x26     ///< Gamma Set
#define ILI9341_DISPOFF    0x28     ///< Display OFF
#define ILI9341_DISPON     0x29     ///< Display ON
#define ILI9341_CASET      0x2A     ///< Column Address Set
#define ILI9341_PASET      0x2B     ///< Page Address Set
#define ILI9341_RAMWR      0x2C     ///< Memory Write
#define ILI9341_RAMRD      0x2E     ///< Memory Read
#define ILI9341_PTLAR      0x30     ///< Partial Area
#define ILI9341_VSCRDEF    0x33     ///< Vertical Scrolling Definition
#define ILI9341_MADCTL     0x36     ///< Memory Access Control
#define ILI9341_VSCRSADD   0x37     ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT     0x3A     ///< COLMOD: Pixel Format Set
#define ILI9341_FRMCTR1    0xB1     ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2    0xB2     ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3    0xB3     ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR     0xB4     ///< Display Inversion Control
#define ILI9341_DFUNCTR    0xB6     ///< Display Function Control
#define ILI9341_PWCTR1     0xC0     ///< Power Control 1
#define ILI9341_PWCTR2     0xC1     ///< Power Control 2
#define ILI9341_VMCTR1     0xC5     ///< VCOM Control 1
#define ILI9341_VMCTR2     0xC7     ///< VCOM Control 2
#define ILI9341_RDID1      0xDA     ///< Read ID 1
#define ILI9341_RDID2      0xDB     ///< Read ID 2
#define ILI9341_RDID3      0xDC     ///< Read ID 3
#define ILI9341_GMCTRP1    0xE0     ///< Positive Gamma Correction
#define ILI9341_GMCTRN1    0xE1     ///< Negative Gamma Correction

//
//  --- COLOR DEFINITIONS ---
//
#define ILI9341_BLACK       0x0000
#define ILI9341_NAVY        0x000F
#define ILI9341_DARKGREEN   0x03E0
#define ILI9341_DARKCYAN    0x03EF
#define ILI9341_MAROON      0x7800
#define ILI9341_PURPLE      0x780F
#define ILI9341_OLIVE       0x7BE0
#define ILI9341_LIGHTGREY   0xC618
#define ILI9341_DARKGREY    0x7BEF
#define ILI9341_BLUE        0x001F
#define ILI9341_GREEN       0x07E0
#define ILI9341_CYAN        0x07FF
#define ILI9341_RED         0xF800
#define ILI9341_MAGENTA     0xF81F
#define ILI9341_YELLOW      0xFFE0
#define ILI9341_WHITE       0xFFFF
#define ILI9341_ORANGE      0xFD20
#define ILI9341_GREENYELLOW 0xAFE5
#define ILI9341_PINK        0xFC18

//
//  --- FUNCTION PROTOTYPES ---
//
/**
 * @brief Initializes the ILI9341 display driver.
 * @param hspi Pointer to the SPI_HandleTypeDef structure for the SPI peripheral.
 */
void ILI9341_Init(SPI_HandleTypeDef* hspi);

/**
 * @brief Sets the rotation of the display.
 * @param rotation The rotation value (0-3).
 */
void ILI9341_SetRotation(uint8_t rotation);

/**
 * @brief Sets the address window for pixel writing.
 * @param x1 The starting X coordinate.
 * @param y1 The starting Y coordinate.
 * @param w The width of the window.
 * @param h The height of the window.
 */
void ILI9341_SetAddressWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h);

/**
 * @brief Fills a rectangle with a specified color.
 * @param x The starting X coordinate of the rectangle.
 * @param y The starting Y coordinate of the rectangle.
 * @param w The width of the rectangle.
 * @param h The height of the rectangle.
 * @param color The 16-bit color to fill the rectangle with.
 */
void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief Draws a single pixel at a specified coordinate with a specified color.
 * @param x The X coordinate of the pixel.
 * @param y The Y coordinate of the pixel.
 * @param color The 16-bit color of the pixel.
 */
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* INC_ILI9341_H_ */
