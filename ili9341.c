/**
 ******************************************************************************
 * @file    ili9341.c
 * @author  Adapted from Adafruit Industries
 * @version V1.0
 * @date    13-August-2025
 * @brief   Source file for ILI9341 TFT LCD driver.
 *
 * This file is adapted from the Adafruit ILI9341 Arduino library for use with
 * STM32CubeIDE and HAL libraries.
 *
 ******************************************************************************
 */

#include "ili9341.h"

// Store the SPI handle
static SPI_HandleTypeDef* g_hspi;

// Store current screen dimensions
static uint16_t _width = ILI9341_TFTWIDTH;
static uint16_t _height = ILI9341_TFTHEIGHT;

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

//
// --- Low-Level SPI Communication Functions ---
//

/**
 * @brief Selects the TFT display by pulling CS low.
 */
static void ILI9341_Select() {
    HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Deselects the TFT display by pulling CS high.
 */
static void ILI9341_Unselect() {
    HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief Sends a command byte to the display.
 * @param cmd The command byte to send.
 */
static void ILI9341_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_RESET); // Command mode
    ILI9341_Select();
    HAL_SPI_Transmit(g_hspi, &cmd, 1, HAL_MAX_DELAY);
    ILI9341_Unselect();
}

/**
 * @brief Sends a data byte to the display.
 * @param data The data byte to send.
 */
static void ILI9341_WriteData(uint8_t data) {
    HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET); // Data mode
    ILI9341_Select();
    HAL_SPI_Transmit(g_hspi, &data, 1, HAL_MAX_DELAY);
    ILI9341_Unselect();
}

/**
 * @brief Sends a buffer of data to the display.
 * @param buff Pointer to the data buffer.
 * @param buff_size Size of the buffer in bytes.
 */
static void ILI9341_WriteDataBuffer(uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET); // Data mode
    ILI9341_Select();
    HAL_SPI_Transmit(g_hspi, buff, buff_size, HAL_MAX_DELAY);
    ILI9341_Unselect();
}

/**
 * @brief Performs a hardware reset of the display.
 */
static void ILI9341_Reset() {
    HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(150);
}

//
// --- Initialization Sequence ---
// This sequence is adapted directly from the Adafruit library.
//
static const uint8_t init_cmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,
  ILI9341_PWCTR2  , 1, 0x10,
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,
  ILI9341_VMCTR2  , 1, 0x86,
  ILI9341_MADCTL  , 1, 0x48,
  ILI9341_VSCRSADD, 1, 0x00,
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27,
  0xF2, 1, 0x00,
  ILI9341_GAMMASET , 1, 0x01,
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80, // Exit Sleep
  ILI9341_DISPON  , 0x80, // Display on
  0x00 // End of list
};


//
// --- Public Functions ---
//

void ILI9341_Init(SPI_HandleTypeDef* hspi) {
    g_hspi = hspi;

    ILI9341_Reset();

    ILI9341_WriteCommand(ILI9341_SWRESET);
    HAL_Delay(150);

    const uint8_t *addr = init_cmd;
    uint8_t cmd, numArgs;
    while ((cmd = *addr++) > 0) {
        numArgs = *addr++;
        uint8_t msb = numArgs & 0x80; // Check for delay flag
        numArgs &= ~0x80;
        if (numArgs > 0) {
            ILI9341_WriteCommand(cmd);
            ILI9341_WriteDataBuffer((uint8_t*)addr, numArgs);
        } else {
             ILI9341_WriteCommand(cmd);
        }
        addr += numArgs;
        if (msb) {
            HAL_Delay(150);
        }
    }
}

void ILI9341_SetRotation(uint8_t rotation) {
    uint8_t madctl_val = 0;
    rotation %= 4;

    switch (rotation) {
        case 0:
            madctl_val = (MADCTL_MX | MADCTL_BGR);
            _width = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 1:
            madctl_val = (MADCTL_MV | MADCTL_BGR);
            _width = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
        case 2:
            madctl_val = (MADCTL_MY | MADCTL_BGR);
            _width = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 3:
            madctl_val = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
            _width = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
    }
    ILI9341_WriteCommand(ILI9341_MADCTL);
    ILI9341_WriteData(madctl_val);
}

void ILI9341_SetAddressWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
    uint16_t x2 = x1 + w - 1;
    uint16_t y2 = y1 + h - 1;
    uint8_t data[4];

    // Column Address Set
    ILI9341_WriteCommand(ILI9341_CASET);
    data[0] = x1 >> 8;
    data[1] = x1 & 0xFF;
    data[2] = x2 >> 8;
    data[3] = x2 & 0xFF;
    ILI9341_WriteDataBuffer(data, 4);

    // Page Address Set
    ILI9341_WriteCommand(ILI9341_PASET);
    data[0] = y1 >> 8;
    data[1] = y1 & 0xFF;
    data[2] = y2 >> 8;
    data[3] = y2 & 0xFF;
    ILI9341_WriteDataBuffer(data, 4);

    // Memory Write
    ILI9341_WriteCommand(ILI9341_RAMWR);
}

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if ((x >= _width) || (y >= _height)) return;

    ILI9341_SetAddressWindow(x, y, 1, 1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ILI9341_WriteDataBuffer(data, sizeof(data));
}

void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if ((x >= _width) || (y >= _height)) return;
    if ((x + w - 1) >= _width) w = _width - x;
    if ((y + h - 1) >= _height) h = _height - y;

    ILI9341_SetAddressWindow(x, y, w, h);

    uint8_t data[] = { color >> 8, color & 0xFF };
    HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET);
    ILI9341_Select();

    // For performance, a buffer could be used here instead of looping
    // For this basic driver, we send pixel by pixel
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            HAL_SPI_Transmit(g_hspi, data, 2, HAL_MAX_DELAY);
        }
    }
    ILI9341_Unselect();
}
