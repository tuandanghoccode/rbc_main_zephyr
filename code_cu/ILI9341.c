/*
 * ILI9341.c
 * ILI9341 SPI Polling Driver (No DMA) for STM32F407
 * Refactored: Removed AI, DMA, and unused bloat.
 */

#include "ILI9341.h"
#include "main.h"
#include "string.h"

/******************************************************************************
 * CONFIG                                     *
 ******************************************************************************/
// Đảm bảo các pin này được định nghĩa trong main.h hoặc thay thế bằng tên pin thực tế của bạn
// Ví dụ: #define ILI9341_CS_GPIO_Port GPIOA ...
#define ILI9341_RESX_LOW()  HAL_GPIO_WritePin(ILI9341_RES_GPIO_Port, ILI9341_RES_Pin, GPIO_PIN_RESET)
#define ILI9341_RESX_HIGH() HAL_GPIO_WritePin(ILI9341_RES_GPIO_Port, ILI9341_RES_Pin, GPIO_PIN_SET)
#define ILI9341_CSX_LOW()   HAL_GPIO_WritePin(ILI9341_CS_GPIO_Port, ILI9341_CS_Pin, GPIO_PIN_RESET)
#define ILI9341_CSX_HIGH()  HAL_GPIO_WritePin(ILI9341_CS_GPIO_Port, ILI9341_CS_Pin, GPIO_PIN_SET)
#define ILI9341_DCX_LOW()   HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_RESET)
#define ILI9341_DCX_HIGH()  HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_SET)

#define LCD_W 320
#define LCD_H 240

/******************************************************************************
 * COMMANDS                                    *
 ******************************************************************************/
#define ILI9341_SWRESET     0x01
#define ILI9341_SLPOUT      0x11
#define ILI9341_DISPON      0x29
#define ILI9341_CASET       0x2A
#define ILI9341_RASET       0x2B
#define ILI9341_RAMWR       0x2C
#define ILI9341_MADCTL      0x36
#define ILI9341_PIXFMT      0x3A

/******************************************************************************
 * GLOBALS                                    *
 ******************************************************************************/
static SPI_HandleTypeDef *ili9341_hspi;

// Font 5x7 Data
const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x10,0x08,0x08,0x10,0x08}, // '~'
    {0x00,0x00,0x00,0x00,0x00}  // DEL
};

/******************************************************************************
 * INTERNAL FUNCTIONS                              *
 ******************************************************************************/

static void ILI9341_WriteCmd(uint8_t cmd)
{
    ILI9341_DCX_LOW(); // Command
    ILI9341_CSX_LOW();
    HAL_SPI_Transmit(ili9341_hspi, &cmd, 1, 10);
    ILI9341_CSX_HIGH();
    ILI9341_DCX_HIGH(); // Default back to Data
}

static void ILI9341_WriteData(const uint8_t *buff, size_t buff_size)
{
    ILI9341_DCX_HIGH(); // Data
    ILI9341_CSX_LOW();
    HAL_SPI_Transmit(ili9341_hspi, (uint8_t*)buff, buff_size, HAL_MAX_DELAY);
    ILI9341_CSX_HIGH();
}

static void ILI9341_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t data[4];

    // Column Address Set
    ILI9341_WriteCmd(ILI9341_CASET);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    ILI9341_WriteData(data, 4);

    // Page Address Set
    ILI9341_WriteCmd(ILI9341_RASET);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    ILI9341_WriteData(data, 4);

    // Write to RAM
    ILI9341_WriteCmd(ILI9341_RAMWR);
}

/******************************************************************************
 * PUBLIC FUNCTIONS                               *
 ******************************************************************************/

void ILI9341_Init(SPI_HandleTypeDef *spi_handle)
{
    ili9341_hspi = spi_handle;

    // Reset Hardware
    ILI9341_RESX_LOW();
    HAL_Delay(50);
    ILI9341_RESX_HIGH();
    HAL_Delay(50);

    // Init Sequence (Giữ nguyên cấu hình của bạn)
    ILI9341_WriteCmd(ILI9341_SWRESET);
    HAL_Delay(100);

    uint8_t params[15];

    params[0] = 0x00; params[1] = 0xD9; params[2] = 0x30;
    ILI9341_WriteCmd(0xCF); ILI9341_WriteData(params, 3); // POWERB

    params[0] = 0x64; params[1] = 0x03; params[2] = 0X12; params[3] = 0X81;
    ILI9341_WriteCmd(0xED); ILI9341_WriteData(params, 4); // POWER_SEQ

    params[0] = 0x85; params[1] = 0x10; params[2] = 0x7A;
    ILI9341_WriteCmd(0xE8); ILI9341_WriteData(params, 3); // DTCA

    params[0] = 0x39; params[1] = 0x2C; params[2] = 0x00; params[3] = 0x34; params[4] = 0x02;
    ILI9341_WriteCmd(0xCB); ILI9341_WriteData(params, 5); // POWERA

    params[0] = 0x20;
    ILI9341_WriteCmd(0xF7); ILI9341_WriteData(params, 1); // PRC

    params[0] = 0x00; params[1] = 0x00;
    ILI9341_WriteCmd(0xEA); ILI9341_WriteData(params, 2); // DTCB

    params[0] = 0x1B;
    ILI9341_WriteCmd(0xC0); ILI9341_WriteData(params, 1); // POWER1

    params[0] = 0x12;
    ILI9341_WriteCmd(0xC1); ILI9341_WriteData(params, 1); // POWER2

    params[0] = 0x08; params[1] = 0x26;
    ILI9341_WriteCmd(0xC5); ILI9341_WriteData(params, 2); // VCOM1

    params[0] = 0XB7;
    ILI9341_WriteCmd(0xC7); ILI9341_WriteData(params, 1); // VCOM2

    // Pixel Format: RGB565
    params[0] = 0x55;
    ILI9341_WriteCmd(ILI9341_PIXFMT); ILI9341_WriteData(params, 1);

    // Frame Rate
    params[0] = 0x00; params[1] = 0x1B;
    ILI9341_WriteCmd(0xB1); ILI9341_WriteData(params, 2); // FRMCTR1

    // Display Function
    params[0] = 0x0A; params[1] = 0xA2;
    ILI9341_WriteCmd(0xB6); ILI9341_WriteData(params, 2); // DFC

    // Gamma
    params[0] = 0x02;
    ILI9341_WriteCmd(0xF2); ILI9341_WriteData(params, 1); // 3GAMMA_EN
    params[0] = 0x01;
    ILI9341_WriteCmd(0x26); ILI9341_WriteData(params, 1); // GAMMA

    // Positive Gamma
    const uint8_t pgamma[] = {0x0F, 0x1D, 0x1A, 0x0A, 0x0D, 0x07, 0x49, 0X66, 0x3B, 0x07, 0x11, 0x01, 0x09, 0x05, 0x04};
    ILI9341_WriteCmd(0xE0); ILI9341_WriteData(pgamma, 15);

    // Negative Gamma
    const uint8_t ngamma[] = {0x00, 0x18, 0x1D, 0x02, 0x0F, 0x04, 0x36, 0x13, 0x4C, 0x07, 0x13, 0x0F, 0x2E, 0x2F, 0x05};
    ILI9341_WriteCmd(0xE1); ILI9341_WriteData(ngamma, 15);

    // Orientation: BGR (0x08) | MY (0x80) | MV (0x20) => Landscape
    // Tùy chỉnh ở đây nếu hình bị ngược
    params[0] = 0x80 | 0x20 | 0x08;
    ILI9341_WriteCmd(ILI9341_MADCTL); ILI9341_WriteData(params, 1);

    // Exit Sleep & Display On
    ILI9341_WriteCmd(ILI9341_SLPOUT);
    HAL_Delay(120);
    ILI9341_WriteCmd(ILI9341_DISPON);
}

// Vẽ viền hình chữ nhật (Outline)
void ILI9341_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // Kiểm tra kích thước để tránh lỗi
    if (w == 0 || h == 0) return;

    // Vẽ 4 cạnh
    ILI9341_FillRect(color, x, y, w, 1);             // Cạnh trên (ngang)
    ILI9341_FillRect(color, x, y + h - 1, w, 1);     // Cạnh dưới (ngang)
    ILI9341_FillRect(color, x, y, 1, h);             // Cạnh trái (dọc)
    ILI9341_FillRect(color, x + w - 1, y, 1, h);     // Cạnh phải (dọc)
}

void ILI9341_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *data)
{
    if ((x >= LCD_W) || (y >= LCD_H)) return;
    if ((x + w - 1) >= LCD_W) return;
    if ((y + h - 1) >= LCD_H) return;

    // Thiết lập vùng cần vẽ
    ILI9341_SetWindow(x, y, x + w - 1, y + h - 1);

    ILI9341_DCX_HIGH();
    ILI9341_CSX_LOW();

    // Gửi dữ liệu ảnh qua SPI (w * h * 2 bytes vì mỗi pixel 2 byte)
    HAL_SPI_Transmit(ili9341_hspi, (uint8_t*)data, w * h * 2, HAL_MAX_DELAY);

    ILI9341_CSX_HIGH();
}
// Vẽ một vùng màu đặc (Hình chữ nhật)
void ILI9341_FillRect(uint16_t color, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if ((x + w) > LCD_W) w = LCD_W - x;
    if ((y + h) > LCD_H) h = LCD_H - y;
    if (w == 0 || h == 0) return;

    ILI9341_SetWindow(x, y, x + w - 1, y + h - 1);

    // Chuẩn bị buffer nhỏ trên stack để gửi một lần nhiều pixel
    // Giúp giảm overhead của hàm HAL_SPI_Transmit
    uint8_t high_byte = color >> 8;
    uint8_t low_byte = color & 0xFF;
    uint8_t line_buff[64]; // Buffer 32 pixels

    // Fill buffer
    for (int i = 0; i < 64; i += 2) {
        line_buff[i] = high_byte;
        line_buff[i+1] = low_byte;
    }

    uint32_t total_pixels = w * h;
    ILI9341_DCX_HIGH();
    ILI9341_CSX_LOW();

    while (total_pixels > 0) {
        // Gửi tối đa 32 pixel mỗi lần (64 bytes)
        uint16_t pixels_to_send = (total_pixels > 32) ? 32 : total_pixels;
        HAL_SPI_Transmit(ili9341_hspi, line_buff, pixels_to_send * 2, 100);
        total_pixels -= pixels_to_send;
    }
    ILI9341_CSX_HIGH();
}

// Vẽ toàn bộ frame từ buffer ảnh (RGB565)
// Buffer size phải = 320 * 240 * 2 bytes
void ILI9341_DrawFrame(const uint8_t *buffer, uint32_t nbytes)
{
    ILI9341_SetWindow(0, 0, LCD_W - 1, LCD_H - 1);
    ILI9341_DCX_HIGH();
    ILI9341_CSX_LOW();
    // Gửi toàn bộ buffer trong 1 lệnh SPI blocking
    // Nếu buffer quá lớn so với DMA max size (65535), HAL_SPI_Transmit vẫn xử lý tốt ở blocking mode
    HAL_SPI_Transmit(ili9341_hspi, (uint8_t*)buffer, nbytes, HAL_MAX_DELAY);
    ILI9341_CSX_HIGH();
}

// Vẽ viền hình chữ nhật (Bounding Box)
void Draw_BoundingBox(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ILI9341_FillRect(color, x, y, w, 1);             // Cạnh trên
    ILI9341_FillRect(color, x, y + h - 1, w, 1);     // Cạnh dưới
    ILI9341_FillRect(color, x, y, 1, h);             // Cạnh trái
    ILI9341_FillRect(color, x + w - 1, y, 1, h);     // Cạnh phải
}

// Vẽ ký tự
void LCD_DrawChar(int x, int y, char c, uint16_t color)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *chr = font5x7[c - 32];

    for (int col = 0; col < 5; col++) {
        uint8_t bits = chr[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                ILI9341_FillRect(color, x + col, y + row, 1, 1);
            }
        }
    }
}

// Viết chuỗi
void LCD_PrintString(int x, int y, const char *str, uint16_t color)
{
    while (*str) {
        LCD_DrawChar(x, y, *str, color);
        x += 6; // 5 width + 1 space
        str++;
    }
}
