/**
 * @file ssd1306draw.c
 * @author Michal Piekos
 * @brief SSD1306 tracking data plotting library for STM32 written using STM32 HAL
 * @version 0.1
 * @date 2023-06-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "ssd1306draw.h"


/*****************************************************************
 * Global variables
******************************************************************/

/**
 * @brief Center point for dial
 */
coordinate_t p0 = {31, 31};

/**
 * @brief Screen buffer
 */
uint8_t screen_buffer[SSD1306_NUM_PAGES][128] = {0};

/**
 * @brief Array of char* with menu positions
 */
char menuposition[SSD1306_MENU_SIZE][SSD1306_MENU_ITEM_MAX_LENGTH] = {
	"Lokalizacja", "Kalibracja"
};

/**
 * @brief Font table. Each character has width 5 and height 8.
 * First index represents character.
 */
uint8_t ssd1306_font[][FONT_SIZE] = {

	0x00, 0x00, 0x00, 0x00, 0x00,   // space
    0x00, 0x00, 0x2f, 0x00, 0x00,   // !
    0x00, 0x07, 0x00, 0x07, 0x00,   // "
    0x14, 0x7f, 0x14, 0x7f, 0x14,   // #
    0x24, 0x2a, 0x7f, 0x2a, 0x12,   // $
    0x23, 0x13, 0x08, 0x64, 0x62,   // %
    0x36, 0x49, 0x55, 0x22, 0x50,   // &
    0x00, 0x05, 0x03, 0x00, 0x00,   // '
    0x00, 0x1c, 0x22, 0x41, 0x00,   // (
    0x00, 0x41, 0x22, 0x1c, 0x00,   // )
    0x14, 0x08, 0x3E, 0x08, 0x14,   // *
    0x08, 0x08, 0x3E, 0x08, 0x08,   // +
    0x00, 0x00, 0xA0, 0x60, 0x00,   // ,
    0x08, 0x08, 0x08, 0x08, 0x08,   // -
    0x00, 0x60, 0x60, 0x00, 0x00,   // .
    0x20, 0x10, 0x08, 0x04, 0x02,   // /

    0x3E, 0x51, 0x49, 0x45, 0x3E,   // 0
    0x00, 0x42, 0x7F, 0x40, 0x00,   // 1
    0x42, 0x61, 0x51, 0x49, 0x46,   // 2
    0x21, 0x41, 0x45, 0x4B, 0x31,   // 3
    0x18, 0x14, 0x12, 0x7F, 0x10,   // 4
    0x27, 0x45, 0x45, 0x45, 0x39,   // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,   // 6
    0x01, 0x71, 0x09, 0x05, 0x03,   // 7
    0x36, 0x49, 0x49, 0x49, 0x36,   // 8
    0x06, 0x49, 0x49, 0x29, 0x1E,   // 9

    0x00, 0x36, 0x36, 0x00, 0x00,   // :
    0x00, 0x56, 0x36, 0x00, 0x00,   // ;
    0x08, 0x14, 0x22, 0x41, 0x00,   // <
    0x14, 0x14, 0x14, 0x14, 0x14,   // =
    0x00, 0x41, 0x22, 0x14, 0x08,   // >
    0x02, 0x01, 0x51, 0x09, 0x06,   // ?
    0x32, 0x49, 0x59, 0x51, 0x3E,   // @

    0x7C, 0x12, 0x11, 0x12, 0x7C,   // A
    0x7F, 0x49, 0x49, 0x49, 0x36,   // B
    0x3E, 0x41, 0x41, 0x41, 0x22,   // C
    0x7F, 0x41, 0x41, 0x22, 0x1C,   // D
    0x7F, 0x49, 0x49, 0x49, 0x41,   // E
    0x7F, 0x09, 0x09, 0x09, 0x01,   // F
    0x3E, 0x41, 0x49, 0x49, 0x7A,   // G
    0x7F, 0x08, 0x08, 0x08, 0x7F,   // H
    0x00, 0x41, 0x7F, 0x41, 0x00,   // I
    0x20, 0x40, 0x41, 0x3F, 0x01,   // J
    0x7F, 0x08, 0x14, 0x22, 0x41,   // K
    0x7F, 0x40, 0x40, 0x40, 0x40,   // L
    0x7F, 0x02, 0x0C, 0x02, 0x7F,   // M
    0x7F, 0x04, 0x08, 0x10, 0x7F,   // N
    0x3E, 0x41, 0x41, 0x41, 0x3E,   // O
    0x7F, 0x09, 0x09, 0x09, 0x06,   // P
    0x3E, 0x41, 0x51, 0x21, 0x5E,   // Q
    0x7F, 0x09, 0x19, 0x29, 0x46,   // R
    0x46, 0x49, 0x49, 0x49, 0x31,   // S
    0x01, 0x01, 0x7F, 0x01, 0x01,   // T
    0x3F, 0x40, 0x40, 0x40, 0x3F,   // U
    0x1F, 0x20, 0x40, 0x20, 0x1F,   // V
    0x3F, 0x40, 0x38, 0x40, 0x3F,   // W
    0x63, 0x14, 0x08, 0x14, 0x63,   // X
    0x07, 0x08, 0x70, 0x08, 0x07,   // Y
    0x61, 0x51, 0x49, 0x45, 0x43,   // Z

    0x00, 0x7F, 0x41, 0x41, 0x00,   // [
    0x55, 0xAA, 0x55, 0xAA, 0x55,   // Backslash (Checker pattern)
    0x00, 0x41, 0x41, 0x7F, 0x00,   // ]
    0x04, 0x02, 0x01, 0x02, 0x04,   // ^
    0x40, 0x40, 0x40, 0x40, 0x40,   // _
    0x00, 0x03, 0x05, 0x00, 0x00,   // `

    0x20, 0x54, 0x54, 0x54, 0x78,   // a
    0x7F, 0x48, 0x44, 0x44, 0x38,   // b
    0x38, 0x44, 0x44, 0x44, 0x20,   // c
    0x38, 0x44, 0x44, 0x48, 0x7F,   // d
    0x38, 0x54, 0x54, 0x54, 0x18,   // e
    0x08, 0x7E, 0x09, 0x01, 0x02,   // f
    0x18, 0xA4, 0xA4, 0xA4, 0x7C,   // g
    0x7F, 0x08, 0x04, 0x04, 0x78,   // h
    0x00, 0x44, 0x7D, 0x40, 0x00,   // i
    0x40, 0x80, 0x84, 0x7D, 0x00,   // j
    0x7F, 0x10, 0x28, 0x44, 0x00,   // k
    0x00, 0x41, 0x7F, 0x40, 0x00,   // l
    0x7C, 0x04, 0x18, 0x04, 0x78,   // m
    0x7C, 0x08, 0x04, 0x04, 0x78,   // n
    0x38, 0x44, 0x44, 0x44, 0x38,   // o
    0xFC, 0x24, 0x24, 0x24, 0x18,   // p
    0x18, 0x24, 0x24, 0x18, 0xFC,   // q
    0x7C, 0x08, 0x04, 0x04, 0x08,   // r
    0x48, 0x54, 0x54, 0x54, 0x20,   // s
    0x04, 0x3F, 0x44, 0x40, 0x20,   // t
    0x3C, 0x40, 0x40, 0x20, 0x7C,   // u
    0x1C, 0x20, 0x40, 0x20, 0x1C,   // v
    0x3C, 0x40, 0x30, 0x40, 0x3C,   // w
    0x44, 0x28, 0x10, 0x28, 0x44,   // x
    0x1C, 0xA0, 0xA0, 0xA0, 0x7C,   // y
    0x44, 0x64, 0x54, 0x4C, 0x44,   // z

    0x00, 0x10, 0x7C, 0x82, 0x00,   // {
    0x00, 0x00, 0xFF, 0x00, 0x00,   // |
    0x00, 0x82, 0x7C, 0x10, 0x00,   // }
    0x00, 0x06, 0x09, 0x09, 0x06    // ~ (Degrees)
};


/*****************************************************************/


/*****************************************************************
 * Core Functions
******************************************************************/

/**
 * @brief Initialization of SSD1306. Needs to be executed once before executing
 * other functions from this library.
 *  
 * @param hi2c. Pointer to I2C_HandleTypeDef
 */
void ssd1306_init(I2C_HandleTypeDef *hi2c)
{
    _ssd1306_tx_command(hi2c, SSD1306_DISPLAY_OFF);
    _ssd1306_tx_command(hi2c, SSD1306_SET_MUX_RATIO);
    #if (SSD1306_NUM_PAGES == 4)
        _ssd1306_tx_command(hi2c, 0x1F);
    #elif (SSD1306_NUM_PAGES == 8)
        _ssd1306_tx_command(hi2c, 0x3F);
    #elif (SSD1306_NUM_PAGES == 16)
        _ssd1306_tx_command(hi2c, 0x3F);
    #else
        #error "Only 32, 64, or 128 lines of height are supported!"
    #endif
    _ssd1306_tx_command(hi2c, SSD1306_SET_DISPLAY_OFFSET);
    _ssd1306_tx_command(hi2c, 0x00);
    _ssd1306_tx_command(hi2c, SSD1306_SET_DISPLAY_START_LINE);
    _ssd1306_tx_command(hi2c, SSD1306_SET_SEGMENT_REMAP | 0x1);  // | 0x1
    _ssd1306_tx_command(hi2c, SSD1306_COM_OUTPUT_SCAN_DOWN);
    _ssd1306_tx_command(hi2c, SSD1306_SET_COM_PINS);
    #if (SSD1306_NUM_PAGES == 4)
        _ssd1306_tx_command(hi2c, 0x02);
    #elif (SSD1306_NUM_PAGES == 8)
        _ssd1306_tx_command(hi2c, 0x12);
    #elif (SSD1306_NUM_PAGES == 16)
        _ssd1306_tx_command(hi2c, 0x12);
    #else
        #error "Only 32, 64, or 128 lines of height are supported!"
    #endif
    _ssd1306_tx_command(hi2c, SSD1306_SET_CONTRAST_CONTROL);
    _ssd1306_tx_command(hi2c, 0xB0);
    _ssd1306_set_addressing_mode(hi2c, SSD1306_HORIZONTAL_ADDRESSING_MODE);
    _ssd1306_tx_command(hi2c, SSD1306_DISABLE_ENTIRE_DISPLAY_ON);
    _ssd1306_tx_command(hi2c, SSD1306_SET_NORMAL_DISPLAY);
    _ssd1306_tx_command(hi2c, SSD1306_SET_OSC_FREQUENCY);
    _ssd1306_tx_command(hi2c, 0x80);
    _ssd1306_tx_command(hi2c, SSD1306_ENABLE_CHARGE_PUMP);
    _ssd1306_tx_command(hi2c, 0x14);
    _ssd1306_tx_command(hi2c, SSD1306_DISPLAY_ON);

//    ssd1306_clear();
    // ssd1306_printchar('a', 1, 1);
    ssd1306_commit(hi2c);
}


/**
 * @brief Clear screen
 * 
 */
void ssd1306_clear(void)
{
	uint8_t *buffer_ptr = *screen_buffer;
	for (uint16_t i=0; i < SSD1306_BUFFER_SIZE; i++) {
		*(buffer_ptr + i) = 0x0;
	}
}


/**
 * @brief Clear one row of data
 * 
 * @param row to be cleared
 */
void ssd1306_clearrow(uint8_t row)
{
    for (uint8_t i=0; i < 128; i++) {
        screen_buffer[row][i] = 0x00;
    }
}

/**
 * @brief Draw line between two coordinates using Bresenham algorithm
 *
 * @param p0
 * @param p1
 */
void ssd1306_line(coordinate_t p0, coordinate_t p1)
{
	//restriction a.x < b.x  and 0 < H/W < 1
	// Make sure p0 is leftmost point
	if (p1.x < p0.x) {
		_swap_points(&p0, &p1);
	}
	// Reverse p1.y if smaller than p0.y
	uint8_t mirrored = 0;
	if (p1.y < p0.y) {
		p1.y += _mirror_y(p0.y, p1.y);
		mirrored = 1;
	}
	// Swap x and y coordinates if width < height
	uint8_t swapped = 0;
	uint8_t y0_original = p0.y;
	if ((p1.x - p0.x) < (p1.y - p0.y)) {
		_swap_xy(&p0, &p1);
		swapped = 1;
	}

	int y = p0.y;
	int w = p1.x - p0.x;
	int h = p1.y - p0.y;
	int f = 2 * h - w; // current error term

	for (int x = p0.x; x <= p1.x; x++)
	{
		uint8_t x2show = x;
		uint8_t y2show = y;
		if (swapped) {
			x2show = y;
			y2show = x;
		}
		if (mirrored) {
			y2show += _mirror_y(y0_original, y2show);
		}
		screen_buffer[(y2show / 8)][x2show] |= (1 << (y2show % 8));

		if (f < 0)
		{
				f = f + 2 * h;
		}
		else {
				y++;
				f = f + 2 * (h - w);
		}
	}
}

/**
 * @brief Commits internal buffer to SSD1306 memory.
 *
 * @param *hi2c STM32 HAL Handle to I2C interface
 */
void ssd1306_commit(I2C_HandleTypeDef *hi2c)
{
	const uint16_t buffer_len = SSD1306_BUFFER_SIZE;
	_ssd1306_tx_command(hi2c, SSD1306_SET_COLUMN_ADDRESS);
	_ssd1306_tx_command(hi2c, 0);
	_ssd1306_tx_command(hi2c, 127);
	_ssd1306_tx_command(hi2c, SSD1306_SET_PAGE_ADDRESS);
	_ssd1306_tx_command(hi2c, 0);
	_ssd1306_tx_command(hi2c, SSD1306_NUM_PAGES - 1);

    while (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY) ;
    
    #if (SSD1306_USE_DMA == 1)
    HAL_I2C_Mem_Write_DMA(hi2c, (SSD1306_ADDRESS<<1), 0x40, I2C_MEMADD_SIZE_8BIT, *screen_buffer, buffer_len);
    #else
	HAL_I2C_Mem_Write(hi2c, (SSD1306_ADDRESS<<1), 0x40, I2C_MEMADD_SIZE_8BIT, *screen_buffer, buffer_len, 50);
    #endif
}

/**
 * @brief Calculate coordinate of the end of the line based on its beginning, angle and length
 *
 * @param angle - in degrees
 * @param p0 - coordinates of the start point
 * @param length
 */
coordinate_t ssd1306_endpoint(uint16_t angle, coordinate_t p0, int length)
{
	coordinate_t p1;
	double radians = (M_PI / 180.0) * angle;
	p1.x = round(p0.x + (length * cos(radians)));
	p1.y = round(p0.y + (length * sin(radians)));
	return p1;
}


/**
 * @brief Prints character in given row (page in SSD1306 nomenclature) and column
 * 
 * @param character one character to be printed
 * @param row from 0 to 7
 * @param column from 0 to 122. Character needs 5 columns to be printed. 127 - 5 = 122.
 * Hence the maximum column number.
 */
void ssd1306_printchar(uint8_t character, uint8_t row, uint8_t column)
{
    character -= 0x20; // table starts from space which is 0x20 in ASCII table
    if (column + 5 < 128) {
			for (uint8_t active_column = column; active_column < column + 5; active_column++) {
				screen_buffer[row][active_column] |= ssd1306_font[character][active_column - column];
			}
    }
}


/**
 * @brief Print string starting from row, column
 * 
 * @param str string to be printed
 * @param row row where printing starts
 * @param column column where printing starts
 */
void ssd1306_printstring(char *str, uint8_t row, uint8_t column)
{
	char *c = str;
	while (*c) {
		ssd1306_printchar(*c++, row, column);
		column += 6;
	}
}

void ssd1306_drawmenu(uint8_t active_item)
{
	char tempstring[SSD1306_MENU_ITEM_MAX_LENGTH + 3];
	uint8_t spacer = 8 / SSD1306_MENU_SIZE;

	for (uint8_t i=0; i<SSD1306_MENU_SIZE; i++) {
		if (active_item == i) {
			sprintf(tempstring, ">> %s", menuposition[i]);
		}
		else {
			sprintf(tempstring, "   %s", menuposition[i]);
		}
		ssd1306_printstring(tempstring, i * spacer, 6);
	}
}

void ssd1306_printint(int16_t number, uint8_t row, uint8_t column)
{
	char str[7];
	sprintf(str, "%d", number);
	ssd1306_printstring(str, row, column);
}

/**
 * @brief Print integer in hexadecimal format
 * 
 * @param number to be printed
 * @param row 0-7 
 * @param column 0-127
 */
void ssd1306_printhex(int16_t number, uint8_t row, uint8_t column)
{
	char str[7];
	sprintf(str, "0x%x", number);
	ssd1306_printstring(str, row, column);
}


/*****************************************************************/


/*****************************************************************
 * Support functions
******************************************************************/

/**
 * @brief Returns value by which b should be mirrored vs a
 *
 */
int8_t _mirror_y(uint8_t a, uint8_t b)
{
	return 2 * (a - b);
}

void _swap_xy(coordinate_t *a, coordinate_t *b)
{
	uint8_t xa_temp = a->x;
	uint8_t xb_temp = b->x;
	a->x = a->y;
	a->y = xa_temp;
	b->x = b->y;
	b->y = xb_temp;
}

void _swap_points(coordinate_t *a, coordinate_t *b)
{
	uint8_t x_temp = a->x;
	uint8_t y_temp = a->y;
	a->x = b->x;
	a->y = b->y;
	b->x = x_temp;
	b->y = y_temp;
}

/**
 * @brief Set SSD1306 memory addressing mode.
 *
 * @param hi2c. Pointer to I2C_HandleTypeDef
 * @param mode. Choices: [SSD1306_HORIZONTAL_ADDRESSING_MODE | SSD1306_VERTICAL_ADDRESSING_MODE | SSD1306_PAGE_ADDRESSING_MODE]
 */
void _ssd1306_set_addressing_mode(I2C_HandleTypeDef *hi2c, uint8_t mode)
{
	_ssd1306_tx_command(hi2c, SSD1306_SET_MEMORY_ADDRESSING_MODE);
	_ssd1306_tx_command(hi2c, mode);
}

/**
 * @brief Transmit SSD1306 command. Command is prefixed with 0x0.
 * Transmission is encapsulated with START and STOP conditions.
 * 
 * @param hi2c. Pointer to I2C_HandleTypeDef
 * @param command. 1 byte command.
 */
void _ssd1306_tx_command(I2C_HandleTypeDef *hi2c, uint8_t command)
{
    HAL_I2C_Mem_Write(hi2c, (SSD1306_ADDRESS<<1), SSD1306_COMMAND, I2C_MEMADD_SIZE_8BIT, &command, 1, 50);
}

/*****************************************************************/
