#define GC9A01_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 240

#define TFT_MISO  -1  // Not used
#define TFT_MOSI  13
#define TFT_SCLK  14
#define TFT_CS     5
#define TFT_DC     2
#define TFT_RST    -1

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8

#define SPI_FREQUENCY  10000000
#define SPI_READ_FREQUENCY  20000000