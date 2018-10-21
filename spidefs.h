#define SPI_HEADER(dir,size,count,dis) (((count)<<4)|((size)<<2)|(dir)|((dis)<<13))

#define SPI_SS_ASSERT		0
#define SPI_SS_DISASSERT	1
#define SPI_DIR_NONE	0
#define SPI_DIR_TX		1
#define SPI_DIR_RX		2
#define SPI_DIR_BOTH	3

#define SPI_BYTES		1
#define SPI_PAGES		2

#define SPI_END				0xF000
