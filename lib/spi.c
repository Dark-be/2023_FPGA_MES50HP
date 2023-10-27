#include <stdint.h>

#include "../include/spi.h"


void spi_init()
{
    SPI_REG(SPI_CTRL) = 0x0000;              // div_cnt = 0000 0 CPOL = 0, CPHA = 0 0
}

void spi_set_ss(uint8_t level)
{
    if (!level)
        SPI_REG(SPI_CTRL) |= 1 << 3;
    else
        SPI_REG(SPI_CTRL) &= ~(1 << 3);
}

void spi_write_byte(uint8_t data)
{
    SPI_REG(SPI_DATA) = data;
    SPI_REG(SPI_CTRL) |= 1 << 0;           // start
    while (SPI_REG(SPI_STATUS) & 0x1);     // wait transfer complete
}

void spi_write_bytes(uint8_t data[], uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
        spi_write_byte(data[i]);
}

uint8_t spi_read_byte()
{
    uint8_t data;

    SPI_REG(SPI_CTRL) |= 1 << 0;           // start
    while (SPI_REG(SPI_STATUS) & 0x1);     // wait transfer complete
    data = SPI_REG(SPI_DATA) & 0xff;       // readback data

    return data;
}

void spi_read_bytes(uint8_t data[], uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++)
        data[i] = spi_read_byte();
}
