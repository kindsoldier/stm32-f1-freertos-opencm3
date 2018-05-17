
/* $Id$ */

#include <libopencm3/stm32/i2c.h>
#include <stdlib.h>
#include <bh1750.h>
#include <i2creg.h>


void bh_setup(uint32_t i2c, uint8_t addr) {
    i2c_write_byte(i2c, addr, BH1750_POWER_ON);
    i2c_write_byte(i2c, addr, BH1750_RESET);
}

uint16_t bh_read(uint32_t i2c, uint8_t addr, uint8_t mode) {
    uint8_t data[BH1750_RES_LEN];
    i2c_read_seq(I2C1, addr, mode, data, BH1750_RES_LEN);
    return ((uint16_t)(data[0]) << 8) | data[1];
}

/* EOF */