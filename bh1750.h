
/* $Id$ */

#ifndef BH1750_H_XYZ
#define BH1750_H_XYZ

#define BH1750_POWER_ON    0x01
#define BH1750_RESET       0x07
#define BH1750_POWER_DOWN  0x00


#define BH1750_CONT_HR_MODE             0x10 /* Measurement at 1lx resolution, time 120ms */
#define BH1750_CONT_HR_MODE_2           0x11 /* Measurement at 0.5lx resolution, time 120ms */
#define BH1750_CONT_LR_MODE             0x13 /* Measurement at 4lx resolution, time is approx 16ms */
#define BH1750_ONE_TIME_HR_MODE         0x20 /* Measurement at 1lx resolution, time is approx 120ms */
#define BH1750_ONE_TIME_HR_MODE_2       0x21 /* Measurement at 0.5lx resolution, time is approx 120ms */
#define BH1750_ONE_TIME_LR_MODE         0x23 /* Measurement at 1lx resolution, time is approx 120ms. */

#define BH1750_ADDR       0x23
#define BH1750_RES_LEN    2

void bh_setup(uint32_t i2c, uint8_t addr);
uint16_t bh_read(uint32_t i2c, uint8_t addr, uint8_t mode);

#endif
