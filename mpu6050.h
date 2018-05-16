
/*
  Original: MPU6050 lib 0x02
  Copyright (c) Davide Gironi, 2012
  Copyright (c) Oleg Borodin, 2018

  References:
  - most of the code is a port of the arduino mpu6050 library by Jeff Rowberg
    https://github.com/jrowberg/i2cdevlib
  - Mahony and Madgwick complementary filter for attitude estimation
    http://www.x-io.co.uk
*/


#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU_REG_XG_OFFS_TC       0x00
#define MPU_REG_YG_OFFS_TC       0x01
#define MPU_REG_ZG_OFFS_TC       0x02
#define MPU_REG_X_FINE_GAIN      0x03
#define MPU_REG_Y_FINE_GAIN      0x04
#define MPU_REG_Z_FINE_GAIN      0x05
#define MPU_REG_XA_OFFS_H        0x06
#define MPU_REG_XA_OFFS_L_TC     0x07
#define MPU_REG_YA_OFFS_H        0x08
#define MPU_REG_YA_OFFS_L_TC     0x09
#define MPU_REG_ZA_OFFS_H        0x0A
#define MPU_REG_ZA_OFFS_L_TC     0x0B
#define MPU_REG_XG_OFFS_USRH     0x13
#define MPU_REG_XG_OFFS_USRL     0x14
#define MPU_REG_YG_OFFS_USRH     0x15
#define MPU_REG_YG_OFFS_USRL     0x16
#define MPU_REG_ZG_OFFS_USRH     0x17
#define MPU_REG_ZG_OFFS_USRL     0x18
#define MPU_REG_SMPLRT_DIV       0x19
#define MPU_REG_CONFIG           0x1A
#define MPU_REG_GYRO_CONFIG      0x1B
#define MPU_REG_ACCEL_CONFIG     0x1C
#define MPU_REG_FF_THR           0x1D
#define MPU_REG_FF_DUR           0x1E
#define MPU_REG_MOT_THR          0x1F
#define MPU_REG_MOT_DUR          0x20
#define MPU_REG_ZRMOT_THR        0x21
#define MPU_REG_ZRMOT_DUR        0x22
#define MPU_REG_FIFO_EN          0x23
#define MPU_REG_I2C_MST_CTRL     0x24
#define MPU_REG_I2C_SLV0_ADDR    0x25
#define MPU_REG_I2C_SLV0_REG     0x26
#define MPU_REG_I2C_SLV0_CTRL    0x27
#define MPU_REG_I2C_SLV1_ADDR    0x28
#define MPU_REG_I2C_SLV1_REG     0x29
#define MPU_REG_I2C_SLV1_CTRL    0x2A
#define MPU_REG_I2C_SLV2_ADDR    0x2B
#define MPU_REG_I2C_SLV2_REG     0x2C
#define MPU_REG_I2C_SLV2_CTRL    0x2D
#define MPU_REG_I2C_SLV3_ADDR    0x2E
#define MPU_REG_I2C_SLV3_REG     0x2F
#define MPU_REG_I2C_SLV3_CTRL    0x30
#define MPU_REG_I2C_SLV4_ADDR    0x31
#define MPU_REG_I2C_SLV4_REG     0x32
#define MPU_REG_I2C_SLV4_DO      0x33
#define MPU_REG_I2C_SLV4_CTRL    0x34
#define MPU_REG_I2C_SLV4_DI      0x35
#define MPU_REG_I2C_MST_STATUS   0x36
#define MPU_REG_INT_PIN_CFG      0x37
#define MPU_REG_INT_ENABLE       0x38
#define MPU_REG_DMP_INT_STATUS   0x39
#define MPU_REG_INT_STATUS       0x3A
#define MPU_REG_ACCEL_XOUT_H     0x3B
#define MPU_REG_ACCEL_XOUT_L     0x3C
#define MPU_REG_ACCEL_YOUT_H     0x3D
#define MPU_REG_ACCEL_YOUT_L     0x3E
#define MPU_REG_ACCEL_ZOUT_H     0x3F
#define MPU_REG_ACCEL_ZOUT_L     0x40
#define MPU_REG_TEMP_OUT_H       0x41
#define MPU_REG_TEMP_OUT_L       0x42
#define MPU_REG_GYRO_XOUT_H      0x43
#define MPU_REG_GYRO_XOUT_L      0x44
#define MPU_REG_GYRO_YOUT_H      0x45
#define MPU_REG_GYRO_YOUT_L      0x46
#define MPU_REG_GYRO_ZOUT_H      0x47
#define MPU_REG_GYRO_ZOUT_L      0x48
#define MPU_REG_EXT_SENS_DATA_00 0x49
#define MPU_REG_EXT_SENS_DATA_01 0x4A
#define MPU_REG_EXT_SENS_DATA_02 0x4B
#define MPU_REG_EXT_SENS_DATA_03 0x4C
#define MPU_REG_EXT_SENS_DATA_04 0x4D
#define MPU_REG_EXT_SENS_DATA_05 0x4E
#define MPU_REG_EXT_SENS_DATA_06 0x4F
#define MPU_REG_EXT_SENS_DATA_07 0x50
#define MPU_REG_EXT_SENS_DATA_08 0x51
#define MPU_REG_EXT_SENS_DATA_09 0x52
#define MPU_REG_EXT_SENS_DATA_10 0x53
#define MPU_REG_EXT_SENS_DATA_11 0x54
#define MPU_REG_EXT_SENS_DATA_12 0x55
#define MPU_REG_EXT_SENS_DATA_13 0x56
#define MPU_REG_EXT_SENS_DATA_14 0x57
#define MPU_REG_EXT_SENS_DATA_15 0x58
#define MPU_REG_EXT_SENS_DATA_16 0x59
#define MPU_REG_EXT_SENS_DATA_17 0x5A
#define MPU_REG_EXT_SENS_DATA_18 0x5B
#define MPU_REG_EXT_SENS_DATA_19 0x5C
#define MPU_REG_EXT_SENS_DATA_20 0x5D
#define MPU_REG_EXT_SENS_DATA_21 0x5E
#define MPU_REG_EXT_SENS_DATA_22 0x5F
#define MPU_REG_EXT_SENS_DATA_23 0x60
#define MPU_REG_MOT_DETECT_STATUS    0x61
#define MPU_REG_I2C_SLV0_DO      0x63
#define MPU_REG_I2C_SLV1_DO      0x64
#define MPU_REG_I2C_SLV2_DO      0x65
#define MPU_REG_I2C_SLV3_DO      0x66
#define MPU_REG_I2C_MST_DELAY_CTRL   0x67
#define MPU_REG_SIGNAL_PATH_RESET    0x68
#define MPU_REG_MOT_DETECT_CTRL      0x69
#define MPU_REG_USER_CTRL        0x6A
#define MPU_REG_PWR_MGMT_1       0x6B
#define MPU_REG_PWR_MGMT_2       0x6C
#define MPU_REG_BANK_SEL         0x6D
#define MPU_REG_MEM_START_ADDR   0x6E
#define MPU_REG_MEM_R_W          0x6F
#define MPU_REG_DMP_CFG_1        0x70
#define MPU_REG_DMP_CFG_2        0x71
#define MPU_REG_FIFO_COUNTH      0x72
#define MPU_REG_FIFO_COUNTL      0x73
#define MPU_REG_FIFO_R_W         0x74
#define MPU_REG_WHO_AM_I         0x75


/* MPU_REG_CONFIG 0x1A */
#define MPU_EXT_SYNC_BASE           3
#define MPU_EXT_SYNC_LEN            3

#define MPU_EXT_SYNC_DISABLED       0
#define MPU_EXT_SYNC_TEMP_OUT_L     1
#define MPU_EXT_SYNC_GYRO_XOUT_L    2
#define MPU_EXT_SYNC_GYRO_YOUT_L    3
#define MPU_EXT_SYNC_GYRO_ZOUT_L    4
#define MPU_EXT_SYNC_ACCEL_XOUT_L   5
#define MPU_EXT_SYNC_ACCEL_YOUT_L   6
#define MPU_EXT_SYNC_ACCEL_ZOUT_L   7

#define MPU_CFG_DLPF_CFG_BASE       0
#define MPU_CFG_DLPF_CFG_LEN        3

#define MPU_DLPF_BW_256         0
#define MPU_DLPF_BW_188         1
#define MPU_DLPF_BW_98          2
#define MPU_DLPF_BW_42          3
#define MPU_DLPF_BW_20          4
#define MPU_DLPF_BW_10          5
#define MPU_DLPF_BW_5           6

/* GYRO_CONFIG 0x1B */
#define MPU_GYRO_FS_BASE        3
#define MPU_GYRO_FS_LEN         2

#define MPU_GYRO_FS_250         0
#define MPU_GYRO_FS_500         1
#define MPU_GYRO_FS_1000        2
#define MPU_GYRO_FS_2000        3

#define MPU_GCONFIG_ZA_ST_BIT   5
#define MPU_GCONFIG_YA_ST_BIT   6
#define MPU_GCONFIG_XA_ST_BIT   7

/* ACCEL_CONFIG 0x1C */
#define MPU_ACCEL_FS_BASE       3
#define MPU_ACCEL_FS_LEN        2
#define MPU_ACCEL_FS_2          0
#define MPU_ACCEL_FS_4          1
#define MPU_ACCEL_FS_8          2
#define MPU_ACCEL_FS_16         3

#define MPU_ACONFIG_ZA_ST_BIT   5
#define MPU_ACONFIG_YA_ST_BIT   6
#define MPU_ACONFIG_XA_ST_BIT   7

/* FIFO_EN 0x23 */
#define MPU_TEMP_FIFO_EN_BIT    7
#define MPU_XG_FIFO_EN_BIT      6
#define MPU_YG_FIFO_EN_BIT      5
#define MPU_ZG_FIFO_EN_BIT      4
#define MPU_ACCEL_FIFO_EN_BIT   3
#define MPU_SLV2_FIFO_EN_BIT    2
#define MPU_SLV1_FIFO_EN_BIT    1
#define MPU_SLV0_FIFO_EN_BIT    0


/* I2C_MST_CTRL 0x24 */
#define MPU_MULT_MST_EN_BIT     7
#define MPU_WAIT_FOR_ES_BIT     6
#define MPU_SLV_3_FIFO_EN_BIT   5
#define MPU_I2C_MST_P_NSR_BIT   4

#define MPU_I2C_CLOCK_DIV_BASE      0
#define MPU_I2C_CLOCK_DIV_LEN       4

#define MPU_I2C_CLOCK_DIV_348       0x0
#define MPU_I2C_CLOCK_DIV_333       0x1
#define MPU_I2C_CLOCK_DIV_320       0x2
#define MPU_I2C_CLOCK_DIV_308       0x3
#define MPU_I2C_CLOCK_DIV_296       0x4
#define MPU_I2C_CLOCK_DIV_286       0x5
#define MPU_I2C_CLOCK_DIV_276       0x6
#define MPU_I2C_CLOCK_DIV_267       0x7
#define MPU_I2C_CLOCK_DIV_258       0x8
#define MPU_I2C_CLOCK_DIV_500       0x9
#define MPU_I2C_CLOCK_DIV_471       0xA
#define MPU_I2C_CLOCK_DIV_444       0xB
#define MPU_I2C_CLOCK_DIV_421       0xC
#define MPU_I2C_CLOCK_DIV_400       0xD
#define MPU_I2C_CLOCK_DIV_381       0xE
#define MPU_I2C_CLOCK_DIV_364       0xF


/*INT_PIN_CFG 0x37 */
#define MPU_INTCFG_INT_LEVEL_BIT        7
#define MPU_INTCFG_INT_OPEN_BIT         6
#define MPU_INTCFG_LATCH_INT_EN_BIT     5
#define MPU_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU_INTCFG_CLKOUT_EN_BIT        0

/* INT_ENABLE 0x38, INT_STATUS 0x39 */
#define MPU_INTERRUPT_FF_BIT            7
#define MPU_INTERRUPT_MOT_BIT           6
#define MPU_INTERRUPT_ZMOT_BIT          5
#define MPU_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU_INTERRUPT_DMP_INT_BIT       1
#define MPU_INTERRUPT_DATA_RDY_BIT      0

/* SIGNAL_PATH_RESET 0x68 */
#define MPU_PATHRESET_GYRO_RESET_BIT    2
#define MPU_PATHRESET_ACCEL_RESET_BIT   1
#define MPU_PATHRESET_TEMP_RESET_BIT    0

/* USER_CTRL 0x6A */
#define MPU_USERCTRL_DMP_EN_BIT             7
#define MPU_USERCTRL_FIFO_EN_BIT            6
#define MPU_USERCTRL_I2C_MST_EN_BIT         5
#define MPU_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU_USERCTRL_DMP_RESET_BIT          3
#define MPU_USERCTRL_FIFO_RESET_BIT         2
#define MPU_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU_USERCTRL_SIG_COND_RESET_BIT     0

/* PWR_MGMT_1 0x6B */
#define MPU_PWR1_DEVICE_RESET_BIT   7
#define MPU_PWR1_SLEEP_BIT          6
#define MPU_PWR1_CYCLE_BIT          5
#define MPU_PWR1_TEMP_DIS_BIT       3

#define MPU_PWR1_CLKSEL_BASE        0
#define MPU_PWR1_CLKSEL_LEN         3

#define MPU_PWR1_CLKSEL_INTERNAL          0
#define MPU_PWR1_CLKSEL_PLL_XGYRO         1
#define MPU_PWR1_CLKSEL_PLL_YGYRO         2
#define MPU_PWR1_CLKSEL_PLL_ZGYRO         3
#define MPU_PWR1_CLKSEL_PLL_EXT32K        4
#define MPU_PWR1_CLKSEL_PLL_EXT19M        5
#define MPU_PWR1_CLKSEL_KEEP_RESET        7

/* PWR_MGMT_2 0x6C */
#define MPU_PWR2_LP_WAKE_CTRL_BASE      6
#define MPU_PWR2_LP_WAKE_CTRL_LEN       2

#define MPU_PWR2_WAKE_FREQ_1P25         0
#define MPU_PWR2_WAKE_FREQ_2P5          1
#define MPU_PWR2_WAKE_FREQ_5            2
#define MPU_PWR2_WAKE_FREQ_10           3

#define MPU_PWR2_STBY_XA_BIT            5
#define MPU_PWR2_STBY_YA_BIT            4
#define MPU_PWR2_STBY_ZA_BIT            3
#define MPU_PWR2_STBY_XG_BIT            2
#define MPU_PWR2_STBY_YG_BIT            1
#define MPU_PWR2_STBY_ZG_BIT            0


#ifndef MPU_GETATTITUDE
#define MPU_GETATTITUDE 1
#endif

/* Definitions for raw data gyro and acc scale */
#define MPU_GYRO_FS         MPU_GYRO_FS_2000
#define MPU_ACCEL_FS        MPU_ACCEL_FS_2

#define MPU_GYRO_LSB_250    131.0
#define MPU_GYRO_LSB_500    65.5
#define MPU_GYRO_LSB_1000   32.8
#define MPU_GYRO_LSB_2000   16.4

#define MPU_ACCEL_LSB_2     16384.0
#define MPU_ACCEL_LSB_4      8192.0
#define MPU_ACCEL_LSB_8      4096.0
#define MPU_ACCEL_LSB_16     2048.0

#if (MPU_GYRO_FS == MPU_GYRO_FS_250)
    #define MPU_GGAIN   MPU_GYRO_LSB_250
#elif (MPU_GYRO_FS == MPU_GYRO_FS_500)
    #define MPU_GGAIN   MPU_GYRO_LSB_500
#elif (MPU_GYRO_FS == MPU_GYRO_FS_1000)
    #define MPU_GGAIN   MPU_GYRO_LSB_1000
#elif (MPU_GYRO_FS == MPU_GYRO_FS_2000)
    #define MPU_GGAIN   MPU_GYRO_LSB_2000
#endif

#if (MPU_ACCEL_FS == MPU_ACCEL_FS_2)
    #define MPU_AGAIN MPU_ACCEL_LSB_2
#elif (MPU_ACCEL_FS == MPU_ACCEL_FS_4)
    #define MPU_AGAIN MPU_ACCEL_LSB_4
#elif (MPU_ACCEL_FS == MPU_ACCEL_FS_8)
    #define MPU_AGAIN MPU_ACCEL_LSB_8
#elif (MPU_ACCEL_FS == MPU_ACCEL_FS_16)
    #define MPU_AGAIN MPU_ACCEL_LSB_16
#endif

#define MPU_AXOFFSET        0
#define MPU_AYOFFSET        0
#define MPU_AZOFFSET        0

#define MPU_GXOFFSET        0
#define MPU_GYOFFSET        0
#define MPU_GZOFFSET        0

typedef struct mpu {
    uint32_t i2c;
    uint8_t addr;
    double q0;
    double q1;
    double q2;
    double q3;
    double integralFBx;
    double integralFBy;
    double integralFBz;
} mpu_t;

void mpu_i2c_setup(void);
void mpu_read_reg(mpu_t * mpu, uint8_t reg, uint8_t * data);
int16_t mpu_read_seq(mpu_t * mpu, uint8_t reg, uint8_t * data, uint8_t len);
void mpu_write_reg(mpu_t *mpu, uint8_t reg, uint8_t data);
void mpu_write_bits(mpu_t *mpu, uint8_t reg, uint8_t base, uint8_t len, uint8_t data);
void mpu_setup(mpu_t * mpu);
void mpu_get_raw_data(mpu_t * mpu, int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz);
void mpu_get_conv_data(mpu_t * mpu, double *axg, double *ayg, double *azg, double *gxds, double *gyds, double *gzds);
void mpu_update_quaternion(mpu_t * mpu);
void mpu_mahony_update(mpu_t * qn, double gx, double gy, double gz, double ax, double ay, double az);
void mpu_get_roll_pitch_yaw(mpu_t * qn, double *roll, double *pitch, double *yaw);

#endif
