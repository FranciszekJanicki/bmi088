#ifndef BMI088_BMI088_CONFIG_H
#define BMI088_BMI088_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_TEMP_SCALE 0.125F
#define BMI088_ACC_PERIOD_INC_US 39.0625F

typedef float float32_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} vec3_int16_t;

typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} vec3_float32_t;

typedef enum {
    BMI088_ERR_OK = 0,
    BMI088_ERR_FAIL = 1 << 0,
    BMI088_ERR_NULL = 1 << 1,
} bmi088_err_t;

typedef enum {
    BMI088_ACC_REG_ADDR_ACC_SOFTRESET = 0x7E,
    BMI088_ACC_REG_ADDR_ACC_PWR_CTRL = 0x7D,
    BMI088_ACC_REG_ADDR_ACC_PWR_CONF = 0x7C,
    BMI088_ACC_REG_ADDR_ACC_SELF_TEST = 0x6D,
    BMI088_ACC_REG_ADDR_INT_MAP_DATA = 0x58,
    BMI088_ACC_REG_ADDR_INT2_IO_CTRL = 0x54,
    BMI088_ACC_REG_ADDR_INT1_IO_CTRL = 0x53,
    BMI088_ACC_REG_ADDR_FIFO_CONFIG_1 = 0x49,
    BMI088_ACC_REG_ADDR_FIFO_CONFIG_0 = 0x48,
    BMI088_ACC_REG_ADDR_FIFO_WTM_1 = 0x47,
    BMI088_ACC_REG_ADDR_FIFO_WTM_0 = 0x46,
    BMI088_ACC_REG_ADDR_FIFO_DOWNS = 0x45,
    BMI088_ACC_REG_ADDR_ACC_RANGE = 0x41,
    BMI088_ACC_REG_ADDR_ACC_CONF = 0x40,
    BMI088_ACC_REG_ADDR_FIFO_DATA = 0x26,
    BMI088_ACC_REG_ADDR_FIFO_LENGTH_1 = 0x25,
    BMI088_ACC_REG_ADDR_FIFO_LENGTH_0 = 0x24,
    BMI088_ACC_REG_ADDR_TEMP_LSB = 0x23,
    BMI088_ACC_REG_ADDR_TEMP_MSB = 0x22,
    BMI088_ACC_REG_ADDR_ACC_INT_STAT = 0x1D,
    BMI088_ACC_REG_ADDR_SENSORTIME_2 = 0x1A,
    BMI088_ACC_REG_ADDR_SENSORTIME_1 = 0x19,
    BMI088_ACC_REG_ADDR_SENSORTIME_0 = 0x18,
    BMI088_ACC_REG_ADDR_ACC_Z_MSB = 0x17,
    BMI088_ACC_REG_ADDR_ACC_Z_LSB = 0x16,
    BMI088_ACC_REG_ADDR_ACC_Y_MSB = 0x15,
    BMI088_ACC_REG_ADDR_ACC_Y_LSB = 0x14,
    BMI088_ACC_REG_ADDR_ACC_X_MSB = 0x13,
    BMI088_ACC_REG_ADDR_ACC_X_LSB = 0x12,
    BMI088_ACC_REG_ADDR_ACC_STATUS = 0x03,
    BMI088_ACC_REG_ADDR_ACC_ERR_REG = 0x02,
    BMI088_ACC_REG_ADDR_ACC_CHIP_ID = 0x00,
} bmi088_acc_reg_addr_t;

typedef enum {
    BMI088_ACC_POWER_MODE_NORMAL,
    BMI088_ACC_POWER_MODE_SUSPEND,
} bmi088_acc_power_mode_t;

typedef enum {
    BMI088_ACC_ODR_12HZ5,
    BMI088_ACC_ODR_25HZ,
    BMI088_ACC_ODR_50HZ,
    BMI088_ACC_ODR_100HZ,
    BMI088_ACC_ODR_200HZ,
    BMI088_ACC_ODR_400HZ,
    BMI088_ACC_ODR_800HZ,
    BMI088_ACC_ODR_1600HZ,
} bmi088_acc_odr_t;

typedef enum {
    BMI088_ACC_SLAVE_ADDR_SD01_GND = 0b0011000,
    BMI088_ACC_SLAVE_ADDR_SD01_VDDIO = 0b0011001,
} bmi088_acc_slave_addr_t;

typedef enum {
    BMI088_GYRO_REG_ADDR_FIFO_DATA = 0x3F,
    BMI088_GYRO_REG_ADDR_FIFO_CONFIG_1 = 0x3E,
    BMI088_GYRO_REG_ADDR_FIFO_CONFIG_0 = 0x3D,
    BMI088_GYRO_REG_ADDR_GYRO_SELF_TEST = 0x3C,
    BMI088_GYRO_REG_ADDR_FIFO_EXT_INT_S = 0x34,
    BMI088_GYRO_REG_ADDR_FIFO_WM_EN = 0x1E,
    BMI088_GYRO_REG_ADDR_INT3_INT4_IO_MAP = 0x18,
    BMI088_GYRO_REG_ADDR_INT3_INT4_IO_CONF = 0x16,
    BMI088_GYRO_REG_ADDR_GYRO_INT_CTRL = 0x15,
    BMI088_GYRO_REG_ADDR_GYRO_SOFTRESET = 0x14,
    BMI088_GYRO_REG_ADDR_GYRO_LPM1 = 0x11,
    BMI088_GYRO_REG_ADDR_GYRO_BANDWIDTH = 0x10,
    BMI088_GYRO_REG_ADDR_GYRO_RANGE = 0x0F,
    BMI088_GYRO_REG_ADDR_FIFO_STATUS = 0x0E,
    BMI088_GYRO_REG_ADDR_GYRO_INT_STAT = 0x0A,
    BMI088_GYRO_REG_ADDR_RATE_Z_MSB = 0x07,
    BMI088_GYRO_REG_ADDR_RATE_Z_LSB = 0x06,
    BMI088_GYRO_REG_ADDR_RATE_Y_MSB = 0x05,
    BMI088_GYRO_REG_ADDR_RATE_Y_LSB = 0x04,
    BMI088_GYRO_REG_ADDR_RATE_X_MSB = 0x03,
    BMI088_GYRO_REG_ADDR_RATE_X_LSB = 0x02,
    BMI088_GYRO_REG_ADDR_GYRO_CHIP_ID = 0x00,
} bmi088_gyro_reg_addr_t;

typedef enum {
    BMI088_GYRO_POWER_MODE_NORMAL,
    BMI088_GYRO_POWER_MODE_SUSPEND,
    BMI088_GYRO_POWER_MODE_DEEP_SUSPEND,
} bmi088_gyro_power_mode_t;

typedef enum {
    BMI088_GYRO_ODR,
} bmi088_gyro_odr_t;

typedef enum {
    BMI088_GYRO_SLAVE_ADDR_SDO2_GND = 0b1101000,
    BMI088_GYRO_SLAVE_ADDR_SD02_VDDIO = 0b1101001,
} bmi088_gyro_slave_addr_t;

typedef struct {
    float32_t acc_scale;
    float32_t gyro_scale;
} bmi088_config_t;

typedef struct {
    void* acc_bus_user;
    bmi088_err_t (*acc_bus_initialize)(void*);
    bmi088_err_t (*acc_bus_deinitialize)(void*);
    bmi088_err_t (*acc_bus_write_data)(void*, uint8_t, uint8_t const*, size_t);
    bmi088_err_t (*acc_bus_read_data)(void*, uint8_t, uint8_t*, size_t);

    void* gyro_bus_user;
    bmi088_err_t (*gyro_bus_initialize)(void*);
    bmi088_err_t (*gyro_bus_deinitialize)(void*);
    bmi088_err_t (*gyro_bus_write_data)(void*, uint8_t, uint8_t const*, size_t);
    bmi088_err_t (*gyro_bus_read_data)(void*, uint8_t, uint8_t*, size_t);
} bmi088_interface_t;

#ifdef __cplusplus
}
#endif

#endif // BMI088_BMI088_CONFIG_H