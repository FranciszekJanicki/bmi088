#include "bmi088.h"
#include "bmi088_config.h"
#include "bmi088_registers.h"
#include <assert.h>
#include <string.h>

static bmi088_err_t bmi088_acc_bus_init(bmi088_t const* bmi088)
{
    return bmi088->interface.acc_bus_init
               ? bmi088->interface.acc_bus_init(bmi088->interface.acc_bus_user)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_acc_bus_deinit(bmi088_t const* bmi088)
{
    return bmi088->interface.acc_bus_deinit
               ? bmi088->interface.acc_bus_deinit(bmi088->interface.acc_bus_user)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_acc_bus_write(bmi088_t const* bmi088,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size)
{
    return bmi088->interface.acc_bus_write
               ? bmi088->interface.acc_bus_write(bmi088->interface.acc_bus_user,
                                                 address,
                                                 data,
                                                 data_size)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_acc_bus_read(bmi088_t const* bmi088,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size)
{
    return bmi088->interface.acc_bus_read
               ? bmi088->interface.acc_bus_read(bmi088->interface.acc_bus_user,
                                                address,
                                                data,
                                                data_size)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_gyro_bus_init(bmi088_t const* bmi088)
{
    return bmi088->interface.gyro_bus_init
               ? bmi088->interface.gyro_bus_init(bmi088->interface.gyro_bus_user)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_gyro_bus_deinit(bmi088_t const* bmi088)
{
    return bmi088->interface.gyro_bus_deinit
               ? bmi088->interface.gyro_bus_deinit(bmi088->interface.gyro_bus_user)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_gyro_bus_write(bmi088_t const* bmi088,
                                          uint8_t address,
                                          uint8_t const* data,
                                          size_t data_size)
{
    return bmi088->interface.gyro_bus_write
               ? bmi088->interface.gyro_bus_write(bmi088->interface.gyro_bus_user,
                                                  address,
                                                  data,
                                                  data_size)
               : BMI088_ERR_NULL;
}

static bmi088_err_t bmi088_gyro_bus_read(bmi088_t const* bmi088,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
{
    return bmi088->interface.gyro_bus_read
               ? bmi088->interface.gyro_bus_read(bmi088->interface.gyro_bus_user,
                                                 address,
                                                 data,
                                                 data_size)
               : BMI088_ERR_NULL;
}

bmi088_err_t bmi088_initialize(bmi088_t* bmi088,
                               bmi088_config_t const* config,
                               bmi088_interface_t const* interface)
{
    assert(bmi088 && config && interface);

    memset(bmi088, 0, sizeof(*bmi088));
    memcpy(&bmi088->config, config, sizeof(*config));
    memcpy(&bmi088->interface, interface, sizeof(*interface));

    bmi088_err_t err = bmi088_acc_bus_init(bmi088);
    err |= bmi088_gyro_bus_init(bmi088);

    return err;
}

bmi088_err_t bmi088_deinitialize(bmi088_t* bmi088)
{
    assert(bmi088);

    bmi088_err_t err = bmi088_acc_bus_deinit(bmi088);
    err |= bmi088_gyro_bus_deinit(bmi088);

    memset(bmi088, 0, sizeof(*bmi088));

    return err;
}

bmi088_err_t bmi088_get_acc_data_raw(bmi088_t const* bmi088, vec3_int16_t* raw)
{
    assert(bmi088 && raw);

    bmi088_acc_reg_t reg = {};

    bmi088_err_t err = bmi088_get_acc_reg(bmi088, &reg);

    raw->x = reg.acc_x;
    raw->y = reg.acc_y;
    raw->z = reg.acc_z;

    return err;
}

bmi088_err_t bmi088_get_acc_data_scaled(bmi088_t const* bmi088, vec3_float32_t* scaled)
{
    assert(bmi088 && scaled);

    vec3_int16_t raw = {};

    bmi088_err_t err = bmi088_get_acc_data_raw(bmi088, &raw);

    scaled->x = (float32_t)(raw.x) * bmi088->config.acc_scale;
    scaled->y = (float32_t)(raw.y) * bmi088->config.acc_scale;
    scaled->z = (float32_t)(raw.z) * bmi088->config.acc_scale;

    return err;
}

bmi088_err_t bmi088_get_gyro_data_raw(bmi088_t const* bmi088, vec3_int16_t* raw)
{
    assert(bmi088 && raw);

    bmi088_gyro_rate_reg_t reg = {};

    bmi088_err_t err = bmi088_get_gyro_rate_reg(bmi088, &reg);

    raw->x = reg.rate_x;
    raw->y = reg.rate_y;
    raw->z = reg.rate_z;

    return err;
}

bmi088_err_t bmi088_get_gyro_data_scaled(bmi088_t const* bmi088, vec3_float32_t* scaled)
{
    assert(bmi088 && scaled);

    vec3_int16_t raw = {};

    bmi088_err_t err = bmi088_get_gyro_data_raw(bmi088, &raw);

    scaled->x = (float32_t)(raw.x) * bmi088->config.gyro_scale;
    scaled->y = (float32_t)(raw.y) * bmi088->config.gyro_scale;
    scaled->z = (float32_t)(raw.z) * bmi088->config.gyro_scale;

    return err;
}

bmi088_err_t bmi088_get_temp_data_raw(bmi088_t const* bmi088, int16_t* raw)
{
    assert(bmi088 && raw);

    bmi088_acc_temp_reg_t reg = {};

    bmi088_err_t err = bmi088_get_acc_temp_reg(bmi088, &reg);

    *raw = reg.temperature;

    // sign extend from 12 to 16 bits
    if (*raw & (1 << 10)) {
        *raw |= 0xF000;
    }

    return err;
}

bmi088_err_t bmi088_get_temp_data_scaled(bmi088_t const* bmi088, float32_t* scaled)
{
    assert(bmi088 && scaled);

    int16_t raw;

    bmi088_err_t err = bmi088_get_temp_data_raw(bmi088, &raw);

    *scaled = (float32_t)(raw)*BMI088_TEMP_SCALE;

    return err;
}

bmi088_err_t bmi088_set_acc_softreset_reg(bmi088_t const* bmi088,
                                          bmi088_acc_softreset_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->softreset & 0xFFU;

    return bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_ACC_SOFTRESET, &data, sizeof(data));
}

bmi088_err_t bmi088_set_acc_pwr_ctrl_reg(bmi088_t const* bmi088,
                                         bmi088_acc_pwr_ctrl_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->acc_enable & 0xFFU;

    return bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_ACC_PWR_CTRL, &data, sizeof(data));
}

bmi088_err_t bmi088_get_acc_pwr_ctrl_reg(bmi088_t const* bmi088, bmi088_acc_pwr_ctrl_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_PWR_CTRL, &data, sizeof(data));

    reg->acc_enable = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_acc_pwr_conf_reg(bmi088_t const* bmi088,
                                         bmi088_acc_pwr_conf_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->pwr_save_mode & 0xFFU;

    return bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_ACC_PWR_CONF, &data, sizeof(data));
}

bmi088_err_t bmi088_get_acc_pwr_conf_reg(bmi088_t const* bmi088, bmi088_acc_pwr_conf_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_PWR_CONF, &data, sizeof(data));

    reg->pwr_save_mode = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_acc_self_test_reg(bmi088_t const* bmi088,
                                          bmi088_acc_self_test_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->acc_self_test & 0xFFU;

    return bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_ACC_SELF_TEST, &data, sizeof(data));
}

bmi088_err_t bmi088_get_acc_self_test_reg(bmi088_t const* bmi088, bmi088_acc_self_test_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_SELF_TEST, &data, sizeof(data));

    reg->acc_self_test = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_acc_int_map_data_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int_map_data_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_INT_MAP_DATA, &data, sizeof(data));

    data &=
        ~((0x01U << 6U) | (0x01U << 5U) | (0x01U << 4U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

    data |= (reg->int2_drdy & 0x01U) << 6U;
    data |= (reg->int2_fwm & 0x01U) << 5U;
    data |= (reg->int2_ffull & 0x01U) << 4U;
    data |= (reg->int1_drdy & 0x01U) << 2U;
    data |= (reg->int1_fwm & 0x01U) << 1U;
    data |= reg->int1_ffull & 0x01U;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_INT_MAP_DATA, &data, sizeof(data));
}

bmi088_err_t bmi088_get_acc_int_map_data_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int_map_data_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_SELF_TEST, &data, sizeof(data));

    reg->int2_drdy = (data >> 6U) & 0x01U;
    reg->int2_fwm = (data >> 5U) & 0x01U;
    reg->int2_ffull = (data >> 4U) & 0x01U;
    reg->int1_drdy = (data >> 2U) & 0x01U;
    reg->int1_fwm = (data >> 1U) & 0x01U;
    reg->int1_ffull = data & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_acc_int2_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int2_io_ctrl_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_INT2_IO_CTRL, &data, sizeof(data));

    data &= ~((0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U));

    data |= (reg->int2_in & 0x01U) << 4U;
    data |= (reg->int2_out & 0x01U) << 3U;
    data |= (reg->int2_od & 0x01U) << 2U;
    data |= (reg->int2_lvl & 0x01U) << 1U;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_INT2_IO_CTRL, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_acc_int2_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int2_io_ctrl_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_INT2_IO_CTRL, &data, sizeof(data));

    reg->int2_in = (data >> 4U) & 0x01U;
    reg->int2_out = (data >> 3U) & 0x01U;
    reg->int2_od = (data >> 2U) & 0x01U;
    reg->int2_lvl = (data >> 1U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_acc_int1_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int1_io_ctrl_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_INT1_IO_CTRL, &data, sizeof(data));

    data &= ~((0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U));

    data |= (reg->int1_in & 0x01U) << 4U;
    data |= (reg->int1_out & 0x01U) << 3U;
    data |= (reg->int1_od & 0x01U) << 2U;
    data |= (reg->int1_lvl & 0x01U) << 1U;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_INT1_IO_CTRL, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_acc_int1_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int1_io_ctrl_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_INT1_IO_CTRL, &data, sizeof(data));

    reg->int1_in = (data >> 4U) & 0x01U;
    reg->int1_out = (data >> 3U) & 0x01U;
    reg->int1_od = (data >> 2U) & 0x01U;
    reg->int1_lvl = (data >> 1U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_acc_fifo_config_reg(bmi088_t const* bmi088,
                                            bmi088_acc_fifo_config_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_CONFIG_0, data, sizeof(data));

    data[0] &= ~(1U);
    data[1] &= ~((0x01U << 6U) | (0x01U << 3U) | (0x01U << 2U));

    data[0] |= (reg->mode & 0x01U);
    data[1] |= (reg->acc_en & 0x01U) << 6U;
    data[1] |= (reg->int1_en & 0x01U) << 3U;
    data[1] |= (reg->int2_en & 0x01U) << 2U;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_FIFO_CONFIG_0, data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_acc_fifo_config_reg(bmi088_t const* bmi088,
                                            bmi088_acc_fifo_config_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_CONFIG_0, data, sizeof(data));

    reg->acc_en = (data[0] >> 6U) & 0x01U;
    reg->int1_en = (data[0] >> 3U) & 0x01U;
    reg->int2_en = (data[0] >> 2U) & 0x01U;
    reg->mode = data[1] & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_acc_fifo_wtm_reg(bmi088_t const* bmi088,
                                         bmi088_acc_fifo_wtm_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_WTM_0, data, sizeof(data));

    data[0] &= ~0xFFU;
    data[1] &= ~0x1FU;

    data[0] |= reg->fifo_water_mark & 0xFFU;
    data[1] |= (reg->fifo_water_mark >> 8U) & 0x1FU;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_FIFO_WTM_0, data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_acc_fifo_wtm_reg(bmi088_t const* bmi088, bmi088_acc_fifo_wtm_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_WTM_0, data, sizeof(data));

    reg->fifo_water_mark = (uint16_t)((data[0] & 0xFFU) | ((data[1] & 0x1FU) << 8U));

    return err;
}

bmi088_err_t bmi088_set_acc_fifo_downs_reg(bmi088_t const* bmi088,
                                           bmi088_acc_fifo_downs_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_DOWNS, &data, sizeof(data));

    data &= ~0x60U;

    data |= (reg->fifo_downs & 0x03U) << 5U;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_FIFO_WTM_0, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_acc_fifo_downs_reg(bmi088_t const* bmi088, bmi088_acc_fifo_downs_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_WTM_0, &data, sizeof(data));

    reg->fifo_downs = (data >> 5U) & 0x03U;

    return err;
}

bmi088_err_t bmi088_set_acc_range_reg(bmi088_t const* bmi088, bmi088_acc_range_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_RANGE, &data, sizeof(data));

    data &= ~0x03;

    data |= reg->acc_range & 0x03U;

    err |= bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_ACC_RANGE, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_acc_range_reg(bmi088_t const* bmi088, bmi088_acc_range_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_RANGE, &data, sizeof(data));

    reg->acc_range = data & 0x03U;

    return err;
}

bmi088_err_t bmi088_set_acc_conf_reg(bmi088_t const* bmi088, bmi088_acc_conf_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data |= (reg->acc_bwp & 0x0FU) << 4U;
    data |= reg->acc_odr & 0x0FU;

    return bmi088_acc_bus_write(bmi088, BMI088_ACC_REG_ADDR_ACC_CONF, &data, sizeof(data));
}

bmi088_err_t bmi088_get_acc_conf_reg(bmi088_t const* bmi088, bmi088_acc_conf_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_CONF, &data, sizeof(data));

    reg->acc_bwp = (data >> 4U) & 0x0FU;
    reg->acc_odr = data & 0x0FU;

    return err;
}

bmi088_err_t bmi088_get_acc_fifo_data_reg(bmi088_t const* bmi088, bmi088_acc_fifo_data_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_DATA, &data, sizeof(data));

    reg->fifo_data = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_get_acc_fifo_length_reg(bmi088_t const* bmi088,
                                            bmi088_acc_fifo_length_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_FIFO_DATA, data, sizeof(data));

    reg->fifo_byte_counter = (uint16_t)((data[0] & 0xFFU) | ((data[1] & 0x3FU) << 8U));

    return err;
}

bmi088_err_t bmi088_get_acc_temp_reg(bmi088_t const* bmi088, bmi088_acc_temp_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_TEMP_MSB, data, sizeof(data));

    reg->temperature = (uint16_t)(((data[0] & 0xFFU) << 3U) | ((data[1] >> 5U) & 0x07U));

    return err;
}

bmi088_err_t bmi088_get_acc_int_stat_reg(bmi088_t const* bmi088, bmi088_acc_int_stat_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_INT_STAT, &data, sizeof(data));

    reg->acc_drdy = (data >> 7U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_get_acc_sensortime_reg(bmi088_t const* bmi088, bmi088_acc_sensortime_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[3] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_SENSORTIME_0,data, sizeof(data));

    reg->sensortime =
        (uint32_t)(data[0] & 0xFFU) | ((data[1] & 0xFFU) << 8U) | ((data[2] & 0xFFU) << 16U);

    return err;
}

bmi088_err_t bmi088_get_acc_z_reg(bmi088_t const* bmi088, bmi088_acc_z_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_Z_LSB,data, sizeof(data));

    reg->acc_z = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_acc_y_reg(bmi088_t const* bmi088, bmi088_acc_y_reg_t* reg)
{
    assert(bmi088 && reg);

    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_Y_LSB, data, sizeof(data));

    reg->acc_y = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_acc_x_reg(bmi088_t const* bmi088, bmi088_acc_x_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_X_LSB, data, sizeof(data));

    reg->acc_x = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_acc_reg(bmi088_t const* bmi088, bmi088_acc_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[6] = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_X_LSB, data, sizeof(data));

    reg->acc_x = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));
    reg->acc_y = (int16_t)((data[2] & 0xFF) | ((data[3] & 0xFF) << 8));
    reg->acc_z = (int16_t)((data[4] & 0xFF) | ((data[5] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_acc_status_reg(bmi088_t const* bmi088, bmi088_acc_status_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_STATUS, &data, sizeof(data));

    reg->drdy_acc = (data >> 7U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_get_acc_err_reg(bmi088_t const* bmi088, bmi088_acc_err_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_ERR_REG, &data, sizeof(data));

    reg->error_code = (data >> 2U) & 0x08U;
    reg->fatal_err = data & 0x01U;

    return err;
}

bmi088_err_t bmi088_get_acc_chip_id_reg(bmi088_t const* bmi088, bmi088_acc_chip_id_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_acc_bus_read(bmi088, BMI088_ACC_REG_ADDR_ACC_CHIP_ID, &data, sizeof(data));

    reg->acc_chip_id = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_get_gyro_fifo_data_reg(bmi088_t const* bmi088, bmi088_gyro_fifo_data_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_DATA, &data, sizeof(data));

    reg->fifo_data_output = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_gyro_fifo_config_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_config_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_CONFIG_0, data, sizeof(data));

    data[0] &= ~0xFEU;
    data[1] &= ~0x01U;

    data[0] |= (reg->fifo_wm_lvl_trig_retain & 0x7FU) << 1U;
    data[1] |= reg->fifo_mode & 0x01U;

    err |= bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_FIFO_CONFIG_0, data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_gyro_fifo_config_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_config_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_CONFIG_0, data, sizeof(data));

    data[0] &= ~0xFEU;
    data[1] &= ~0x01U;

    data[0] |= (reg->fifo_wm_lvl_trig_retain & 0x7FU) << 1U;
    data[1] |= reg->fifo_mode & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_gyro_self_test_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_self_test_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_SELF_TEST, &data, sizeof(data));

    data &= ~((0x01U << 4U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

    data |= (reg->rate_ok & 0x01U) << 4U;
    data |= (reg->bist_fail & 0x01U) << 2U;
    data |= (reg->bist_rdy & 0x01U) << 1U;
    data |= (reg->trig_bist & 0x01U);

    err |= bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_GYRO_SELF_TEST, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_gyro_self_test_reg(bmi088_t const* bmi088, bmi088_gyro_self_test_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_SELF_TEST, &data, sizeof(data));

    reg->rate_ok = (data >> 4U) & 0x01U;
    reg->bist_fail = (data >> 2U) & 0x01U;
    reg->bist_rdy = (data >> 1U) & 0x01U;
    reg->trig_bist = data & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_gyro_fifo_ext_int_s_reg(bmi088_t const* bmi088,
                                                bmi088_gyro_fifo_ext_int_s_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_EXT_INT_S, &data, sizeof(data));

    data &= ~((0x01U << 5U) | (0x01U << 4U));

    data |= (reg->ext_fifo_s_en & 0x01U) << 5U;
    data |= (reg->ext_fifo_s_sel & 0x01U) << 4U;

    err |= bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_FIFO_EXT_INT_S, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_gyro_fifo_ext_int_s_reg(bmi088_t const* bmi088,
                                                bmi088_gyro_fifo_ext_int_s_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_EXT_INT_S, &data, sizeof(data));

    reg->ext_fifo_s_en = (data >> 5U) & 0x01U;
    reg->ext_fifo_s_sel = (data >> 4U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_gyro_fifo_wtm_en_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_wm_en_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->fifo_wm_en & 0xFFU;

    return bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_FIFO_WM_EN, &data, sizeof(data));
}

bmi088_err_t bmi088_get_gyro_fifo_wtm_en_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_wm_en_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_WM_EN, &data, sizeof(data));

    reg->fifo_wm_en = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_gyro_int3_int4_io_map_reg(bmi088_t const* bmi088,
                                                  bmi088_gyro_int3_int4_io_map_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_INT3_INT4_IO_MAP, &data, sizeof(data));

    data &= ~((0x01U << 7U) | (0x01U << 5U) | (0x01U << 2U) | 0x01U);

    data |= (reg->int4_data & 0x01U) << 7U;
    data |= (reg->int4_fifo & 0x01U) << 5U;
    data |= (reg->int3_fifo & 0x01U) << 2U;
    data |= (reg->int3_data & 0x01U);

    err |=
        bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_INT3_INT4_IO_MAP, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_gyro_int3_int4_io_map_reg(bmi088_t const* bmi088,
                                                  bmi088_gyro_int3_int4_io_map_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_INT3_INT4_IO_MAP, &data, sizeof(data));

    reg->int4_data = (data >> 7U) & 0x01U;
    reg->int4_fifo = (data >> 5U) & 0x01U;
    reg->int3_fifo = (data >> 2U) & 0x01U;
    reg->int3_data = data & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_gyro_int3_int4_io_conf_reg(bmi088_t const* bmi088,
                                                   bmi088_gyro_int3_int4_io_conf_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_INT3_INT4_IO_CONF, &data, sizeof(data));

    data &= ~((0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

    data |= (reg->int4_od & 0x01U) << 3U;
    data |= (reg->int4_lvl & 0x01U) << 2U;
    data |= (reg->int3_od & 0x01U) << 1U;
    data |= reg->int3_lvl & 0x01U;

    err |=
        bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_INT3_INT4_IO_CONF, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_gyro_int3_int4_io_conf_reg(bmi088_t const* bmi088,
                                                   bmi088_gyro_int3_int4_io_conf_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_INT3_INT4_IO_CONF, &data, sizeof(data));

    reg->int4_od = (data >> 3U) & 0x01U;
    reg->int4_lvl = (data >> 2U) & 0x01U;
    reg->int3_od = (data >> 1U) & 0x01U;
    reg->int3_lvl = data & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_gyro_int_ctrl_reg(bmi088_t const* bmi088,
                                          bmi088_gyro_int_ctrl_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_INT_CTRL, &data, sizeof(data));

    data &= ~((0x01U << 7U) | (0x01U << 6U));

    data |= (reg->data_en & 0x01U) << 7U;
    data |= (reg->fifo_en & 0x01U) << 6U;

    err |= bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_GYRO_INT_CTRL, &data, sizeof(data));

    return err;
}

bmi088_err_t bmi088_get_gyro_int_ctrl_reg(bmi088_t const* bmi088, bmi088_gyro_int_ctrl_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_INT_CTRL, &data, sizeof(data));

    reg->data_en = (data >> 7U) & 0x01U;
    reg->fifo_en = (data >> 6U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_set_gyro_softreset_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_softreset_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->softreset & 0xFFU;

    return bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_GYRO_SOFTRESET, &data, sizeof(data));
}

bmi088_err_t bmi088_set_gyro_lpm1_reg(bmi088_t const* bmi088, bmi088_gyro_lpm1_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->gyro_pm & 0xFFU;

    return bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_GYRO_LPM1, &data, sizeof(data));
}

bmi088_err_t bmi088_get_gyro_lpm1_reg(bmi088_t const* bmi088, bmi088_gyro_lpm1_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_LPM1, &data, sizeof(data));

    reg->gyro_pm = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_gyro_bandwidth_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_bandwidth_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->gyro_bw & 0xFFU;

    return bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_GYRO_BANDWIDTH, &data, sizeof(data));
}

bmi088_err_t bmi088_get_gyro_bandwidth_reg(bmi088_t const* bmi088, bmi088_gyro_bandwidth_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_BANDWIDTH, &data, sizeof(data));

    reg->gyro_bw = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_set_gyro_range_reg(bmi088_t const* bmi088, bmi088_gyro_range_reg_t const* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    data = reg->gyro_range & 0xFFU;

    return bmi088_gyro_bus_write(bmi088, BMI088_GYRO_REG_ADDR_GYRO_RANGE, &data, sizeof(data));
}

bmi088_err_t bmi088_get_gyro_range_reg(bmi088_t const* bmi088, bmi088_gyro_range_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_RANGE, &data, sizeof(data));

    reg->gyro_range = data & 0xFFU;

    return err;
}

bmi088_err_t bmi088_get_gyro_fifo_status_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_status_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_FIFO_STATUS, &data, sizeof(data));

    reg->fifo_overrun = (data >> 7U) & 0x01U;
    reg->fifo_frame_counter = data & 0x7FU;

    return err;
}

bmi088_err_t bmi088_get_get_gyro_int_stat_reg(bmi088_t const* bmi088,
                                              bmi088_gyro_int_stat_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_INT_STAT, &data, sizeof(data));

    reg->gyro_drdy = (data >> 7U) & 0x01U;
    reg->fifo_int = (data >> 4U) & 0x01U;

    return err;
}

bmi088_err_t bmi088_get_gyro_rate_z_reg(bmi088_t const* bmi088, bmi088_gyro_rate_z_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_RATE_Z_LSB, data, sizeof(data));

    reg->rate_z = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_gyro_rate_y_reg(bmi088_t const* bmi088, bmi088_gyro_rate_y_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_RATE_Y_LSB, data, sizeof(data));

    reg->rate_y = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_gyro_rate_x_reg(bmi088_t const* bmi088, bmi088_gyro_rate_x_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[2] = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_RATE_X_LSB, data, sizeof(data));

    reg->rate_x = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_gyro_rate_reg(bmi088_t const* bmi088, bmi088_gyro_rate_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data[6] = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_RATE_X_LSB, data, sizeof(data));

    reg->rate_x = (int16_t)((data[0] & 0xFF) | ((data[1] & 0xFF) << 8));
    reg->rate_y = (int16_t)((data[2] & 0xFF) | ((data[3] & 0xFF) << 8));
    reg->rate_z = (int16_t)((data[4] & 0xFF) | ((data[5] & 0xFF) << 8));

    return err;
}

bmi088_err_t bmi088_get_gyro_chip_id_reg(bmi088_t const* bmi088, bmi088_gyro_chip_id_reg_t* reg)
{
    assert(bmi088 && reg);

    uint8_t data = {};

    bmi088_err_t err =
        bmi088_gyro_bus_read(bmi088, BMI088_GYRO_REG_ADDR_GYRO_CHIP_ID, &data, sizeof(data));

    reg->gyro_chip_id = data & 0xFFU;

    return err;
}
