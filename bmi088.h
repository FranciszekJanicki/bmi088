#ifndef BMI088_BMI088_H
#define BMI088_BMI088_H

#include "bmi088_config.h"
#include "bmi088_registers.h"

typedef struct {
    bmi088_config_t config;
    bmi088_interface_t interface;
} bmi088_t;

bmi088_err_t bmi088_initialize(bmi088_t* bmi088,
                               bmi088_config_t const* config,
                               bmi088_interface_t const* interface);
bmi088_err_t bmi088_deinitialize(bmi088_t* bmi088);

bmi088_err_t bmi088_get_acc_data_raw(bmi088_t const* bmi088, vec3_int16_t* raw);
bmi088_err_t bmi088_get_acc_data_scaled(bmi088_t const* bmi088, vec3_float32_t* scaled);

bmi088_err_t bmi088_get_gyro_data_raw(bmi088_t const* bmi088, vec3_int16_t* raw);
bmi088_err_t bmi088_get_gyro_data_scaled(bmi088_t const* bmi088, vec3_float32_t* scaled);

bmi088_err_t bmi088_get_temp_data_raw(bmi088_t const* bmi088, int16_t* raw);
bmi088_err_t bmi088_get_temp_data_scaled(bmi088_t const* bmi088, float32_t* scaled);

bmi088_err_t bmi088_set_acc_softreset_reg(bmi088_t const* bmi088,
                                          bmi088_acc_softreset_reg_t const* reg);

bmi088_err_t bmi088_set_acc_pwr_ctrl_reg(bmi088_t const* bmi088,
                                         bmi088_acc_pwr_ctrl_reg_t const* reg);
bmi088_err_t bmi088_get_acc_pwr_ctrl_reg(bmi088_t const* bmi088, bmi088_acc_pwr_ctrl_reg_t* reg);

bmi088_err_t bmi088_set_acc_pwr_conf_reg(bmi088_t const* bmi088,
                                         bmi088_acc_pwr_conf_reg_t const* reg);
bmi088_err_t bmi088_get_acc_pwr_conf_reg(bmi088_t const* bmi088, bmi088_acc_pwr_conf_reg_t* reg);

bmi088_err_t bmi088_set_acc_self_test_reg(bmi088_t const* bmi088,
                                          bmi088_acc_self_test_reg_t const* reg);
bmi088_err_t bmi088_get_acc_self_test_reg(bmi088_t const* bmi088, bmi088_acc_self_test_reg_t* reg);

bmi088_err_t bmi088_set_acc_int_map_data_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int_map_data_reg_t const* reg);
bmi088_err_t bmi088_get_acc_int_map_data_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int_map_data_reg_t* reg);

bmi088_err_t bmi088_set_acc_int2_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int2_io_ctrl_reg_t const* reg);
bmi088_err_t bmi088_get_acc_int2_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int2_io_ctrl_reg_t* reg);

bmi088_err_t bmi088_set_acc_int1_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int1_io_ctrl_reg_t const* reg);
bmi088_err_t bmi088_get_acc_int1_io_ctrl_reg(bmi088_t const* bmi088,
                                             bmi088_acc_int1_io_ctrl_reg_t* reg);

bmi088_err_t bmi088_set_acc_fifo_config_reg(bmi088_t const* bmi088,
                                            bmi088_acc_fifo_config_reg_t const* reg);
bmi088_err_t bmi088_get_acc_fifo_config_reg(bmi088_t const* bmi088,
                                            bmi088_acc_fifo_config_reg_t* reg);

bmi088_err_t bmi088_set_acc_fifo_wtm_reg(bmi088_t const* bmi088,
                                         bmi088_acc_fifo_wtm_reg_t const* reg);
bmi088_err_t bmi088_get_acc_fifo_wtm_reg(bmi088_t const* bmi088, bmi088_acc_fifo_wtm_reg_t* reg);

bmi088_err_t bmi088_set_acc_fifo_downs_reg(bmi088_t const* bmi088,
                                           bmi088_acc_fifo_downs_reg_t const* reg);
bmi088_err_t bmi088_get_acc_fifo_downs_reg(bmi088_t const* bmi088,
                                           bmi088_acc_fifo_downs_reg_t* reg);

bmi088_err_t bmi088_set_acc_range_reg(bmi088_t const* bmi088, bmi088_acc_range_reg_t const* reg);
bmi088_err_t bmi088_get_acc_range_reg(bmi088_t const* bmi088, bmi088_acc_range_reg_t* reg);

bmi088_err_t bmi088_set_acc_conf_reg(bmi088_t const* bmi088, bmi088_acc_conf_reg_t const* reg);
bmi088_err_t bmi088_get_acc_conf_reg(bmi088_t const* bmi088, bmi088_acc_conf_reg_t* reg);

bmi088_err_t bmi088_get_acc_fifo_data_reg(bmi088_t const* bmi088, bmi088_acc_fifo_data_reg_t* reg);

bmi088_err_t bmi088_get_acc_fifo_length_reg(bmi088_t const* bmi088,
                                            bmi088_acc_fifo_length_reg_t* reg);

bmi088_err_t bmi088_get_acc_temp_reg(bmi088_t const* bmi088, bmi088_acc_temp_reg_t* reg);

bmi088_err_t bmi088_get_acc_int_stat_reg(bmi088_t const* bmi088, bmi088_acc_int_stat_reg_t* reg);

bmi088_err_t bmi088_get_acc_sensortime_reg(bmi088_t const* bmi088,
                                           bmi088_acc_sensortime_reg_t* reg);

bmi088_err_t bmi088_get_acc_z_reg(bmi088_t const* bmi088, bmi088_acc_z_reg_t* reg);

bmi088_err_t bmi088_get_acc_y_reg(bmi088_t const* bmi088, bmi088_acc_y_reg_t* reg);

bmi088_err_t bmi088_get_acc_x_reg(bmi088_t const* bmi088, bmi088_acc_x_reg_t* reg);

bmi088_err_t bmi088_get_acc_reg(bmi088_t const* bmi088, bmi088_acc_reg_t* reg);

bmi088_err_t bmi088_get_acc_status_reg(bmi088_t const* bmi088, bmi088_acc_status_reg_t* reg);

bmi088_err_t bmi088_get_acc_err_reg(bmi088_t const* bmi088, bmi088_acc_err_reg_t* reg);

bmi088_err_t bmi088_get_acc_chip_id_reg(bmi088_t const* bmi088, bmi088_acc_chip_id_reg_t* reg);

bmi088_err_t bmi088_get_gyro_fifo_data_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_fifo_data_reg_t* reg);

bmi088_err_t bmi088_set_gyro_fifo_config_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_config_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_fifo_config_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_config_reg_t* reg);

bmi088_err_t bmi088_set_gyro_self_test_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_self_test_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_self_test_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_self_test_reg_t* reg);

bmi088_err_t bmi088_set_gyro_fifo_ext_int_s_reg(bmi088_t const* bmi088,
                                                bmi088_gyro_fifo_ext_int_s_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_fifo_ext_int_s_reg(bmi088_t const* bmi088,
                                                bmi088_gyro_fifo_ext_int_s_reg_t* reg);

bmi088_err_t bmi088_set_gyro_fifo_wtm_en_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_wm_en_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_fifo_wtm_en_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_wm_en_reg_t* reg);

bmi088_err_t bmi088_set_gyro_int3_int4_io_map_reg(bmi088_t const* bmi088,
                                                  bmi088_gyro_int3_int4_io_map_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_int3_int4_io_map_reg(bmi088_t const* bmi088,
                                                  bmi088_gyro_int3_int4_io_map_reg_t* reg);

bmi088_err_t bmi088_set_gyro_int3_int4_io_conf_reg(bmi088_t const* bmi088,
                                                   bmi088_gyro_int3_int4_io_conf_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_int3_int4_io_conf_reg(bmi088_t const* bmi088,
                                                   bmi088_gyro_int3_int4_io_conf_reg_t* reg);

bmi088_err_t bmi088_set_gyro_int_ctrl_reg(bmi088_t const* bmi088,
                                          bmi088_gyro_int_ctrl_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_int_ctrl_reg(bmi088_t const* bmi088, bmi088_gyro_int_ctrl_reg_t* reg);

bmi088_err_t bmi088_set_gyro_softreset_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_softreset_reg_t const* reg);

bmi088_err_t bmi088_set_gyro_lpm1_reg(bmi088_t const* bmi088, bmi088_gyro_lpm1_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_lpm1_reg(bmi088_t const* bmi088, bmi088_gyro_lpm1_reg_t* reg);

bmi088_err_t bmi088_set_gyro_bandwidth_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_bandwidth_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_bandwidth_reg(bmi088_t const* bmi088,
                                           bmi088_gyro_bandwidth_reg_t* reg);

bmi088_err_t bmi088_set_gyro_range_reg(bmi088_t const* bmi088, bmi088_gyro_range_reg_t const* reg);
bmi088_err_t bmi088_get_gyro_range_reg(bmi088_t const* bmi088, bmi088_gyro_range_reg_t* reg);

bmi088_err_t bmi088_get_gyro_fifo_status_reg(bmi088_t const* bmi088,
                                             bmi088_gyro_fifo_status_reg_t* reg);

bmi088_err_t bmi088_get_get_gyro_int_stat_reg(bmi088_t const* bmi088,
                                              bmi088_gyro_int_stat_reg_t* reg);

bmi088_err_t bmi088_get_gyro_rate_z_reg(bmi088_t const* bmi088, bmi088_gyro_rate_z_reg_t* reg);

bmi088_err_t bmi088_get_gyro_rate_y_reg(bmi088_t const* bmi088, bmi088_gyro_rate_y_reg_t* reg);

bmi088_err_t bmi088_get_gyro_rate_x_reg(bmi088_t const* bmi088, bmi088_gyro_rate_x_reg_t* reg);

bmi088_err_t bmi088_get_gyro_rate_reg(bmi088_t const* bmi088, bmi088_gyro_rate_reg_t* reg);

bmi088_err_t bmi088_get_gyro_chip_id_reg(bmi088_t const* bmi088, bmi088_gyro_chip_id_reg_t* reg);

#endif // BMI088_BMI088_H