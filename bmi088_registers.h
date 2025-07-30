#ifndef BMI088_BMI088_REGISTERS_H
#define BMI088_BMI088_REGISTERS_H

#include <stdint.h>

typedef struct {
  uint8_t softreset : 8;
} bmi088_acc_softreset_reg_t;

typedef struct {
  uint8_t acc_enable : 8;
} bmi088_acc_pwr_ctrl_reg_t;

typedef struct {
  uint8_t pwr_save_mode : 8;
} bmi088_acc_pwr_conf_reg_t;

typedef struct {
  uint8_t acc_self_test : 8;
} bmi088_acc_self_test_reg_t;

typedef struct {
  uint8_t int2_drdy : 1;
  uint8_t int2_fwm : 1;
  uint8_t int2_ffull : 1;
  uint8_t int1_drdy : 1;
  uint8_t int1_fwm : 1;
  uint8_t int1_ffull : 1;
} bmi088_acc_int_map_data_reg_t;

typedef struct {
  uint8_t int2_in : 1;
  uint8_t int2_out : 1;
  uint8_t int2_od : 1;
  uint8_t int2_lvl : 1;
} bmi088_acc_int2_io_ctrl_reg_t;

typedef struct {
  uint8_t int1_in : 1;
  uint8_t int1_out : 1;
  uint8_t int1_od : 1;
  uint8_t int1_lvl : 1;
} bmi088_acc_int1_io_ctrl_reg_t;

typedef struct {
  uint8_t acc_en : 1;
  uint8_t int1_en : 1;
  uint8_t int2_en : 1;
  uint8_t mode : 1;
} bmi088_acc_fifo_config_reg_t;

typedef struct {
  uint16_t fifo_water_mark : 13;
} bmi088_acc_fifo_wtm_reg_t;

typedef struct {
  uint8_t fifo_downs : 2;
} bmi088_acc_fifo_downs_reg_t;

typedef struct {
  uint8_t acc_range : 2;
} bmi088_acc_range_reg_t;

typedef struct {
  uint8_t acc_bwp : 4;
  uint8_t acc_odr : 4;
} bmi088_acc_conf_reg_t;

typedef struct {
  uint8_t fifo_data : 8;
} bmi088_acc_fifo_data_reg_t;

typedef struct {
  uint16_t fifo_byte_counter : 14;
} bmi088_acc_fifo_length_reg_t;

typedef struct {
  uint16_t temperature : 11;
} bmi088_acc_temp_reg_t;

typedef struct {
  uint8_t acc_drdy : 1;
} bmi088_acc_int_stat_reg_t;

typedef struct {
  uint32_t sensortime : 24;
} bmi088_acc_sensortime_reg_t;

typedef struct {
  int16_t acc_z : 16;
} bmi088_acc_z_reg_t;

typedef struct {
  int16_t acc_y : 16;
} bmi088_acc_y_reg_t;

typedef struct {
  int16_t acc_x : 16;
} bmi088_acc_x_reg_t;

typedef struct {
  int16_t acc_x : 16;
  int16_t acc_y : 16;
  int16_t acc_z : 16;
} bmi088_acc_reg_t;

typedef struct {
  uint8_t drdy_acc : 1;
} bmi088_acc_status_reg_t;

typedef struct {
  uint8_t error_code : 3;
  uint8_t fatal_err : 1;
} bmi088_acc_err_reg_t;

typedef struct {
  uint8_t acc_chip_id : 8;
} bmi088_acc_chip_id_reg_t;

typedef struct {
  uint8_t fifo_data_output : 8;
} bmi088_gyro_fifo_data_reg_t;

typedef struct {
  uint8_t fifo_wm_lvl_trig_retain : 7;
  uint8_t fifo_mode : 1;
} bmi088_gyro_fifo_config_reg_t;

typedef struct {
  uint8_t rate_ok : 1;
  uint8_t bist_fail : 1;
  uint8_t bist_rdy : 1;
  uint8_t trig_bist : 1;
} bmi088_gyro_self_test_reg_t;

typedef struct {
  uint8_t ext_fifo_s_en : 1;
  uint8_t ext_fifo_s_sel : 1;
} bmi088_gyro_fifo_ext_int_s_reg_t;

typedef struct {
  uint8_t fifo_wm_en : 8;
} bmi088_gyro_fifo_wm_en_reg_t;

typedef struct {
  uint8_t int4_data : 1;
  uint8_t int4_fifo : 1;
  uint8_t int3_fifo : 1;
  uint8_t int3_data : 1;
} bmi088_gyro_int3_int4_io_map_reg_t;

typedef struct {
  uint8_t int4_od : 1;
  uint8_t int4_lvl : 1;
  uint8_t int3_od : 1;
  uint8_t int3_lvl : 1;
} bmi088_gyro_int3_int4_io_conf_reg_t;

typedef struct {
  uint8_t data_en : 1;
  uint8_t fifo_en : 1;
} bmi088_gyro_int_ctrl_reg_t;

typedef struct {
  uint8_t softreset : 8;
} bmi088_gyro_softreset_reg_t;

typedef struct {
  uint8_t gyro_pm : 8;
} bmi088_gyro_lpm1_reg_t;

typedef struct {
  uint8_t gyro_bw : 8;
} bmi088_gyro_bandwidth_reg_t;

typedef struct {
  uint8_t gyro_range : 8;
} bmi088_gyro_range_reg_t;

typedef struct {
  uint8_t fifo_overrun : 1;
  uint8_t fifo_frame_counter : 7;
} bmi088_gyro_fifo_status_reg_t;

typedef struct {
  uint8_t gyro_drdy : 1;
  uint8_t fifo_int : 1;
} bmi088_gyro_int_stat_reg_t;

typedef struct {
  int16_t rate_z : 16;
} bmi088_gyro_rate_z_reg_t;

typedef struct {
  int16_t rate_y : 16;
} bmi088_gyro_rate_y_reg_t;

typedef struct {
  int16_t rate_x : 16;
} bmi088_gyro_rate_x_reg_t;

typedef struct {
  int16_t rate_x : 16;
  int16_t rate_y : 16;
  int16_t rate_z : 16;
} bmi088_gyro_rate_reg_t;

typedef struct {
  uint8_t gyro_chip_id : 8;
} bmi088_gyro_chip_id_reg_t;

#endif // BMI088_BMI088_REGISTERS_H
