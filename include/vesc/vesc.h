/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _vesc_h
#define _vesc_h

#include <stdint.h>

#define VESC_MAX_DESCRIPTORS 256
#define VESC_TIMEOUT_DEFAULT 1

#ifdef __cplusplus
extern "C" {
#endif

enum VESC_BAUD
{
	VESC_BAUD_115200 = 0,
	VESC_BAUD_MAX
};

enum VESC_ERROR
{
	VESC_SUCCESS = 0,
	VESC_ERROR_IO = -1,
	VESC_ERROR_INVALID_PARAM = -2,
	VESC_ERROR_ACCESS = -3,
	VESC_ERROR_NO_DEVICE = -4,
	VESC_ERROR_NOT_FOUND = -5,
	VESC_ERROR_BUSY = -6,
	VESC_ERROR_TIMEOUT = -7,
	VESC_ERROR_NO_MEM = -11,
	VESC_ERROR_OTHER = -99,
	VESC_ERROR_INVALID_RESPONSE = -100,
	VESC_ERROR_CHECKSUM_FAILURE = -101,
	VESC_ERROR_NOT_CONNECTED = -102,
};

enum VESC_STATE
{
	VESC_STATE_OFF = 0,
	VESC_STATE_DETECTING,
	VESC_STATE_RUNNING,
	VESC_STATE_FULL_BRAKE,
};

enum VESC_PWM_MODE
{
	VESC_PWM_MODE_NONSYNCHRONOUS_HISW = 0,
	VESC_PWM_MODE_SYNCHRONOUS,
	VESC_PWM_MODE_BIPOLAR,
};

enum VESC_COMM_MODE
{
	VESC_COMM_MODE_INTEGRATE = 0,
	VESC_COMM_MODE_DELAY,
};

enum VESC_MOTOR_TYPE
{
	VESC_MOTOR_TYPE_BLDC = 0,
	VESC_MOTOR_TYPE_DC,
};

enum VESC_FAULT_CODE
{
	VESC_FAULT_CODE_NONE = 0,
	VESC_FAULT_CODE_OVER_VOLTAGE,
	VESC_FAULT_CODE_UNDER_VOLTAGE,
	VESC_FAULT_CODE_DRV8302,
	VESC_FAULT_CODE_ABS_OVER_CURRENT,
	VESC_FAULT_CODE_OVER_TEMP_FET,
	VESC_FAULT_CODE_OVER_TEMP_MOTOR,
};

enum VESC_CONTROL_MODE
{
	VESC_CONTROL_MODE_DUTY = 0,
	VESC_CONTROL_MODE_SPEED,
	VESC_CONTROL_MODE_CURRENT,
	VESC_CONTROL_MODE_CURRENT_BRAKE,
	VESC_CONTROL_MODE_POS,
	VESC_CONTROL_MODE_NONE,
};

enum VESC_APP
{
	VESC_APP_NONE = 0,
	VESC_APP_PPM,
	VESC_APP_ADC,
	VESC_APP_UART,
	VESC_APP_PPM_UART,
	VESC_APP_ADC_UART,
	VESC_APP_NUNCHUK,
	VESC_APP_NRF,
	VESC_APP_CUSTOM,
};

enum VESC_PPM_CTRL_TYPE{
	VESC_PPM_CTRL_TYPE_NONE = 0,
	VESC_PPM_CTRL_TYPE_CURRENT,
	VESC_PPM_CTRL_TYPE_CURRENT_NOREV,
	VESC_PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE,
	VESC_PPM_CTRL_TYPE_DUTY,
	VESC_PPM_CTRL_TYPE_DUTY_NOREV,
	VESC_PPM_CTRL_TYPE_PID,
	VESC_PPM_CTRL_TYPE_PID_NOREV,
};

struct vesc_values
{
	int16_t temp_mos1;
	int16_t temp_mos2;
	int16_t temp_mos3;
	int16_t temp_mos4;
	int16_t temp_mos5;
	int16_t temp_mos6;
	int16_t temp_pcb;
	int32_t current_motor;
	int32_t current_in;
	int16_t duty_now;
	int32_t rpm;
	int16_t v_in;
	int32_t amp_hours;
	int32_t amp_hours_charged;
	int32_t watt_hours;
	int32_t watt_hours_charged;
	int32_t tachometer;
	int32_t tachometer_abs;
	uint8_t fault_code;
} __attribute__((packed));

struct vesc_config
{
	uint8_t pwm_mode;
	uint8_t comm_mode;
	uint8_t motor_type;
	int32_t l_current_max;
	int32_t l_current_min;
	int32_t l_in_current_max;
	int32_t l_in_current_min;
	int32_t l_abs_current_max;
	int32_t l_min_erpm;
	int32_t l_max_erpm;
	int32_t l_max_erpm_fbrake;
	int32_t l_max_erpm_fbrake_cc;
	int32_t l_min_vin;
	int32_t l_max_vin;
	uint8_t l_slow_abs_current;
	uint8_t l_rpm_lim_neg_torque;
	int32_t l_temp_fet_start;
	int32_t l_temp_fet_end;
	int32_t l_temp_motor_start;
	int32_t l_temp_motor_end;
	int32_t l_min_duty;
	int32_t l_max_duty;
	uint8_t sl_is_sensorless;
	int32_t sl_min_erpm;
	int32_t sl_min_erpm_cycle_int_limit;
	int32_t sl_max_fullbreak_current_dir_change;
	int32_t sl_cycle_int_limit;
	int32_t sl_phase_advance_at_br;
	int32_t sl_cycle_int_rpm_br;
	int32_t sl_bemf_coupling_k;
	int8_t hall_dir;
	int8_t hall_fwd_add;
	int8_t hall_rev_add;
	int32_t s_pid_kp;
	int32_t s_pid_ki;
	int32_t s_pid_kd;
	int32_t s_pid_min_rpm;
	int32_t p_pid_kp;
	int32_t p_pid_ki;
	int32_t p_pid_kd;
	int32_t cc_startup_boost_duty;
	int32_t cc_min_current;
	int32_t cc_gain;
	int32_t cc_ramp_step_max;
	int32_t m_fault_stop_time_ms;
} __attribute__((packed));

/**
 * Functions
 */
void vesc_close(const int vescd);
int vesc_open(const char *port, const enum VESC_BAUD baud, const uint8_t timeout);
int vesc_process(const int vescd);
int vesc_request_config(const int vescd);
int vesc_request_fw_version(const int vescd);
int vesc_request_values(const int vescd);
int vesc_set_cb_context(const int vescd, void *context);
int vesc_set_cb_get_config(const int vescd, int (*get_config_cb)(void *context, struct vesc_config *config));
int vesc_set_cb_get_fw_version(const int vescd, int (*get_fw_version_cb)(void *context, uint8_t major, uint8_t minor));
int vesc_set_cb_get_values(const int vescd, int (*get_values_cb)(void *context, struct vesc_values *values));
int vesc_set_rpm(const int vescd, int32_t rpm);
const char * vesc_strerror(const int error);

#ifdef __cplusplus
}
#endif

#endif /* _vesc_h */
