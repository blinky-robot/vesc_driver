#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

# Enumerations
pwm_mode_enum = gen.enum([
	gen.const("Nonsynchronous",	int_t,	0,	"This mode is not recommended"),
	gen.const("Synchronous",	int_t,	1,	"Recommended and most tested mode"),
	gen.const("Bipolar",		int_t,	2,	"Some glitches occasionally, can kill MOSFETs"),
	], "PWM Mode")
comm_mode_enum = gen.enum([
	gen.const("Integrate",		int_t,	0,	"Integrate"),
	gen.const("Delay",		int_t,	1,	"Delay"),
	], "Commutation Mode")
motor_type_enum = gen.enum([
	gen.const("BLDC",		int_t,	0,	"Brushless Direct Current"),
	gen.const("DC",			int_t,	1,	"Brushed Direct Current"),
	], "Motor Type")
sensor_mode_enum = gen.enum([
	gen.const("Sensorless",		int_t,	0,	"Sensorless Mode"),
	gen.const("Sensored",		int_t,	1,	"Sensored Mode"),
	gen.const("Hybrid",		int_t,	2,	"Hybrid Sensored/Sensorless Mode"),
	], "Sensor Mode")

# VESC Config
gen.add("pwm_mode",		int_t,		0,	"PWM Mode",			0,	0,	2,	edit_method=pwm_mode_enum);
gen.add("comm_mode",		int_t,		0,	"Commutation Mode",		0,	0,	1,	edit_method=comm_mode_enum);
gen.add("motor_type",		int_t,		0,	"Motor Type",			0,	0,	1,	edit_method=motor_type_enum);
gen.add("sensor_mode",		int_t,		0,	"Sensor Mode",			0,	0,	2,	edit_method=sensor_mode_enum);
gen.add("l_current_max",	double_t,	0,	"Maximum motor current",	60.0,	0.0,	200.0);
gen.add("l_current_min",	double_t,	0,	"Minimum motor current (regen)",	-60.0,	-200.0,	0.0);
gen.add("l_in_current_max",	double_t,	0,	"Maximum battery current",	60.0,	0.0,	200.0);
gen.add("l_in_current_min",	double_t,	0,	"Minimum battery current (regen)",	-20.0,	-200.0,	0.0);
gen.add("l_abs_current_max",	double_t,	0,	"Absolute maximum current",	130.0,	0.0,	200.0);
gen.add("l_min_erpm",		double_t,	0,	"Minimum ERPM",			-100000.0,	-200000.0,	0.0);
gen.add("l_max_erpm",		double_t,	0,	"Maximum ERPM",			100000.0,	0.0,	200000.0);
gen.add("l_max_erpm_fbrake",	double_t,	0,	"Maximum ERPM at full brake",	300.00,	0.0,	100000.0);
gen.add("l_max_erpm_fbrake_cc",	double_t,	0,	"Maximum ERPM at full brake in current control mode",	1500.00,	0.0,	100000.0);
gen.add("l_min_vin",		double_t,	0,	"Minimum input voltage",	8.0,	6.0,	60.0);
gen.add("l_max_vin",		double_t,	0,	"Maximum input voltage",	8.0,	6.0,	60.0);
gen.add("l_slow_abs_current",	bool_t,		0,	"Slow absolute maximum current",	True);
gen.add("l_rpm_lim_neg_torque",	bool_t,		0,	"Limit ERPM with negative torque",	True);
gen.add("l_temp_fet_start",	double_t,	0,	"MOSFET Temperature Limit Start",	80.0,	-100.0,	300.0);
gen.add("l_temp_fet_end",	double_t,	0,	"MOSFET Temperature Limit End",		100.0,	-100.0,	300.0);
gen.add("l_temp_motor_start",	double_t,	0,	"Motor Temperature Limit Start",	80.0,	-100.0,	300.0);
gen.add("l_temp_motor_end",	double_t,	0,	"Motor Temperature Limit End",		100.0,	-100.0,	300.0);
gen.add("l_min_duty",		double_t,	0,	"Minimum duty cycle",			0.005,	0.005,	0.5);
gen.add("l_max_duty",		double_t,	0,	"Maximum duty cycle",			0.95,	0.01,	0.95);
gen.add("sl_min_erpm",		double_t,	0,	"Sensorless minimum ERPM",	150.0,	0.0,	100000.0);
gen.add("sl_min_erpm_cycle_int_limit",	double_t,	0,	"Sensorless minimum ERPM for integrator limit",	1100.0,	0.0,	10000.0);
gen.add("sl_max_fullbreak_current_dir_change",	double_t,	0,	"Sensorless maximum full brake current during direction change",	10.0,	0.0,	300.0);
gen.add("sl_cycle_int_limit",	double_t,	0,	"Sensorless integrator limit",	62.0,	0.0,	10000.0);
gen.add("sl_phase_advance_at_br",	double_t,	0,	"Sensorless phase advance at BR ERPM",	0.8,	0.0,	1.0);
gen.add("sl_cycle_int_rpm_br",	double_t,	0,	"Sensorless BR ERPM",	80000.0,	0.0,	200000.0);
gen.add("sl_bemf_coupling_k",	double_t,	0,	"Sensorless BEMF Coupling",	600.0,	0.0,	5000.0);
gen.add("hall_table_0",		int_t,		0,	"Hall sensor table value 0",	-1,	-1,	6);
gen.add("hall_table_1",		int_t,		0,	"Hall sensor table value 1",	1,	-1,	6);
gen.add("hall_table_2",		int_t,		0,	"Hall sensor table value 2",	3,	-1,	6);
gen.add("hall_table_3",		int_t,		0,	"Hall sensor table value 3",	2,	-1,	6);
gen.add("hall_table_4",		int_t,		0,	"Hall sensor table value 4",	5,	-1,	6);
gen.add("hall_table_5",		int_t,		0,	"Hall sensor table value 5",	6,	-1,	6);
gen.add("hall_table_6",		int_t,		0,	"Hall sensor table value 6",	4,	-1,	6);
gen.add("hall_table_7",		int_t,		0,	"Hall sensor table value 7",	-1,	-1,	6);
gen.add("hall_sl_erpm",		double_t,	0,	"Sensorless ERPM (hybrid mode)",	2000.0,	0.0,	200000.0);
gen.add("s_pid_kp",		double_t,	0,	"Speed control KP",		0.0001,	0.0,	99.99);
gen.add("s_pid_ki",		double_t,	0,	"Speed control KI",		0.002,	0.0,	99.99);
gen.add("s_pid_kd",		double_t,	0,	"Speed control KD",		0.0,	0.0,	99.99);
gen.add("s_pid_min_rpm",	double_t,	0,	"Speed control minimum ERPM",	900.0,	0.0,	200000.0);
gen.add("p_pid_kp",		double_t,	0,	"Position control KP",		0.0001,	0.0,	99.99);
gen.add("p_pid_ki",		double_t,	0,	"Position control KI",		0.002,	0.0,	99.99);
gen.add("p_pid_kd",		double_t,	0,	"Position control KD",		0.0,	0.0,	99.99);
gen.add("cc_startup_boost_duty",	double_t,	0,	"Current control startup boost",	0.01,	0.0,	1.0);
gen.add("cc_min_current",	double_t,	0,	"Current control minimum current",	1.0,	0.0,	99.99);
gen.add("cc_gain",		double_t,	0,	"Current control gain",	0.0046,	0.0,	50.0);
gen.add("cc_ramp_step_max",	double_t,	0,	"Current control maximum ramp step (at 1000 Hz)",	0.04,	0.0,	50.0);
gen.add("m_fault_stop_time_ms",	int_t,		0,	"Fault stop timeout (in milliseconds)",	3000,	-1,	40000000);

exit(gen.generate("vesc_driver", "vesc_driver", "VESC"))
