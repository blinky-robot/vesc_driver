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

#include "vesc_driver/vesc_driver.hpp"

namespace vesc_driver
{
	/**
	 * Public Data
	 */
	const int VESC::default_baud = (int)VESC_BAUD_115200;
	const std::string VESC::default_port = "/dev/ttyACM0";

	/**
	 * Callback Wrappers
	 */
	int getConfigCallbackWrapper(void *context, struct vesc_config *config)
	{
		if (context == NULL)
		{
			return VESC_ERROR_INVALID_PARAM;
		}

		return ((class VESC *)context)->getConfigCallback(config);
	}

	int getFwVersionCallbackWrapper(void *context, uint8_t major, uint8_t minor)
	{
		if (context == NULL)
		{
			return VESC_ERROR_INVALID_PARAM;
		}

		return ((class VESC *)context)->getFwVersionCallback(major, minor);
	}

	int getValuesCallbackWrapper(void *context, struct vesc_values *values)
	{
		if (context == NULL)
		{
			return VESC_ERROR_INVALID_PARAM;
		}

		return ((class VESC *)context)->getValuesCallback(values);
	}

	int setConfigCallbackWrapper(void *context)
	{
		if (context == NULL)
		{
			return VESC_ERROR_INVALID_PARAM;
		}

		return ((class VESC *)context)->setConfigCallback();
	}

	/**
	 * Public Functions
	 */
	Exception::Exception(const enum VESC_ERROR vesc_error)
		: ros::Exception(vesc_strerror(vesc_error)),
		  vesc_error(vesc_error)
	{
	}

	VESC::VESC(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
		: diag_timer(nh.createTimer(ros::Duration(1.0), &VESC::diagTimerCallback, this)),
		  dyn_re_cb(boost::bind(&VESC::dynReCallback, this, _1, _2)),
		  dyn_re_srv(NULL),
		  nh(nh),
		  nh_priv(nh_priv),
		  rad_per_tick(1.047197551),
		  read_thread(NULL),
		  this_name("vesc_driver"),
		  timeout(50),
		  vescd(-1)
	{
		nh_priv.param("baud", baud, VESC::default_baud);
		nh_priv.param("port", port, VESC::default_port);
		nh_priv.param("rad_per_tick", rad_per_tick, rad_per_tick);
		nh_priv.param("timeout", timeout, timeout);

		diag.setHardwareIDf("VESC on '%s'", port.c_str());
		diag.add("VESC Status", this, &VESC::queryDiagnostics);
	}

	VESC::~VESC()
	{
		if (read_thread != NULL)
		{
			read_thread->interrupt();
			delete read_thread;
		}

		delete dyn_re_srv;

		if (vescd >= 0)
		{
			vesc_close(vescd);
		}
	}

	void VESC::close()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex);

		if (read_thread != NULL)
		{
			read_thread->interrupt();
			delete read_thread;
			read_thread = NULL;
		}

		if (vescd >= 0)
		{
			vesc_close(vescd);
			vescd = -1;
			ROS_INFO_NAMED(this_name, "Closed port to controller");
		}
	}

	void VESC::getFwVersion(uint8_t &major, uint8_t &minor)
	{
		int ret;

		ret = VESC::startIfNecessary();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		boost::mutex::scoped_lock lock(read_fw_version_mutex);

		io_mutex.lock_upgrade();
		ret = vesc_request_fw_version(vescd);
		io_mutex.unlock_upgrade();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		if (!read_fw_version_sig.timed_wait(lock, boost::posix_time::milliseconds(timeout)))
		{
			throw Exception(VESC_ERROR_TIMEOUT);
		}

		major = read_fw_version_major;
		minor = read_fw_version_minor;
	}

	void VESC::getStatus(double &velocity, double &position)
	{
		int ret;

		ret = VESC::startIfNecessary();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		boost::mutex::scoped_lock lock(read_values_mutex);

		io_mutex.lock_upgrade();
		ret = vesc_request_values(vescd);
		io_mutex.unlock_upgrade();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		if (!read_values_sig.timed_wait(lock, boost::posix_time::milliseconds(timeout)))
		{
			throw Exception(VESC_ERROR_TIMEOUT);
		}

		position = rad_per_tick * read_values.tachometer;
		velocity = read_values.rpm * M_PI * 2.0 / 60.0;
	}

	void VESC::open()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex);

		if (VESC::stat())
		{
			ROS_DEBUG_NAMED(this_name, "Tried to open the port, but it is already open");
			return;
		}

		vescd = vesc_open(port.c_str(), (enum VESC_BAUD)baud, 1);
		if (vescd < 0)
		{
			ROS_ERROR_THROTTLE_NAMED(1, this_name, "Attempt to open device '%s' failed: %s", port.c_str(), vesc_strerror(vescd));
			return;
		}

		vesc_set_cb_context(vescd, (void *)this);
		vesc_set_cb_get_config(vescd, getConfigCallbackWrapper);
		vesc_set_cb_get_fw_version(vescd, getFwVersionCallbackWrapper);
		vesc_set_cb_get_values(vescd, getValuesCallbackWrapper);
		vesc_set_cb_set_config(vescd, setConfigCallbackWrapper);

		if (read_thread == NULL)
		{
			read_thread = new boost::thread(&VESC::spin, this);
		}

		ROS_DEBUG_NAMED(this_name, "Opened port controller at '%s'", port.c_str());
	}

	void VESC::setCurrent(const double current)
	{
		int ret;

		ret = VESC::startIfNecessary();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		io_mutex.lock_upgrade();
		ret = vesc_set_current(vescd, current * 1000.0 + 0.5);
		io_mutex.unlock_upgrade();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}
	}

	void VESC::setVelocity(const double velocity)
	{
		int ret;

		ret =  VESC::startIfNecessary();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		io_mutex.lock_upgrade();
		ret = vesc_set_rpm(vescd, (velocity * 60.0 / (2.0 * M_PI)) + 0.5);
		io_mutex.unlock_upgrade();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}
	}

	bool VESC::stat()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex);

		return (vescd < 0) ? false : true;
	}

	void VESC::start()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex);

		if (!VESC::stat())
		{
			VESC::open();
		}

		diag_timer.start();

		if (VESC::stat())
		{
			try
			{
				uint8_t major;
				uint8_t minor;

				VESC::getFwVersion(major, minor);

				if (major < 1 || (major == 1 && minor < 5))
				{
					ROS_ERROR("This driver is not compatible with VESC firmware v%hhu.%hhu. Please load v1.5 or newer.", major, minor);

					VESC::close();

					return;
				}

				VESC::mergeSettings();

				if (dyn_re_srv == NULL)
				{
					dyn_re_srv = new dynamic_reconfigure::Server<vesc_driver::VESCConfig>(dyn_re_mutex, nh_priv);
				}

				dyn_re_srv->setCallback(dyn_re_cb);

				ROS_INFO_NAMED(this_name, "Connected to controller at '%s'", port.c_str());
			}
			catch (Exception &e)
			{
				ROS_ERROR_NAMED(this_name, "Communication failure during startup: %s", e.what());

				if (dyn_re_srv != NULL)
				{
					dyn_re_srv->clearCallback();
				}

				VESC::closeSilently();
			}
		}
	}

	void VESC::stop()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex);

		if (dyn_re_srv != NULL)
		{
			dyn_re_srv->clearCallback();
		}

		diag_timer.stop();
	}

	/**
	 * Private Functions
	 */
	void VESC::closeSilently()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex);

		if (read_thread != NULL)
		{
			read_thread->interrupt();
			delete read_thread;
			read_thread = NULL;
		}

		if (vescd >= 0)
		{
			vesc_close(vescd);
			vescd = -1;
		}
	}

	void VESC::configToDynRe(struct vesc_driver::VESCConfig &dyn_re_cfg, const struct vesc_config &vesc_cfg)
	{
		dyn_re_cfg.pwm_mode = vesc_cfg.pwm_mode;
		dyn_re_cfg.comm_mode = vesc_cfg.comm_mode;
		dyn_re_cfg.motor_type = vesc_cfg.motor_type;
		dyn_re_cfg.sensor_mode = vesc_cfg.sensor_mode;
		dyn_re_cfg.l_current_max = vesc_cfg.l_current_max / 1000.0;
		dyn_re_cfg.l_current_min = vesc_cfg.l_current_min / 1000.0;
		dyn_re_cfg.l_in_current_max = vesc_cfg.l_in_current_max / 1000.0;
		dyn_re_cfg.l_in_current_min = vesc_cfg.l_in_current_min / 1000.0;
		dyn_re_cfg.l_abs_current_max = vesc_cfg.l_abs_current_max / 1000.0;
		dyn_re_cfg.l_min_erpm = vesc_cfg.l_min_erpm / 1000.0;
		dyn_re_cfg.l_max_erpm = vesc_cfg.l_max_erpm / 1000.0;
		dyn_re_cfg.l_max_erpm_fbrake = vesc_cfg.l_max_erpm_fbrake / 1000.0;
		dyn_re_cfg.l_max_erpm_fbrake_cc = vesc_cfg.l_max_erpm_fbrake_cc / 1000.0;
		dyn_re_cfg.l_min_vin = vesc_cfg.l_min_vin / 1000.0;
		dyn_re_cfg.l_max_vin = vesc_cfg.l_max_vin / 1000.0;
		dyn_re_cfg.l_slow_abs_current = vesc_cfg.l_slow_abs_current;
		dyn_re_cfg.l_rpm_lim_neg_torque = vesc_cfg.l_rpm_lim_neg_torque;
		dyn_re_cfg.l_temp_fet_start = vesc_cfg.l_temp_fet_start / 1000.0;
		dyn_re_cfg.l_temp_fet_end = vesc_cfg.l_temp_fet_end / 1000.0;
		dyn_re_cfg.l_temp_motor_start = vesc_cfg.l_temp_motor_start / 1000.0;
		dyn_re_cfg.l_temp_motor_end = vesc_cfg.l_temp_motor_end / 1000.0;
		dyn_re_cfg.l_min_duty = vesc_cfg.l_min_duty / 1000000.0;
		dyn_re_cfg.l_max_duty = vesc_cfg.l_max_duty / 1000000.0;
		dyn_re_cfg.sl_min_erpm = vesc_cfg.sl_min_erpm / 1000.0;
		dyn_re_cfg.sl_min_erpm_cycle_int_limit = vesc_cfg.sl_min_erpm_cycle_int_limit / 1000.0;
		dyn_re_cfg.sl_max_fullbreak_current_dir_change = vesc_cfg.sl_max_fullbreak_current_dir_change / 1000.0;
		dyn_re_cfg.sl_cycle_int_limit = vesc_cfg.sl_cycle_int_limit / 1000.0;
		dyn_re_cfg.sl_phase_advance_at_br = vesc_cfg.sl_phase_advance_at_br / 1000.0;
		dyn_re_cfg.sl_cycle_int_rpm_br = vesc_cfg.sl_cycle_int_rpm_br / 1000.0;
		dyn_re_cfg.sl_bemf_coupling_k = vesc_cfg.sl_bemf_coupling_k / 1000.0;
		dyn_re_cfg.hall_table_0 = vesc_cfg.hall_table[0];
		dyn_re_cfg.hall_table_1 = vesc_cfg.hall_table[1];
		dyn_re_cfg.hall_table_2 = vesc_cfg.hall_table[2];
		dyn_re_cfg.hall_table_3 = vesc_cfg.hall_table[3];
		dyn_re_cfg.hall_table_4 = vesc_cfg.hall_table[4];
		dyn_re_cfg.hall_table_5 = vesc_cfg.hall_table[5];
		dyn_re_cfg.hall_table_6 = vesc_cfg.hall_table[6];
		dyn_re_cfg.hall_table_7 = vesc_cfg.hall_table[7];
		dyn_re_cfg.hall_sl_erpm = vesc_cfg.hall_sl_erpm / 1000.0;
		dyn_re_cfg.s_pid_kp = vesc_cfg.s_pid_kp / 1000000.0;
		dyn_re_cfg.s_pid_ki = vesc_cfg.s_pid_ki / 1000000.0;
		dyn_re_cfg.s_pid_kd = vesc_cfg.s_pid_kd / 1000000.0;
		dyn_re_cfg.s_pid_min_rpm = vesc_cfg.s_pid_min_rpm / 1000.0;
		dyn_re_cfg.p_pid_kp = vesc_cfg.p_pid_kp / 1000000.0;
		dyn_re_cfg.p_pid_ki = vesc_cfg.p_pid_ki / 1000000.0;
		dyn_re_cfg.p_pid_kd = vesc_cfg.p_pid_kd / 1000000.0;
		dyn_re_cfg.cc_startup_boost_duty = vesc_cfg.cc_startup_boost_duty / 1000000.0;
		dyn_re_cfg.cc_min_current = vesc_cfg.cc_min_current / 1000.0;
		dyn_re_cfg.cc_gain = vesc_cfg.cc_gain / 1000000.0;
		dyn_re_cfg.cc_ramp_step_max = vesc_cfg.cc_ramp_step_max / 1000000.0;
		dyn_re_cfg.m_fault_stop_time_ms = vesc_cfg.m_fault_stop_time_ms;
	}

	void VESC::diagTimerCallback(const ros::TimerEvent &event)
	{
		try
		{
			int ret;

			ret = VESC::startIfNecessary();
			if (ret != VESC_SUCCESS)
			{
				throw Exception((enum VESC_ERROR)ret);
			}

			// We do our own rate limiting
			diag.force_update();
		}
		catch(Exception &e)
		{
			if (e.vesc_error != VESC_ERROR_NOT_CONNECTED)
			{
				if (e.vesc_error != VESC_ERROR_TIMEOUT && e.vesc_error != VESC_ERROR_CHECKSUM_FAILURE && e.vesc_error != VESC_ERROR_BUSY)
				{
					ROS_ERROR_THROTTLE_NAMED(1, this_name, "Communication error while querying diagnostics: %s", e.what());

					VESC::close();
				}
				else
				{
					ROS_WARN_THROTTLE_NAMED(1, this_name, "Communication error while querying diagnostics: %s", e.what());
				}
			}
		}
	}

	void VESC::dynReCallback(vesc_driver::VESCConfig &config, uint32_t level)
	{
		int ret;

		ROS_DEBUG_NAMED(this_name, "Dynamic Reconfigure Callback");

		boost::mutex::scoped_lock cfg_lock(read_config_mutex);

		read_config.pwm_mode = config.pwm_mode;
		read_config.comm_mode = config.comm_mode;
		read_config.motor_type = config.motor_type;
		read_config.sensor_mode = config.sensor_mode;
		read_config.l_current_max = roundDouble(config.l_current_max * 1000.0);
		read_config.l_current_min = roundDouble(config.l_current_min * 1000.0);
		read_config.l_in_current_max = roundDouble(config.l_in_current_max * 1000.0);
		read_config.l_in_current_min = roundDouble(config.l_in_current_min * 1000.0);
		read_config.l_abs_current_max = roundDouble(config.l_abs_current_max * 1000.0);
		read_config.l_min_erpm = roundDouble(config.l_min_erpm * 1000.0);
		read_config.l_max_erpm = roundDouble(config.l_max_erpm * 1000.0);
		read_config.l_max_erpm_fbrake = roundDouble(config.l_max_erpm_fbrake * 1000.0);
		read_config.l_max_erpm_fbrake_cc = roundDouble(config.l_max_erpm_fbrake_cc * 1000.0);
		read_config.l_min_vin = roundDouble(config.l_min_vin * 1000.0);
		read_config.l_max_vin = roundDouble(config.l_max_vin * 1000.0);
		read_config.l_slow_abs_current = config.l_slow_abs_current;
		read_config.l_rpm_lim_neg_torque = config.l_rpm_lim_neg_torque;
		read_config.l_temp_fet_start = roundDouble(config.l_temp_fet_start * 1000.0);
		read_config.l_temp_fet_end = roundDouble(config.l_temp_fet_end * 1000.0);
		read_config.l_temp_motor_start = roundDouble(config.l_temp_motor_start * 1000.0);
		read_config.l_temp_motor_end = roundDouble(config.l_temp_motor_end * 1000.0);
		read_config.l_min_duty = roundDouble(config.l_min_duty * 1000000.0);
		read_config.l_max_duty = roundDouble(config.l_max_duty * 1000000.0);
		read_config.sl_min_erpm = roundDouble(config.sl_min_erpm * 1000.0);
		read_config.sl_min_erpm_cycle_int_limit = roundDouble(config.sl_min_erpm_cycle_int_limit * 1000.0);
		read_config.sl_max_fullbreak_current_dir_change = roundDouble(config.sl_max_fullbreak_current_dir_change * 1000.0);
		read_config.sl_cycle_int_limit = roundDouble(config.sl_cycle_int_limit * 1000.0);
		read_config.sl_phase_advance_at_br = roundDouble(config.sl_phase_advance_at_br * 1000.0);
		read_config.sl_cycle_int_rpm_br = roundDouble(config.sl_cycle_int_rpm_br * 1000.0);
		read_config.sl_bemf_coupling_k = roundDouble(config.sl_bemf_coupling_k * 1000.0);
		read_config.hall_table[0] = config.hall_table_0;
		read_config.hall_table[1] = config.hall_table_1;
		read_config.hall_table[2] = config.hall_table_2;
		read_config.hall_table[3] = config.hall_table_3;
		read_config.hall_table[4] = config.hall_table_4;
		read_config.hall_table[5] = config.hall_table_5;
		read_config.hall_table[6] = config.hall_table_6;
		read_config.hall_table[7] = config.hall_table_7;
		read_config.hall_sl_erpm = roundDouble(config.hall_sl_erpm * 1000.0);
		read_config.s_pid_kp = roundDouble(config.s_pid_kp * 1000000.0);
		read_config.s_pid_ki = roundDouble(config.s_pid_ki * 1000000.0);
		read_config.s_pid_kd = roundDouble(config.s_pid_kd * 1000000.0);
		read_config.s_pid_min_rpm = roundDouble(config.s_pid_min_rpm * 1000.0);
		read_config.p_pid_kp = roundDouble(config.p_pid_kp * 1000000.0);
		read_config.p_pid_ki = roundDouble(config.p_pid_ki * 1000000.0);
		read_config.p_pid_kd = roundDouble(config.p_pid_kd * 1000000.0);
		read_config.cc_startup_boost_duty = roundDouble(config.cc_startup_boost_duty * 1000000.0);
		read_config.cc_min_current = roundDouble(config.cc_min_current * 1000.0);
		read_config.cc_gain = roundDouble(config.cc_gain * 1000000.0);
		read_config.cc_ramp_step_max = roundDouble(config.cc_ramp_step_max * 1000000.0);
		read_config.m_fault_stop_time_ms = config.m_fault_stop_time_ms;

		cfg_lock.unlock();

		// Write the config back so the precision matches
		configToDynRe(config, read_config);

		boost::mutex::scoped_lock lock(write_config_mutex);

		io_mutex.lock_upgrade();
		ret = vesc_set_config(vescd, &read_config);
		io_mutex.unlock_upgrade();
		if (ret != VESC_SUCCESS)
		{
			ROS_ERROR_NAMED(this_name, "Failed to update VESC configuration: %s", vesc_strerror(ret));
			return;
		}

		if (!write_config_sig.timed_wait(lock, boost::posix_time::milliseconds(timeout)))
		{
			ROS_ERROR_NAMED(this_name, "Timeout updating VESC configuration");
			return;
		}

		ROS_DEBUG_NAMED(this_name, "Configuration written successfully");
	}

	vesc_driver::VESCConfig VESC::getConfig()
	{
		vesc_driver::VESCConfig cfg;
		int ret;

		boost::mutex::scoped_lock lock(read_config_mutex);

		io_mutex.lock_upgrade();
		ret = vesc_request_config(vescd);
		io_mutex.unlock_upgrade();
		if (ret != VESC_SUCCESS)
		{
			throw Exception((enum VESC_ERROR)ret);
		}

		if (!read_config_sig.timed_wait(lock, boost::posix_time::milliseconds(timeout)))
		{
			throw Exception(VESC_ERROR_TIMEOUT);
		}

		configToDynRe(cfg, read_config);

		return cfg;
	}

	void VESC::mergeSettings()
	{
		ROS_DEBUG_NAMED(this_name, "Merging settings");

		boost::recursive_mutex::scoped_lock lock(dyn_re_mutex);

		// Start with the settings already on the controller
		vesc_driver::VESCConfig cfg = VESC::getConfig();

		// Update any settings in the parameter server
		cfg.__fromServer__(nh_priv);
		cfg.__clamp__();

		// Update the parameter server
		if (dyn_re_srv != NULL)
		{
			dyn_re_srv->updateConfig(cfg);
		}
		else
		{
			cfg.__toServer__(nh_priv);
		}
	}

	void VESC::queryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		if (VESC::stat())
		{
			int ret;

			ROS_DEBUG_NAMED(this_name, "Getting diagnostics");

			boost::mutex::scoped_lock values_lock(read_values_mutex);

			io_mutex.lock_upgrade();
			ret = vesc_request_values(vescd);
			io_mutex.unlock_upgrade();
			if (ret != VESC_SUCCESS)
			{
				throw Exception((enum VESC_ERROR)ret);
			}

			if (!read_values_sig.timed_wait(values_lock, boost::posix_time::milliseconds(timeout)))
			{
				throw Exception(VESC_ERROR_TIMEOUT);
			}

			stat.add("MOSFET 1 Temperature", read_values.temp_mos1 / 10.0);
			stat.add("MOSFET 2 Temperature", read_values.temp_mos2 / 10.0);
			stat.add("MOSFET 3 Temperature", read_values.temp_mos3 / 10.0);
			stat.add("MOSFET 4 Temperature", read_values.temp_mos4 / 10.0);
			stat.add("MOSFET 5 Temperature", read_values.temp_mos5 / 10.0);
			stat.add("MOSFET 6 Temperature", read_values.temp_mos6 / 10.0);
			stat.add("PCB Temperature", read_values.temp_pcb / 10.0);
			stat.add("Motor Current", read_values.current_motor / 100.0);
			stat.add("Battery Current", read_values.current_in / 100.0);
			stat.add("Duty Cycle", read_values.duty_now / 1000.0);
			stat.add("Motor RPM", read_values.rpm);
			stat.add("Input Voltage", read_values.v_in / 10.0);
			stat.add("Amp-Hours", read_values.amp_hours / 10000.0);
			stat.add("Amp-Hours Charged", read_values.amp_hours_charged / 10000.0);
			stat.add("Watt-Hours", read_values.watt_hours / 10000.0);
			stat.add("Watt-Hours Charged", read_values.watt_hours_charged / 10000.0);
			stat.add("Tachometer Value", read_values.tachometer);
			stat.add("Tachometer Absolute Value", read_values.tachometer_abs);
			stat.addf("Fault Code", "%s (%d)", vesc_strfault((enum VESC_FAULT_CODE)read_values.fault_code), (int)read_values.fault_code);

			boost::mutex::scoped_lock fw_version_lock(read_fw_version_mutex);

			stat.addf("Firmware Version", "%hhu.%hhu", read_fw_version_major, read_fw_version_minor);

			if (read_values.fault_code == VESC_FAULT_CODE_NONE)
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "VESC Communication OK");
			}
			else
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "VESC is reporting fault(s)");
			}
		}
		else
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "VESC is disconnected");
		}
	}

	inline double VESC::roundDouble(const double x)
	{
		return x < 0.0 ? ceil(x - 0.5) : floor(x + 0.5);
	}

	void VESC::spin()
	{
		while (ros::ok())
		{
			boost::this_thread::interruption_point();
			spinOnce();
		}
	}

	void VESC::spinOnce()
	{
		int ret;

		io_mutex.lock_shared();
		ret = vesc_process(vescd);
		io_mutex.unlock_shared();
		if (ret != VESC_SUCCESS && ret != VESC_ERROR_TIMEOUT)
		{
			ROS_ERROR_NAMED(this_name, "Failed to read data from VESC: %s", vesc_strerror(ret));

			if (ret != VESC_ERROR_CHECKSUM_FAILURE)
			{
				VESC::close();
			}
		}
	}

	enum VESC_ERROR VESC::startIfNecessary()
	{
		boost::recursive_timed_mutex::scoped_lock lock(state_mutex, boost::posix_time::milliseconds(timeout));

		if (!lock)
		{
			return VESC_ERROR_BUSY;
		}

		if (!VESC::stat())
		{
			VESC::start();

			if (!VESC::stat())
			{
				return VESC_ERROR_NOT_CONNECTED;
			}
		}

		return VESC_SUCCESS;
	}

	int VESC::getConfigCallback(struct vesc_config *config)
	{
		boost::mutex::scoped_lock lock(read_config_mutex);

		read_config = *config;

		lock.unlock();

		read_config_sig.notify_all();

		return VESC_SUCCESS;
	}

	int VESC::getFwVersionCallback(uint8_t major, uint8_t minor)
	{
		boost::mutex::scoped_lock lock(read_fw_version_mutex);

		read_fw_version_major = major;
		read_fw_version_minor = minor;

		lock.unlock();

		read_fw_version_sig.notify_all();

		return VESC_SUCCESS;
	}

	int VESC::getValuesCallback(struct vesc_values *values)
	{
		boost::mutex::scoped_lock lock(read_values_mutex);

		read_values = *values;

		lock.unlock();

		read_values_sig.notify_all();

		return VESC_SUCCESS;
	}

	int VESC::setConfigCallback()
	{
		write_config_sig.notify_all();

		return VESC_SUCCESS;
	}
}
