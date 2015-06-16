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
	const int VESC::default_timeout = VESC_TIMEOUT_DEFAULT;

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
		  read_thread(NULL),
		  this_name("vesc_driver"),
		  vescd(-1)
	{
		if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
		{
			ros::console::notifyLoggerLevelsChanged();
		}

		nh_priv.param("baud", baud, VESC::default_baud);
		nh_priv.param("port", port, VESC::default_port);
		nh_priv.param("timeout", timeout, VESC::default_timeout);

		diag.setHardwareIDf("VESC on '%s'", port.c_str());
		diag.add("VESC Status", this, &VESC::queryDiagnostics);
	}

	VESC::~VESC()
	{
		read_thread->interrupt();
		delete read_thread;

		delete dyn_re_srv;

		if (vescd >= 0)
		{
			vesc_close(vescd);
		}
	}

	void VESC::close()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		read_thread->interrupt();
		delete read_thread;
		read_thread = NULL;

		if (vescd >= 0)
		{
			vesc_close(vescd);
			vescd = -1;
			ROS_INFO_NAMED(this_name, "Disconnected from bus");
		}
	}

	void VESC::open()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (VESC::stat())
		{
			ROS_DEBUG_NAMED(this_name, "Tried to open the port, but it is already open");
			return;
		}

		vescd = vesc_open(port.c_str(), (enum VESC_BAUD)baud, (uint8_t)timeout);
		if (vescd < 0)
		{
			ROS_WARN_THROTTLE_NAMED(1, this_name, "Attempt to open device '%s' failed: %s", port.c_str(), vesc_strerror(vescd));
			return;
		}

		vesc_set_cb_context(vescd, (void *)this);
		vesc_set_cb_get_config(vescd, getConfigCallbackWrapper);
		vesc_set_cb_get_fw_version(vescd, getFwVersionCallbackWrapper);
		vesc_set_cb_get_values(vescd, getValuesCallbackWrapper);

		if (read_thread == NULL)
		{
			read_thread = new boost::thread(&VESC::spin, this);
		}
		ROS_INFO_NAMED(this_name, "Connected to bus at '%s'", port.c_str());
	}

	bool VESC::stat()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		return (vescd < 0) ? false : true;
	}

	void VESC::start()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (!VESC::stat())
		{
			VESC::open();
		}

		diag_timer.start();

		if (VESC::stat())
		{
			try
			{
				VESC::mergeSettings();

				if (dyn_re_srv == NULL)
				{
					dyn_re_srv = new dynamic_reconfigure::Server<vesc_driver::VESCConfig>(dyn_re_mutex, nh_priv);
				}
				dyn_re_srv->setCallback(dyn_re_cb);
			}
			catch (Exception &e)
			{
				ROS_ERROR_NAMED(this_name, "Communication failure: %s", e.what());

				if (dyn_re_srv != NULL)
				{
					dyn_re_srv->clearCallback();
				}

				VESC::close();
			}
		}
	}

	void VESC::stop()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		dyn_re_srv->clearCallback();

		diag_timer.stop();
	}

	/**
	 * Private Functions
	 */
	void VESC::diagTimerCallback(const ros::TimerEvent &event)
	{
		try
		{
			diag.update();
		}
		catch(Exception &e)
		{
			ROS_ERROR_NAMED(this_name, "Communication error: %s", e.what());
			if (e.vesc_error != VESC_ERROR_TIMEOUT && e.vesc_error != VESC_ERROR_CHECKSUM_FAILURE)
			{
				VESC::close();
			}
		}
	}

	void VESC::dynReCallback(vesc_driver::VESCConfig &config, uint32_t level)
	{
		//int ret;

		ROS_DEBUG_NAMED(this_name, "Dynamic Reconfigure Callback");

		/*last_settings.min_angle_limit = config.min_angle_limit;
		last_settings.max_angle_limit = config.max_angle_limit;
		last_settings.limit_temperature = config.limit_temperature;
		last_settings.max_limit_voltage = config.max_limit_voltage;
		last_settings.min_limit_voltage = config.min_limit_voltage;
		last_settings.max_torque = config.max_torque;
		last_settings.compliance_p = config.compliance_p;
		last_settings.compliance_d = config.compliance_d;
		last_settings.compliance_i = config.compliance_i;
		last_settings.imax = config.imax;

		rad_offset = config.rad_offset;
		rad_per_tick = config.rad_per_tick;*/

		//ret = sc_write_settings(parent->vescd, id, &last_settings);
		//if (ret != VESC_SUCCESS)
		//{
		//	ROS_ERROR_NAMED(this_name, "Failed to update VESC settings: %s", vesc_strerror(ret));
		//}
	}

	vesc_driver::VESCConfig VESC::getConfig()
	{
		vesc_driver::VESCConfig cfg;
		int ret;

		read_config_mutex.lock();

		ret = vesc_request_config(vescd);
		if (ret != VESC_SUCCESS)
		{
			read_config_mutex.unlock();
			throw Exception((enum VESC_ERROR)ret);
		}

		if (!read_config_mutex.timed_lock(boost::posix_time::milliseconds(timeout * 100)))
		{
			read_config_mutex.unlock();
			throw Exception(VESC_ERROR_TIMEOUT);
		}

		/*cfg.min_angle_limit = last_settings.min_angle_limit;
		cfg.max_angle_limit = last_settings.max_angle_limit;
		cfg.limit_temperature = last_settings.limit_temperature;
		cfg.max_limit_voltage = last_settings.max_limit_voltage;
		cfg.min_limit_voltage = last_settings.min_limit_voltage;
		cfg.max_torque = last_settings.max_torque;
		cfg.compliance_p = last_settings.compliance_p;
		cfg.compliance_d = last_settings.compliance_d;
		cfg.compliance_i = last_settings.compliance_i;
		cfg.imax = last_settings.imax;*/

		read_config_mutex.unlock();

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
		if (!VESC::stat())
		{
			VESC::start();
		}

		if (VESC::stat())
		{
			int ret;

			ROS_DEBUG_NAMED(this_name, "Getting diagnostics");

			read_values_mutex.lock();

			ret = vesc_request_values(vescd);
			if (ret != VESC_SUCCESS)
			{
				read_values_mutex.unlock();
				throw Exception((enum VESC_ERROR)ret);
			}

			read_fw_version_mutex.lock();

			ret = vesc_request_fw_version(vescd);
			if (ret != VESC_SUCCESS)
			{
				read_values_mutex.unlock();
				read_fw_version_mutex.unlock();
				throw Exception((enum VESC_ERROR)ret);
			}

			if (!read_values_mutex.timed_lock(boost::posix_time::milliseconds(timeout * 100)))
			{
				read_values_mutex.unlock();
				read_fw_version_mutex.unlock();
				throw Exception(VESC_ERROR_TIMEOUT);
			}

			if (!read_fw_version_mutex.timed_lock(boost::posix_time::milliseconds(timeout * 100)))
			{
				read_fw_version_mutex.unlock();
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
			stat.add("Fault Code", (int)read_values.fault_code);
			stat.addf("Firmware Version", "%hhu.%hhu", read_fw_version_major, read_fw_version_minor);

			if (read_values.fault_code == VESC_FAULT_CODE_NONE)
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "VESC Communication OK");
			}
			else
			{
				stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "VESC is reporting fault(s)");
			}

			read_fw_version_mutex.unlock();
			read_values_mutex.unlock();

			diag_timer.setPeriod(ros::Duration(1.0));
		}
		else
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "VESC is disconnected");
		}
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

		ret = vesc_process(vescd);
		if (ret != VESC_SUCCESS && ret != VESC_ERROR_TIMEOUT)
		{
			ROS_ERROR_NAMED(this_name, "Failed to read data from VESC: %s", vesc_strerror(ret));

			if (ret != VESC_ERROR_CHECKSUM_FAILURE)
			{
				boost::recursive_mutex::scoped_lock lock(io_mutex);
				VESC::close();
			}
		}
	}

	int VESC::getConfigCallback(struct vesc_config *config)
	{
		read_config = *config;

		read_config_mutex.unlock();

		return VESC_SUCCESS;
	}

	int VESC::getFwVersionCallback(uint8_t major, uint8_t minor)
	{
		read_fw_version_major = major;
		read_fw_version_minor = minor;

		read_fw_version_mutex.unlock();

		return VESC_SUCCESS;
	}

	int VESC::getValuesCallback(struct vesc_values *values)
	{
		read_values = *values;

		read_values_mutex.unlock();

		return VESC_SUCCESS;
	}
}
