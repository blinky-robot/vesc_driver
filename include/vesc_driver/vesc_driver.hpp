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

#include <boost/thread/mutex.hpp>

#include <vesc/vesc.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>

#include "vesc_driver/VESCConfig.h"

namespace vesc_driver
{
	class Exception : public ros::Exception
	{
	public:
		Exception(const enum VESC_ERROR vesc_error);

		const enum VESC_ERROR vesc_error;
	};

	class VESC
	{
	public:
		VESC(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
		~VESC();

		void close();
		void getFwVersion(uint8_t &major, uint8_t &minor);
		void getStatus(double &velocity, double &position);
		void open();
		void setCurrent(const double current);
		void setVelocity(const double velocity);
		void start();
		bool stat();
		void stop();

		static const int default_baud;
		static const std::string default_port;
		static const int default_timeout;

	private:
		static void configToDynRe(struct vesc_driver::VESCConfig &dyn_re_cfg, const struct vesc_config &vesc_cfg);
		void diagTimerCallback(const ros::TimerEvent &event);
		void dynReCallback(vesc_driver::VESCConfig &config, uint32_t level);
		vesc_driver::VESCConfig getConfig();
		void mergeSettings();
		void queryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
		static inline double roundDouble(const double x);
		void spin();
		void spinOnce();

		friend int getConfigCallbackWrapper(void *context, struct vesc_config *config);
		int getConfigCallback(struct vesc_config *config);
		friend int getFwVersionCallbackWrapper(void *context, uint8_t major, uint8_t minor);
		int getFwVersionCallback(uint8_t major, uint8_t minor);
		friend int getValuesCallbackWrapper(void *context, struct vesc_values *values);
		int getValuesCallback(struct vesc_values *values);
		friend int setConfigCallbackWrapper(void *context);
		int setConfigCallback();

		bool active;
		int baud;
		diagnostic_updater::Updater diag;
		ros::Timer diag_timer;
		const dynamic_reconfigure::Server<vesc_driver::VESCConfig>::CallbackType dyn_re_cb;
		boost::recursive_mutex dyn_re_mutex;
		dynamic_reconfigure::Server<vesc_driver::VESCConfig> *dyn_re_srv;
		boost::recursive_mutex io_mutex;
		const ros::NodeHandle nh;
		const ros::NodeHandle nh_priv;
		std::string port;
		double rad_per_tick;
		uint8_t read_fw_version_major;
		uint8_t read_fw_version_minor;
		boost::timed_mutex read_fw_version_mutex;
		struct vesc_config read_config;
		boost::timed_mutex read_config_mutex;
		boost::thread *read_thread;
		struct vesc_values read_values;
		boost::timed_mutex read_values_mutex;
		const std::string this_name;
		int timeout;
		int vescd;
		boost::timed_mutex write_config_mutex;
	};
}

