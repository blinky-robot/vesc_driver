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

#include "vesc_driver/vesc_handle.hpp"

namespace vesc_driver
{
	VESCHandle::VESCHandle(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
		: nh(nh),
		  nh_priv(nh_priv),
		  vesc(nh, nh_priv),
		  this_name("vesc_driver"),
		  actuator_engaged(false),
		  actuator_cmd_vel(std::numeric_limits<double>::quiet_NaN()),
		  actuator_eff(std::numeric_limits<double>::quiet_NaN()),
		  actuator_pos(std::numeric_limits<double>::quiet_NaN()),
		  actuator_vel(std::numeric_limits<double>::quiet_NaN()),
		  actuator_handle(getActuatorName(), &actuator_pos, &actuator_vel, &actuator_eff),
		  actuator_handle_cmd_vel(actuator_handle, &actuator_cmd_vel)
	{
		vesc.start();
	}

	VESCHandle::~VESCHandle()
	{
		vesc.stop();
	}

	void VESCHandle::registerStateHandle(hardware_interface::ActuatorStateInterface &asi)
	{
		asi.registerHandle(actuator_handle);
	}

	void VESCHandle::registerVelocityHandle(hardware_interface::VelocityActuatorInterface &avi)
	{
		avi.registerHandle(actuator_handle_cmd_vel);
	}

	void VESCHandle::read()
	{
		try
		{
			vesc.getStatus(actuator_vel, actuator_pos);
		}
		catch(vesc_driver::Exception &e)
		{
			if (e.vesc_error != VESC_ERROR_NOT_CONNECTED)
			{
				if (e.vesc_error != VESC_ERROR_TIMEOUT && e.vesc_error != VESC_ERROR_CHECKSUM_FAILURE && e.vesc_error != VESC_ERROR_BUSY)
				{
					ROS_ERROR_THROTTLE_NAMED(1, this_name, "Failed to read state of ESC: %s", e.what());

					vesc.close();
				}
				else
				{
					ROS_WARN_THROTTLE_NAMED(1, this_name, "Failed to read state of ESC: %s", e.what());
				}
			}
		}
	}

	void VESCHandle::write()
	{
		try
		{
			if (isnan(actuator_cmd_vel))
			{
				actuator_engaged = false;

				vesc.setCurrent(0.0);
			}
			else
			{
				actuator_engaged = true;

				vesc.setVelocity(actuator_cmd_vel);
			}
		}
		catch(vesc_driver::Exception &e)
		{
			if (e.vesc_error != VESC_ERROR_NOT_CONNECTED)
			{
				if (e.vesc_error != VESC_ERROR_TIMEOUT && e.vesc_error != VESC_ERROR_CHECKSUM_FAILURE && e.vesc_error != VESC_ERROR_BUSY)
				{
					ROS_ERROR_THROTTLE_NAMED(1, this_name, "Failed to command ESC: %s", e.what());

					vesc.close();
				}
				else
				{
					ROS_WARN_THROTTLE_NAMED(1, this_name, "Failed to command ESC: %s", e.what());
				}
			}
		}
	}

	std::string VESCHandle::getActuatorName() const
	{
		std::string actuator_name;

		if (!nh_priv.getParam("actuator_name", actuator_name))
		{
			actuator_name = "vesc";
			ROS_WARN_NAMED(this_name, "Actuator name for ESC is not set - defaulting to '%s'", actuator_name.c_str());
		}

		return actuator_name;
	}
}
