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

#ifndef _vesc_handle_hpp
#define _vesc_handle_hpp

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <ros/ros.h>

#include "vesc_driver/vesc_driver.hpp"

namespace vesc_driver
{
	class VESCHandle
	{
	public:
		VESCHandle(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
		~VESCHandle();

		void registerStateHandle(hardware_interface::ActuatorStateInterface &asi);
		void registerVelocityHandle(hardware_interface::VelocityActuatorInterface &avi);

		void read();
		void write();

	private:
		std::string getActuatorName() const;

		const ros::NodeHandle nh;
		const ros::NodeHandle nh_priv;

		vesc_driver::VESC vesc;
		std::string this_name;

		bool actuator_engaged;

		double actuator_cmd_vel;
		double actuator_eff;
		double actuator_pos;
		double actuator_vel;

		hardware_interface::ActuatorStateHandle actuator_handle;
		hardware_interface::ActuatorHandle actuator_handle_cmd_vel;
	};
}

#endif /* _vesc_handle_hpp */
