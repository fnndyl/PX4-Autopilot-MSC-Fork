/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * 
 * This customized controller is designed to control a moving platform with more
 * degrees of freedom than the standard MovingPlatformController. Need a PD 
 * position controller for all 6 axes. Noise is not necessary but can be added.
 * Must also be some way of publishing a given trajectory to the platform through 
 * ROS2.
 * 
 *****************************************************************************/

#include "BargeController.hpp"

 using namespace custom;

 // Register the plugin
 GZ_ADD_PLUGIN(
    BargeController,
    gz::sim::System,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemConfigure
 )

 // Configure system - runs on startup
 void BargeController::Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
{

    _entity = entity;
    _model = gz::sim::Model(entity);

    const std::string link_name = sdf->Get<std::string>("link_name");
    _link_entity = _model.LinkByName(ecm, link_name);

    if (!_link_entity) {
		throw std::runtime_error("BargeController::Configure: Link \"" + link_name + "\" was not found. "
					 "Please ensure that your model contains the corresponding link.");
	}

    _link = gz::sim::Link(_link_entity);

    // Need to report linear & angular velocity
    _link.EnableVelocityChecks(ecm, true);

    _world_entity = gz::sim::worldEntity(ecm);
    _world = gz::sim::World(_world_entity);

    // getVehicleModelName();



    _startup_timer = gz::common::Timer();
    _startup_timer.Start();

    // Initialize node + get setpoints from function
    {
		std::string topic = "/x500/odom";

		// Subscribe to node and register a callback
		if (!_node.Subscribe(topic, &BargeController::bargeCallback, this))
		{
			gzerr << "Error subscribing to topic [" << topic << "]" << std::endl;
		}

	}
	
    // Get gravity, model mass, platform height.
    {
        const auto gravity = _world.Gravity(ecm);

        if (gravity.has_value()) {
            _gravity = gravity.value().Z();
        } else {
            gzwarn << "Unable to get gazebo world gravity. Keeping default of " << _gravity << std::endl;
        }

        auto inertial_component = ecm.Component<gz::sim::components::Inertial>(_link_entity);

		if (inertial_component) {
			_platform_mass = inertial_component->Data().MassMatrix().Mass();
			_platform_diag_moments = inertial_component->Data().MassMatrix().DiagonalMoments();

		} else {
			gzwarn << "Unable to get inertial component for link " << link_name << ". Keeping default mass of " << _platform_mass <<
			       std::endl;
		}
    }
}

void BargeController::PreUpdate(const gz::sim::UpdateInfo &info, 
	gz::sim::EntityComponentManager &ecm) 
{
	// Wait for vehicle to spawn
	// const bool vehicle_has_spawned = 0 != _world.ModelByName(ecm, _vehicle_model_name);
	// const bool keep_stationary = _wait_for_vehicle_spawned && !vehicle_has_spawned;

	// gzdbg << "Did something else" << std::endl;

}

void BargeController::getPlatformSetpoint() 
{

}

void BargeController::bargeCallback(const gz::msgs::Odometry &_msg) {



}