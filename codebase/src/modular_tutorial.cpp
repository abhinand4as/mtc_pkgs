/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

Task createModule() {
	Task t;
        t.stages()->setName("Serial Container");
        const std::string group = "panda_arm";
	auto c = std::make_unique<SerialContainer>("Cartesian Path");
	c->setProperty("group", group);
	t.loadRobotModel();

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	
	{
                auto& state = scene->getCurrentStateNonConst();
                state.setToDefaultValues(state.getJointModelGroup(group), "ready");

                auto fixed = std::make_unique<stages::FixedState>("initial state");
                fixed->setState(scene);
                t.add(std::move(fixed));
        }

        {
                auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
               // stage->setGroup(group);
               // geometry_msgs::Vector3Stamped direction;
               // direction.header.frame_id = "world";
               // direction.vector.x = 0.2;
               // stage->setDirection(direction);
               // t.add(std::move(stage));

		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.0; 
		direction.vector.y = 0.0;
		direction.vector.z = 0.0;
		stage->setDirection(direction);
		c->insert(std::move(stage));
        }

//	{
//		
//		auto stage = std::make_unique<stages::MoveRelative>("y +0.1", cartesian);
//		stage->properties().configureInitFrom(Stage::PARENT);
//		geometry_msgs::Vector3Stamped direction;
//		direction.header.frame_id = "world";
//		direction.vector.y = 0.1;
// 		stage->setDirection(direction);	
//		c->insert(std::move(stage));
//	}

	t.add(std::move(c));
        return t;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "mtc_tutorial");
        // run an asynchronous spinner to communicate with the move_group node and rviz
        ros::AsyncSpinner spinner(1);
        spinner.start();

        auto task = createModule();
        try {
                if (task.plan())
                        task.introspection().publishSolution(*task.solutions().front());
        } catch (const InitStageException& ex) {
                std::cerr << "planning failed with exception" << std::endl << ex << task;
        }

        ros::waitForShutdown();  // keep alive for interactive inspection in rviz
        return 0;
}
 
