#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/fix_collision_objects.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace moveit::task_constructor;

void spawnObject(const planning_scene::PlanningScenePtr& scene) {
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.45;
	o.primitive_poses[0].position.y = 0.23;
	o.primitive_poses[0].position.z = 0.3;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	scene->processCollisionObjectMsg(o);
}


Task createTask(){


    Eigen::Isometry3d grasp_frame_transform_ = Eigen::Isometry3d::Identity();
    grasp_frame_transform_.translation() << 0.0, 0.0, 0.1;
	//grasp_frame_transform_.rotate(Eigen::AngleAxisf (0.5*M_PI, Eigen::Vector3f::UnitX()));

    tf2::Quaternion q;
    q.setRPY(0.785, 1.571, 0.0);

    Eigen::Quaterniond quat(q.getW(), q.getX(), q.getY(), q.getZ());
    grasp_frame_transform_.rotate(quat);


	ROS_INFO("INSIDE PICK PLACE");
	Task t;
	t.stages()->setName("Pick and Place");
	t.loadRobotModel();
	
	//properties
	t.setProperty("group", std::string("panda_arm"));
	t.setProperty("eef", std::string("hand"));
	t.setProperty("gripper", std::string("hand"));

	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();
	
	Stage* initial_stage = nullptr;
	// create a fixed initial scene
	{
		auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues();  // initialize state
		state.setToDefaultValues(state.getJointModelGroup("panda_arm"), "ready");
		state.update();
		spawnObject(scene);

		auto initial = std::make_unique<stages::FixedState>();
		initial->setState(scene);
		initial_stage = initial.get();
		t.add(std::move(initial));
	}

	{
		stages::Connect::GroupPlannerVector planners = { { "panda_arm", pipeline }, { "hand", pipeline } };
		auto move = std::make_unique<stages::Connect>("connect", planners);
		move->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("approach object", cartesian);
		move->restrictDirection(stages::MoveRelative::BACKWARD);
		move->properties().configureInitFrom(Stage::PARENT);
		move->properties().set("marker_ns", std::string("approach"));
		move->setIKFrame("panda_link8");
		move->setMinMaxDistance(0.05, 0.1);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "panda_link8";
		direction.vector.z = 1;
		move->setDirection(direction);
		t.add(std::move(move));
	}

	{
		auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		gengrasp->properties().configureInitFrom(Stage::PARENT);
		gengrasp->setPreGraspPose("open");
		gengrasp->setObject("object");
		gengrasp->setAngleDelta(M_PI / 10.);
		gengrasp->setMonitoredStage(initial_stage);

		auto filter = std::make_unique<stages::PredicateFilter>("filtered");
		gengrasp->properties().exposeTo(filter->properties(), { "eef" });
		filter->properties().configureInitFrom(Stage::PARENT);
		filter->insert(std::move(gengrasp));
		filter->setPredicate([](const SolutionBase& s, std::string& comment) {
			bool accept = s.cost() < 2;
			if (!accept)
				comment += " (rejected)";
			return accept;
		});

		auto ik = std::make_unique<stages::ComputeIK>("compute ik", std::move(filter));
		PropertyMap& props = ik->properties();
		props.configureInitFrom(Stage::PARENT, { "group", "eef", "default_pose" });
		props.configureInitFrom(Stage::INTERFACE, { "target_pose" });  // derived from child's solution
		ik->setIKFrame( grasp_frame_transform_, "panda_link8");;
		ik->setMaxIKSolutions(1);
		t.add(std::move(ik));
	}

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("allow object collision");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

		move->allowCollisions(
		    "object", t.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveTo>("close gripper", pipeline);
		move->restrictDirection(stages::MoveTo::FORWARD);
		move->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		move->setGoal("close");
		t.add(std::move(move));
	}
	

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("attach object");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);
		move->attachObject("object", "panda_link8");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("lift object", cartesian);
		move->properties().configureInitFrom(Stage::PARENT, { "group" });
		move->setMinMaxDistance(0.03, 0.05);
		move->properties().set("marker_ns", std::string("lift"));
		move->setIKFrame("panda_link8");

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 1;
		move->setDirection(direction);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("shift object", cartesian);
		move->properties().configureInitFrom(Stage::PARENT, { "group" });
		move->setMinMaxDistance(0.1, 0.2);
		move->properties().set("marker_ns", std::string("lift"));
		move->setIKFrame("panda_link8");

		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "object";
		twist.twist.linear.y = 2;
		//twist.twist.angular.y = 2;
		move->setDirection(twist);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveTo>("open gripper", pipeline);
		move->restrictDirection(stages::MoveTo::FORWARD);
		move->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		move->setGoal("open");
		t.add(std::move(move));
	}

	{
        auto stage = std::make_unique<stages::ModifyPlanningScene>("dettach object");
        stage->detachObject("object", "panda_link8");
        t.add(std::move(stage));
    }

	// {
	// 	auto stage = std::make_unique<stages::MoveRelative>("retreat after place", pipeline);
	// 	stage->properties().configureInitFrom(Stage::PARENT, { "group" });
	// 	stage->setMinMaxDistance(.08, .25);
	// 	stage->setIKFrame("panda_link8");
	// 	stage->properties().set("marker_ns", "retreat");
	// 	geometry_msgs::Vector3Stamped vec;
	// 	vec.header.frame_id = "panda_link8";
	// 	vec.vector.z = -1.0;
	// 	stage->setDirection(vec);
	// 	t.add(std::move(stage));
	// }

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("allow object collision");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

		move->allowCollisions(
		    "object", t.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(), false);
		t.add(std::move(move));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("move home", pipeline);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal("ready");
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

return t;

}


int main(int argc, char** argv) {
	    
	    ros::init(argc, argv, "mtc_tutorial");
        // run an asynchronous spinner to communicate with the move_group node and rviz
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
        
        auto task = createTask();

        try {
                if (task.plan())
                        task.introspection().publishSolution(*task.solutions().front());
        } catch (const InitStageException& ex) {
                std::cerr << "planning failed with exception" << std::endl << ex << task;
        }

        ros::waitForShutdown();  // keep alive for interactive inspection in rviz
        return 0;

}
