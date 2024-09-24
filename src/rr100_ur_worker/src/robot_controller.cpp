#include "rr100_ur_worker/robot_controller.h"
namespace rhoban
{
    RobotController::RobotController(ros::NodeHandle nh_, ros::NodeHandle nh_priv) : nh(nh_)
    {
        std::string task_array_topic = nh_priv.param<std::string>("task_array_topic", "/compound_tasks");
        std::string ik_action = nh_priv.param<std::string>("ik_action", "/kinematics_server/goal_pose/");
        std::string joint_trajectory_action = nh_priv.param<std::string>("joint_trajectory_action", "/scaled_pos_joint_traj_controller/follow_joint_trajectory");
        std::string gripper_command_action = nh_priv.param<std::string>("gripper_command_action", "/gripper_controller/gripper_cmd");
        std::string placement_service = nh_priv.param<std::string>("placement_service", "/robot_base_placer/get_placement");
        std::string move_base_action = nh_priv.param<std::string>("move_base_action", "/move_base");

        update_timer = nh.createTimer(ros::Duration(0.1), &RobotController::update, this);
        task_sub = nh.subscribe(task_array_topic, 10, &RobotController::taskarray_callback, this);

        try
        {
            ROS_INFO("Instantiating robot arm controller...");
            arm_controller = std::make_unique<ArmController>(ik_action, joint_trajectory_action, gripper_command_action);
            ROS_INFO("Instantiating robot placement controller...");
            placement_controller = std::make_unique<PlacementController>(nh, placement_service, move_base_action);
        }
        catch (const std::exception &e)
        {
            ROS_FATAL_STREAM("RobotController : " << e.what());
            std::exit(-1);
        }
    }

    RobotController::~RobotController()
    {
    }

    void RobotController::add_task(TaskPtr task)
    {
        ROS_INFO("Adding task %s", task->get_name().c_str());
        task_queue.push_back(task);
    }

    void RobotController::update(const ros::TimerEvent &)
    {
        const std::lock_guard<std::mutex> guard(task_queue_mutex);

        if (task_queue.empty())
            return;

        auto task = task_queue.front();
        auto res = task->execute();

        if (!res)
        {
            std::string name = task->get_name();
            ROS_WARN("RobotController : Failed to execute %s", name.c_str());
            if (name.compare("ReachingTask") == 0)
            {
                std::shared_ptr<ReachingTask> reaching_task = std::dynamic_pointer_cast<ReachingTask>(task);
                task_queue.insert(task_queue.begin(), std::make_unique<PlacementTask>(*placement_controller, reaching_task->get_target()));
            }
        } else 
        {
            ROS_INFO("%s successful", task->get_name().c_str());
            task_queue.erase(task_queue.begin());
        };
    }

    void RobotController::taskarray_callback(const rr100_ur_worker::TaskArrayConstPtr &task_array)
    {
        const std::lock_guard<std::mutex> guard(task_queue_mutex);
        std::shared_ptr<CompoundTask> task =
            std::make_shared<CompoundTask>(*arm_controller, *placement_controller, priority::Normal, task_array->tasks.size());
        for (auto &&task_msg : task_array->tasks)
        {
            switch (task_msg.type)
            {
            case rr100_ur_worker::TaskArray::TYPE_REACHING:
                task->add_reaching(*arm_controller, task_msg.target, task_msg.duration);
                break;
            case rr100_ur_worker::TaskArray::TYPE_GRIPPER:
                GripperTask::Action action;
                switch (task_msg.action)
                {
                case rr100_ur_worker::Task::ACTION_GRIP:
                    action = GripperTask::Action::Grip;
                    break;
                case rr100_ur_worker::Task::ACTION_RELEASE:
                    action = GripperTask::Action::Release;

                    break;
                case rr100_ur_worker::Task::ACTION_SET:
                default:
                    action = GripperTask::Action::Set;
                    break;
                }
                task->add_gripper(*arm_controller, action, task_msg.gripper_position);
                break;
            case rr100_ur_worker::TaskArray::TYPE_PLACEMENT:
                task->add_placement(*placement_controller, task_msg.target);
                break;
            default:
                ROS_ERROR("RobotController : unknown task type %d", task_msg.type);
                return;
            }
        }
        task_queue.push_back(task);
    }
} // namespace rhoban
