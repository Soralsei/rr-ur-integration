#include "rr100_ur_worker/robot_controller.h"
namespace rhoban
{
    RobotController::RobotController(ros::NodeHandle nh_, ros::NodeHandle nh_priv) : nh(nh_)
    {
        std::string task_array_topic = nh_priv.param<std::string>("task_array_topic", "/compound_tasks");
        std::string task_topic = nh_priv.param<std::string>("task_topic", "/tasks");
        std::string ik_action = nh_priv.param<std::string>("ik_action", "/kinematics_server/goal_pose/");
        std::string joint_trajectory_action = nh_priv.param<std::string>("joint_trajectory_action", "/scaled_pos_joint_traj_controller/follow_joint_trajectory");
        std::string gripper_command_action = nh_priv.param<std::string>("gripper_command_action", "/gripper_controller/gripper_cmd");
        std::string placement_service = nh_priv.param<std::string>("placement_service", "/robot_base_placer/get_placement");
        std::string move_base_action = nh_priv.param<std::string>("move_base_action", "/move_base");

        update_timer = nh.createTimer(ros::Duration(0.1), &RobotController::update, this);
        compound_task_sub = nh.subscribe(task_array_topic, 10, &RobotController::taskarray_callback, this);
        task_sub = nh.subscribe(task_topic, 10, &RobotController::task_callback, this);

        tf = std::make_unique<tf2_ros::Buffer>(ros::Duration(100.0));
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf);

        try
        {
            ROS_INFO("Instantiating robot arm controller...");
            arm_controller = std::make_unique<ArmController>(ik_action, joint_trajectory_action, gripper_command_action);
            ROS_INFO("Instantiating robot placement controller...");
            placement_controller = std::make_unique<PlacementController>(nh, placement_service, move_base_action, *tf);
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
        auto succeeded = task->execute();

        if (!succeeded)
        {
            std::string name = task->get_name();
            ROS_WARN("RobotController : Failed to execute %s", name.c_str());
            if (task->should_retry())
            {
                task->retry();
                bool is_reaching_task = name.compare("ReachingTask") == 0;
                // If it is a reaching task that should be retried,
                // add wrap it in a CompoundTask preceded by a PlacementTask
                if (is_reaching_task)
                {
                    std::shared_ptr<CompoundTask> new_task = std::make_shared<CompoundTask>(
                        *arm_controller,
                        *placement_controller,
                        priority::Normal,
                        2);
                    std::shared_ptr<ReachingTask> reaching_task = std::dynamic_pointer_cast<ReachingTask>(task);
                    new_task->add(std::make_unique<PlacementTask>(*placement_controller, reaching_task->get_target()))
                        .add(reaching_task);
                    task_queue.pop_front();
                    task_queue.push_front(new_task);
                }
            }
        }
        ROS_INFO_COND(succeeded, "Task succeeded");
        task_queue.pop_front();
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
                task->add_reaching(*arm_controller, task_msg.target, *tf, task_msg.num_retries, task_msg.duration);
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
                task->add_gripper(*arm_controller, action, task_msg.num_retries, task_msg.gripper_position);
                break;

            case rr100_ur_worker::TaskArray::TYPE_PLACEMENT:
                task->add_placement(*placement_controller, task_msg.target, task_msg.num_retries);
                break;

            default:
                ROS_ERROR("RobotController : unknown task type %d", task_msg.type);
                return;
            }
        }
        add_task(task);
    }

    void RobotController::task_callback(const rr100_ur_worker::TaskConstPtr &task_msg)
    {
        const std::lock_guard<std::mutex> guard(task_queue_mutex);
        std::shared_ptr<Task> task;

        geometry_msgs::TransformStamped target_to_map;
        geometry_msgs::PoseStamped transformed_target;

        switch (task_msg->type)
        {
        case rr100_ur_worker::TaskArray::TYPE_REACHING:
            try
            {
                target_to_map = tf->lookupTransform("map", task_msg->target.header.frame_id, ros::Time(0));
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
                return;
            }
            tf2::doTransform(task_msg->target, transformed_target, target_to_map);
            task = std::make_shared<ReachingTask>(*arm_controller, transformed_target, task_msg->duration);
            break;

        case rr100_ur_worker::TaskArray::TYPE_GRIPPER:
            GripperTask::Action action;
            switch (task_msg->action)
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
            task = std::make_shared<GripperTask>(*arm_controller, action, task_msg->gripper_position);
            break;

        case rr100_ur_worker::TaskArray::TYPE_PLACEMENT:
            task = std::make_shared<PlacementTask>(*placement_controller, task_msg->target);
            break;

        default:
            ROS_ERROR("RobotController : unknown task type %d", task_msg->type);
            return;
        }
        add_task(task);
    }
} // namespace rhoban
