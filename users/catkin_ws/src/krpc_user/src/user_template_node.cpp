#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <platform_msgs/UserLogic.h>
#include <platform_msgs/StopProcessingUserNode.h>
#include <geometry_msgs/PoseStamped.h>
#include <ib2_msgs/CtlCommandAction.h>
#include <ib2_msgs/CtlCommandGoal.h>
#include <ib2_msgs/CtlStatusType.h>
#include <krpc_ib2_msgs/CheckPoints.h>
#include <krpc_ib2_msgs/MissionFinished.h>
#include <krpc_ib2_msgs/MissionStart.h>
#include <krpc_ib2_msgs/CheckpointPassedAction.h>
#include <krpc_ib2_msgs/CheckpointPassedGoal.h>
#include <algorithm>
#include <regex>
#include <string>

struct CheckpointTarget {
    int checkpoint_id;
    geometry_msgs::PoseStamped pose;
};

class UserTemplate {
public:
    UserTemplate(ros::NodeHandle& nh) :
        nh_(nh),
        command_client_("/ctl/command", true),
        checkpoint_client_("/event/checkpoint_passed", true),
        action_active_(false),
        started_(false)
    {
        start_sub_ = nh_.subscribe("/ib2_user/start", 1, &UserTemplate::startCallback, this); // DO NOT REMOVE THIS LINE
        stop_service_ = nh_.advertiseService("/ib2_user/stop", &UserTemplate::stopProcessing, this); // DO NOT REMOVE THIS LINE

        mission_start_ = nh_.serviceClient<krpc_ib2_msgs::MissionStart>("/event/mission_start");
        mission_finished_ = nh_.serviceClient<krpc_ib2_msgs::MissionFinished>("/event/mission_finished");
        checkpoints_ = nh_.serviceClient<krpc_ib2_msgs::CheckPoints>("/mission/get_checkpoints");
        ros::param::set("/ib2_user/ready", true); // DO NOT REMOVE THIS LINE
    }

    void run() {
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber start_sub_;
    ros::ServiceServer stop_service_;
    actionlib::SimpleActionClient<ib2_msgs::CtlCommandAction> command_client_;
    actionlib::SimpleActionClient<krpc_ib2_msgs::CheckpointPassedAction> checkpoint_client_;
    ros::ServiceClient mission_start_, mission_finished_, checkpoints_;
    bool action_active_;
    bool started_;

    bool notifyMissionStart() {
        mission_start_.waitForExistence();
        krpc_ib2_msgs::MissionStart service;
        if (!mission_start_.call(service)) return false;
        ROS_INFO("mission_start result: success=%s message=%s", service.response.success ? "true" : "false", service.response.message.c_str());
        return service.response.success;
    }

    bool notifyMissionFinished() {
        mission_finished_.waitForExistence();
        krpc_ib2_msgs::MissionFinished service;
        if (!mission_finished_.call(service)) return false;
        ROS_INFO("mission_finished result: success=%s message=%s", service.response.success ? "true" : "false", service.response.message.c_str());
        return service.response.success;
    }

    bool resolveFirstCheckpoint(CheckpointTarget& target) {
        target.checkpoint_id = 1;
        target.pose.pose.orientation.w = 1.0;
        checkpoints_.waitForExistence();
        krpc_ib2_msgs::CheckPoints service;
        if (!checkpoints_.call(service) || !service.response.success) return true;

        std::string payload = service.response.message;
        std::replace(payload.begin(), payload.end(), '\n', ' ');
        const std::regex checkpoint_regex("\\\"checkpoint_id\\\"\\s*:\\s*(-?[0-9]+).*?\\\"position\\\"\\s*:\\s*\\[\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)");
        std::sregex_iterator it(payload.begin(), payload.end(), checkpoint_regex);
        std::sregex_iterator end;
        bool found = false;
        for (; it != end; ++it) {
            int id = std::stoi((*it)[1]);
            if (found && id >= target.checkpoint_id) continue;
            std::sregex_iterator next = it;
            ++next;
            const std::size_t checkpoint_start = static_cast<std::size_t>(it->position());
            const std::size_t checkpoint_end = next == end
                ? payload.size()
                : static_cast<std::size_t>(next->position());
            target.checkpoint_id = id;
            target.pose.pose.position.x = std::stod((*it)[2]);
            target.pose.pose.position.y = std::stod((*it)[3]);
            target.pose.pose.position.z = std::stod((*it)[4]);
            target.pose.pose.orientation.x = 0.0;
            target.pose.pose.orientation.y = 0.0;
            target.pose.pose.orientation.z = 0.0;
            target.pose.pose.orientation.w = 1.0;
            const std::string checkpoint_json = payload.substr(
                checkpoint_start, checkpoint_end - checkpoint_start);
            const std::regex quaternion_regex("\\\"quaternion\\\"\\s*:\\s*\\[\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)");
            std::smatch quaternion_match;
            if (std::regex_search(checkpoint_json, quaternion_match, quaternion_regex)) {
                target.pose.pose.orientation.x = std::stod(quaternion_match[1]);
                target.pose.pose.orientation.y = std::stod(quaternion_match[2]);
                target.pose.pose.orientation.z = std::stod(quaternion_match[3]);
                target.pose.pose.orientation.w = std::stod(quaternion_match[4]);
            }
            found = true;
        }
        if (found) ROS_INFO("selected checkpoint_id=%d", target.checkpoint_id);
        return true;
    }

    bool moveToCheckpoint(const CheckpointTarget& target) {
        command_client_.waitForServer();
        ib2_msgs::CtlCommandGoal goal;
        goal.target = target.pose;
        goal.type.type = ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET;
        action_active_ = true;
        command_client_.sendGoal(goal);
        while (!ros::isShuttingDown()) {
            if (command_client_.waitForResult(ros::Duration(0.2))) {
                action_active_ = false;
                return command_client_.getResult()->type == ib2_msgs::CtlCommandResult::TERMINATE_SUCCESS;
            }
        }
        action_active_ = false;
        return false;
    }

    bool notifyCheckpointPassed(int checkpoint_id) {
        checkpoint_client_.waitForServer();
        krpc_ib2_msgs::CheckpointPassedGoal goal;
        goal.checkpoint_id = checkpoint_id;
        checkpoint_client_.sendGoal(goal);
        while (!ros::isShuttingDown()) {
            if (checkpoint_client_.waitForResult(ros::Duration(0.2)))
                return checkpoint_client_.getResult()->success;
        }
        return false;
    }

    void startCallback(const platform_msgs::UserLogic::ConstPtr& msg) {
        if (started_) return;
        started_ = true;
        ROS_INFO_STREAM("start " << *msg);
        if (!notifyMissionStart()) return;
        CheckpointTarget target;
        if (!resolveFirstCheckpoint(target) || !moveToCheckpoint(target)) return;
        if (!notifyCheckpointPassed(target.checkpoint_id)) return;
        if (!ros::isShuttingDown()) notifyMissionFinished();
    }

    bool stopProcessing(platform_msgs::StopProcessingUserNode::Request& req,
                       platform_msgs::StopProcessingUserNode::Response& res) {
        if (action_active_) command_client_.cancelGoal();
        if (checkpoint_client_.isServerConnected()) checkpoint_client_.cancelGoal();
        ROS_INFO("finish processing");
        res.result = platform_msgs::StopProcessingUserNodeResponse::SUCCESS;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "krpc_user_cpp");
    ros::NodeHandle nh("~");
    UserTemplate user(nh);
    user.run();
    return 0;
}
