// =============================================================================
// ANNOTATED VERSION of krpc_user/src/user_template_node.cpp
// (Kibo-RPC sample workspace — engr.greyhawks)
//
// This is a commented walkthrough of the original template. Logic is
// UNCHANGED — only explanatory comments have been added. Compare directly
// against user_template_annotated.py: both implement the exact same node
// (same topics/services/actions), just in different languages.
// =============================================================================

#include <ros/ros.h>                                  // Core ROS C++ client library (rospy's C++ equivalent)
#include <actionlib/client/simple_action_client.h>     // ROS "actions": long-running goal/feedback/result calls

// platform_msgs: fixed contracts defined by the Kibo-RPC platform — do not modify.
#include <platform_msgs/UserLogic.h>
#include <platform_msgs/StopProcessingUserNode.h>

// geometry_msgs: standard ROS type for a 3D position + orientation with a timestamp/frame.
#include <geometry_msgs/PoseStamped.h>

// ib2_msgs: types for controlling the Int-Ball2 robot itself.
#include <ib2_msgs/CtlCommandAction.h>   // The action definition (goal/result/feedback bundle)
#include <ib2_msgs/CtlCommandGoal.h>     // What you send to request a move
#include <ib2_msgs/CtlStatusType.h>      // Enum describing WHAT KIND of command you're sending
// Note: CtlCommandResult (used below for TERMINATE_SUCCESS) comes bundled
// inside CtlCommandAction.h's generated headers — no separate include needed.

// krpc_ib2_msgs: your own package's custom message/service/action types
// (see CheckPoints.srv, MissionStart.srv, MissionFinished.srv, CheckpointPassed.action).
#include <krpc_ib2_msgs/CheckPoints.h>
#include <krpc_ib2_msgs/MissionFinished.h>
#include <krpc_ib2_msgs/MissionStart.h>
#include <krpc_ib2_msgs/CheckpointPassedAction.h>
#include <krpc_ib2_msgs/CheckpointPassedGoal.h>

#include <algorithm>   // std::replace
#include <regex>       // std::regex — used to hand-parse the checkpoint JSON string
#include <string>

// -----------------------------------------------------------------------
// A small plain struct to carry "where to go next": which checkpoint ID,
// and the full pose (position + orientation) to send to the controller.
// This is the C++ equivalent of the Python version's loose dict
// {'checkpoint_id': ..., 'position': ..., 'quaternion': ...}.
// -----------------------------------------------------------------------
struct CheckpointTarget {
    int checkpoint_id;
    geometry_msgs::PoseStamped pose;
};

class UserTemplate {
public:
    // ---------------------------------------------------------------
    // CONSTRUCTOR — C++ equivalent of Python's __init__.
    // Member-initializer list sets up:
    //   - nh_: the NodeHandle, ROS's handle for creating pubs/subs/services
    //   - command_client_ / checkpoint_client_: action clients, constructed
    //     with `true` as the second arg meaning "auto-spin a thread for me"
    //     (spin_thread=true) so results can be polled without manually
    //     pumping callbacks for these two clients specifically
    //   - action_active_ / started_: same guard flags as the Python version
    // ---------------------------------------------------------------
    UserTemplate(ros::NodeHandle& nh) :
        nh_(nh),
        command_client_("/ctl/command", true),
        checkpoint_client_("/event/checkpoint_passed", true),
        action_active_(false),
        started_(false)
    {
        // SUBSCRIBER — the competition start signal. Queue size 1 means only
        // the latest message is buffered if callbacks can't keep up.
        // DO NOT REMOVE: without this, the node never learns the mission started.
        start_sub_ = nh_.subscribe("/ib2_user/start", 1, &UserTemplate::startCallback, this); // DO NOT REMOVE THIS LINE

        // SERVICE SERVER — lets the platform forcibly stop your node.
        // DO NOT REMOVE: the platform relies on this existing to shut you down cleanly.
        stop_service_ = nh_.advertiseService("/ib2_user/stop", &UserTemplate::stopProcessing, this); // DO NOT REMOVE THIS LINE

        // SERVICE CLIENTS — simple synchronous request/response calls,
        // templated on the exact service type each one will call.
        mission_start_ = nh_.serviceClient<krpc_ib2_msgs::MissionStart>("/event/mission_start");
        mission_finished_ = nh_.serviceClient<krpc_ib2_msgs::MissionFinished>("/event/mission_finished");
        checkpoints_ = nh_.serviceClient<krpc_ib2_msgs::CheckPoints>("/mission/get_checkpoints");

        // Tells the platform "my node has finished initializing and is ready
        // to receive the start signal". DO NOT REMOVE.
        ros::param::set("/ib2_user/ready", true); // DO NOT REMOVE THIS LINE
    }

    // ---------------------------------------------------------------
    // run() — C++'s version of rospy.spin(), but using an AsyncSpinner
    // with 2 threads instead of a single-threaded spin. This matters
    // because the action clients above were constructed with their own
    // auto-spinning thread (spin_thread=true) — using a multi-threaded
    // spinner here avoids callback contention between the subscriber/
    // service callbacks and the action clients' internal callback queues.
    // waitForShutdown() blocks until ROS is told to shut down (Ctrl+C,
    // or the platform killing the node).
    // ---------------------------------------------------------------
    void run() {
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    }

private:
    // ---------------------------------------------------------------
    // MEMBER VARIABLES — C++ requires these to be declared explicitly
    // (unlike Python where self.__whatever just gets created on assignment).
    // Note everything here is private: this class exposes only the
    // constructor and run() as its public interface.
    // ---------------------------------------------------------------
    ros::NodeHandle nh_;
    ros::Subscriber start_sub_;
    ros::ServiceServer stop_service_;
    actionlib::SimpleActionClient<ib2_msgs::CtlCommandAction> command_client_;
    actionlib::SimpleActionClient<krpc_ib2_msgs::CheckpointPassedAction> checkpoint_client_;
    ros::ServiceClient mission_start_, mission_finished_, checkpoints_;
    bool action_active_;
    bool started_;

    // =================================================================
    // MISSION LIFECYCLE HELPERS
    // Same role as the Python template's __notify_mission_start/finished:
    // wait for the service to exist, call it, log the result, return
    // whether it succeeded.
    // =================================================================

    bool notifyMissionStart() {
        mission_start_.waitForExistence();               // blocks until krpc_api_node is up
        krpc_ib2_msgs::MissionStart service;              // empty request/response object
        if (!mission_start_.call(service)) return false;  // false here means the CALL ITSELF failed
                                                            // (e.g. server crashed) — different from
                                                            // the server replying success=false
        ROS_INFO("mission_start result: success=%s message=%s",
                 service.response.success ? "true" : "false",
                 service.response.message.c_str());
        return service.response.success;
    }

    bool notifyMissionFinished() {
        mission_finished_.waitForExistence();
        krpc_ib2_msgs::MissionFinished service;
        if (!mission_finished_.call(service)) return false;
        ROS_INFO("mission_finished result: success=%s message=%s",
                 service.response.success ? "true" : "false",
                 service.response.message.c_str());
        return service.response.success;
    }

    // =================================================================
    // CHECKPOINT PARSING — the trickiest part of this file, and the main
    // place the C++ version differs meaningfully from the Python one.
    //
    // The Python version can just call json.loads() on the payload string.
    // C++ has no JSON library declared as a dependency in CMakeLists.txt,
    // so this hand-rolls the parsing with regular expressions instead —
    // more fragile (a payload format change breaks the regex silently),
    // but avoids adding an extra build dependency.
    //
    // Returns the target BY REFERENCE (out-parameter `target`) rather than
    // returning it directly — a common C++ pattern, versus Python simply
    // returning a dict.
    // =================================================================

    bool resolveFirstCheckpoint(CheckpointTarget& target) {
        // Safe defaults, applied up front — mirrors the Python version's
        // default_target dict.
        target.checkpoint_id = 1;
        target.pose.pose.orientation.w = 1.0;  // identity quaternion (no rotation)

        checkpoints_.waitForExistence();
        krpc_ib2_msgs::CheckPoints service;
        // If the call fails outright OR the server reports success=false,
        // bail out early and just use the defaults set above.
        if (!checkpoints_.call(service) || !service.response.success) return true;

        std::string payload = service.response.message;
        // Flatten newlines to spaces first so the regex can match across
        // what would otherwise be line breaks in pretty-printed JSON.
        std::replace(payload.begin(), payload.end(), '\n', ' ');

        // This regex looks for a "checkpoint_id": <int> followed (later in
        // the string, non-greedily) by a "position": [x, y, z] array, and
        // captures the id and the three position numbers.
        // NOTE: this assumes "checkpoint_id" always appears BEFORE
        // "position" in the JSON for a given object — a real JSON parser
        // wouldn't care about key order, but this regex does.
        const std::regex checkpoint_regex(
            "\\\"checkpoint_id\\\"\\s*:\\s*(-?[0-9]+).*?"
            "\\\"position\\\"\\s*:\\s*\\[\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)"
        );
        std::sregex_iterator it(payload.begin(), payload.end(), checkpoint_regex);
        std::sregex_iterator end;
        bool found = false;

        // Iterate over every regex match in the payload (i.e. every
        // checkpoint object found), keeping whichever has the LOWEST id —
        // same "always pick the lowest checkpoint" strategy as the Python
        // template's min(checkpoints, key=...).
        for (; it != end; ++it) {
            int id = std::stoi((*it)[1]);
            // Skip this match if we already found a lower (or equal) id.
            if (found && id >= target.checkpoint_id) continue;

            // To find this checkpoint's optional "quaternion" field, we need
            // to isolate just the JSON substring belonging to THIS
            // checkpoint object — approximated here as "from this match's
            // start position to the start of the NEXT match" (or end of
            // string if this is the last one). This is a rough approximation
            // of object boundaries since we're not actually parsing JSON
            // structure, just text positions.
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
            // Default to identity rotation; overwritten below if a
            // "quaternion" field is actually found for this checkpoint.
            target.pose.pose.orientation.x = 0.0;
            target.pose.pose.orientation.y = 0.0;
            target.pose.pose.orientation.z = 0.0;
            target.pose.pose.orientation.w = 1.0;

            // Slice out just this checkpoint's chunk of the payload, then
            // search THAT smaller string for an optional quaternion array —
            // keeps the quaternion regex from accidentally matching a
            // different checkpoint's orientation data.
            const std::string checkpoint_json = payload.substr(
                checkpoint_start, checkpoint_end - checkpoint_start);
            const std::regex quaternion_regex(
                "\\\"quaternion\\\"\\s*:\\s*\\[\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)\\s*,\\s*(-?[0-9.eE+]+)"
            );
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
        // Always returns true (unlike the Python version's more explicit
        // fallback branches) — "true" here just means "the service call
        // itself completed", not "we necessarily found real checkpoint
        // data"; if nothing was found, target still holds the safe
        // defaults set at the top of the function.
        return true;
    }

    // =================================================================
    // MOTION — same MOVE_TO_ABSOLUTE_TARGET pattern as the Python version.
    // =================================================================

    bool moveToCheckpoint(const CheckpointTarget& target) {
        command_client_.waitForServer();

        ib2_msgs::CtlCommandGoal goal;
        goal.target = target.pose;
        // MOVE_TO_ABSOLUTE_TARGET = "fly to this exact pose in the world
        // frame", same command type constant as the Python template.
        goal.type.type = ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET;

        action_active_ = true;  // lets stopProcessing() know there's something to cancel
        command_client_.sendGoal(goal);

        // Same poll-every-0.2s pattern as the Python version, so a
        // shutdown mid-move can be noticed and handled instead of blocking
        // forever.
        while (!ros::isShuttingDown()) {
            if (command_client_.waitForResult(ros::Duration(0.2))) {
                action_active_ = false;
                // TERMINATE_SUCCESS = "the robot arrived / completed cleanly";
                // anything else counts as failure.
                return command_client_.getResult()->type == ib2_msgs::CtlCommandResult::TERMINATE_SUCCESS;
            }
        }
        action_active_ = false;
        return false;  // loop exited because ROS is shutting down, not because we succeeded
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

    // =================================================================
    // startCallback — this is the mission flow entry point, equivalent to
    // the Python template's __callback_start + __run_user_flow COMBINED
    // into one method (the C++ version doesn't split them into two).
    //
    // *** THIS IS WHAT YOU EDIT *** to implement real mission logic —
    // the C++ template has no explicit "USER EDIT AREA" comment markers
    // like the Python one does, but the equivalent section is the four
    // lines from resolveFirstCheckpoint(...) through notifyCheckpointPassed(...).
    // =================================================================

    void startCallback(const platform_msgs::UserLogic::ConstPtr& msg) {
        // Guard against /ib2_user/start firing more than once.
        if (started_) return;
        started_ = true;

        ROS_INFO_STREAM("start " << *msg);

        if (!notifyMissionStart()) return;

        // *** USER EDIT AREA (implicit) ***
        // As shipped: resolve ONE target (lowest checkpoint id), move
        // there, report it passed. Replace with real multi-checkpoint
        // navigation / scoring logic.
        CheckpointTarget target;
        if (!resolveFirstCheckpoint(target) || !moveToCheckpoint(target)) return;
        if (!notifyCheckpointPassed(target.checkpoint_id)) return;
        // *** end USER EDIT AREA ***

        if (!ros::isShuttingDown()) notifyMissionFinished();
    }

    // ---------------------------------------------------------------
    // stopProcessing — service callback for /ib2_user/stop, C++'s
    // equivalent of the Python template's __stop_processing. Unlike the
    // Python version (which only cancels the move command), this also
    // explicitly cancels the checkpoint-passed action goal if that
    // client is still connected — slightly more thorough cleanup.
    // ---------------------------------------------------------------
    bool stopProcessing(platform_msgs::StopProcessingUserNode::Request& req,
                       platform_msgs::StopProcessingUserNode::Response& res) {
        if (action_active_) command_client_.cancelGoal();
        if (checkpoint_client_.isServerConnected()) checkpoint_client_.cancelGoal();
        ROS_INFO("finish processing");
        res.result = platform_msgs::StopProcessingUserNodeResponse::SUCCESS;
        return true;  // return value here means "service handled successfully",
                       // separate from res.result which is the actual outcome code
    }
};

// =====================================================================
// main() — C++ requires an explicit entry point (unlike Python's
// `if __name__ == '__main__':`).
//   - ros::init registers the node; "krpc_user_cpp" is its default name
//     (though the launch file can override the node NAME via remapping —
//     it's the executable TYPE that team_user_script actually selects).
//   - ros::NodeHandle nh("~") creates a PRIVATE node handle (the "~"
//     prefix), meaning any topics/params this handle creates would be
//     namespaced under the node's own name — note this handle (nh) is
//     passed into UserTemplate, but the class itself calls
//     nh_.subscribe/advertiseService/serviceClient with plain (non-"~")
//     names, so those still resolve to the exact global names shown
//     (e.g. "/ib2_user/start") rather than being private-namespaced.
// =====================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "krpc_user_cpp");
    ros::NodeHandle nh("~");
    UserTemplate user(nh);
    user.run();
    return 0;
}
