#!/usr/bin/python3
# -*- coding:utf-8 -*-
#
# =============================================================================
# ANNOTATED VERSION of krpc_user/scripts/user_template.py
# (Kibo-RPC sample workspace — engr.greyhawks)
#
# This file is a commented walkthrough of the original template. Logic is
# UNCHANGED from the original — only explanatory comments have been added.
# Use this to understand the structure before you start editing the real
# user_template.py in your catkin_ws.
#
# Annotated through claude
# =============================================================================

import json
import rospy          # Core ROS Python client library: nodes, topics, params, logging
import actionlib       # ROS "actions": long-running goal/feedback/result interactions
                        # (as opposed to services, which are simple one-shot request/response)

# --- Message/service/action types this node consumes -----------------------
# platform_msgs: generic types defined by the Kibo-RPC platform (not your package)
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse

# geometry_msgs: standard ROS type for a 3D position + orientation with a timestamp/frame
from geometry_msgs.msg import PoseStamped

# krpc_ib2_msgs: the custom types defined in YOUR krpc_ib2_msgs package
# (see CheckPoints.srv, MissionStart.srv, MissionFinished.srv, CheckpointPassed.action)
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal

# ib2_msgs: types for controlling the Int-Ball2 robot itself
# CtlCommandAction = the action definition (goal/result/feedback bundle)
# CtlCommandGoal   = what you send to request a move
# CtlCommandResult = what you get back (success/fail code)
# CtlStatusType    = enum-like type describing WHAT KIND of command you're sending
#                    (e.g. MOVE_TO_ABSOLUTE_TARGET vs. other motion modes)
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType

# Optional IF examples. Uncomment only what your program uses.
# from std_msgs.msg import Time
# from platform_msgs.msg import UserNodeStatus
# from sensor_msgs.msg import Image
# from ib2_msgs.msg import Navigation


class UserTemplate:

    def __init__(self):
        # Registers this process as a ROS node named 'krpc_user'. Must be called
        # once per process, before any other rospy calls.
        rospy.init_node('krpc_user')

        # ---------------------------------------------------------------
        # ACTION CLIENTS
        # An "action client" lets this node SEND GOALS to a server elsewhere
        # and track long-running progress (unlike a plain service call, which
        # blocks until one instant response comes back).
        # ---------------------------------------------------------------

        # Talks to the robot's low-level motion controller ('/ctl/command').
        # This is provided by the PLATFORM (not krpc_api_server) — it's the
        # real (or simulated) flight-control stack that actually moves Int-Ball2.
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)

        # Talks to krpc_api_node's checkpoint-passed action server — this is
        # how you formally report "I reached checkpoint N" for scoring.
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # ---------------------------------------------------------------
        # SERVICE CLIENTS
        # A "service client" makes simple, synchronous request → response
        # calls (like a function call over the network). Used here for
        # bookkeeping/event calls that complete instantly, as opposed to
        # actions which represent something that takes real time (like moving).
        # ---------------------------------------------------------------
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # ---------------------------------------------------------------
        # SUBSCRIBER — the competition start signal.
        # The platform publishes to /ib2_user/start when the mission timer
        # begins; that's what actually kicks off __callback_start below.
        # DO NOT REMOVE: without this subscriber, your node never learns
        # that the mission has started.
        # ---------------------------------------------------------------
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # Optional subscriber examples — commented out in the template.
        # Uncomment + implement the matching callback if your mission logic
        # needs live telemetry (position/velocity) or camera frames.
        # /sensor_fusion/navigation provides the current Int-Ball2 pose, velocity and acceleration.
        # self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)
        # /camera_main/image_raw provides the main camera image.
        # self.__subscriber_camera = rospy.Subscriber('/camera_main/image_raw', Image, self.__callback_camera_image)

        # ---------------------------------------------------------------
        # SERVICE SERVER — lets the platform stop your node mid-mission
        # (e.g. time runs out, or the operator aborts the run). When called,
        # __stop_processing (below) cancels any in-flight move.
        # DO NOT REMOVE: the platform relies on this existing to shut you
        # down cleanly.
        # ---------------------------------------------------------------
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        # ---------------------------------------------------------------
        # INTERNAL STATE
        # __started:       guards against __callback_start firing twice
        # __action_active: tracks whether a /ctl/command goal is currently
        #                  in flight, so __stop_processing knows whether
        #                  there's anything to cancel
        # ---------------------------------------------------------------
        self.__started = False
        self.__action_active = False

        # Tells the platform "my node has finished initializing and is ready
        # to receive the start signal". DO NOT REMOVE — if this never gets
        # set, the platform may never send /ib2_user/start.
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE

    # =====================================================================
    # MISSION LIFECYCLE HELPERS
    # These three methods just wrap the three "bookkeeping" calls every
    # mission needs: announce start, report a checkpoint, announce finish.
    # =====================================================================

    def __notify_mission_start(self):
        # Blocks until the /event/mission_start service actually exists
        # (i.e. krpc_api_node has finished starting up) before calling it.
        rospy.wait_for_service('/event/mission_start')
        res = self.__mission_start()
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        rospy.wait_for_service('/event/mission_finished')
        res = self.__mission_finished()
        rospy.loginfo('mission_finished result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_checkpoint_passed(self, checkpoint_id):
        # Unlike mission_start/finished (services), this is an ACTION call,
        # because in the real competition confirming a checkpoint might
        # involve the backend doing real work (e.g. image verification)
        # rather than an instant reply.
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        self.__checkpoint_client.wait_for_server()
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_client.send_goal(goal)

        # Poll every 0.2s for a result instead of a single blocking wait,
        # so the loop can bail out cleanly if ROS starts shutting down
        # mid-wait (e.g. mission time expired).
        while not rospy.is_shutdown():
            if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result.success
        if rospy.is_shutdown():
            self.__checkpoint_client.cancel_goal()
            rospy.logwarn('checkpoint_passed action cancelled by node shutdown')
        return False

    # =====================================================================
    # CHECKPOINT PARSING
    # get_checkpoints returns a raw JSON *string* (not a structured ROS
    # message) inside res.message, because the checkpoint schema can vary
    # (field naming, optional attitude data). These two helpers parse and
    # pick a target out of that JSON defensively.
    # =====================================================================

    def __get_checkpoint_id(self, checkpoint):
        # The cheat_sheet/JSON payload may use different key names for the
        # same concept depending on how it was generated — check all of them.
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in checkpoint:
                return int(checkpoint[key])
        return None

    def __resolve_first_checkpoint_target(self):
        # Fallback target used whenever anything goes wrong parsing real
        # checkpoint data — keeps the node from crashing outright.
        default_target = {
            'checkpoint_id': 1,
            'position': [0.0, 0.0, 0.0],
            'quaternion': [0.0, 0.0, 0.0, 1.0],  # identity rotation (no rotation)
        }

        rospy.wait_for_service('/mission/get_checkpoints')
        res = self.__checkpoints()
        rospy.loginfo('get_checkpoints result: success=%s message=%s', res.success, res.message)
        if not res.success:
            return default_target

        try:
            payload = json.loads(res.message)
            # Accept either casing/key for the checkpoint list.
            checkpoints = payload.get('checkpoints', payload.get('Checkpoints', []))
            # Keep only entries that are dict-shaped AND have a resolvable ID.
            checkpoints = [cp for cp in checkpoints if isinstance(cp, dict) and self.__get_checkpoint_id(cp) is not None]
            if not checkpoints:
                rospy.logwarn('No checkpoints found in payload')
                return default_target

            # THIS TEMPLATE'S ENTIRE "STRATEGY": always go to the
            # lowest-numbered checkpoint. Real mission logic would instead
            # plan a route across ALL checkpoints (see USER EDIT AREA below).
            checkpoint = min(checkpoints, key=self.__get_checkpoint_id)
            position = checkpoint.get('position', default_target['position'])

            if not (isinstance(position, list) and len(position) >= 3):
                rospy.logwarn('Invalid checkpoint position format')
                return default_target

            # Orientation ("attitude") is optional in the payload — fall back
            # to the identity quaternion if it's missing or malformed.
            quaternion = default_target['quaternion']
            attitude = checkpoint.get('attitude')
            if isinstance(attitude, dict):
                attitude_quaternion = attitude.get('quaternion')
                if isinstance(attitude_quaternion, list) and len(attitude_quaternion) >= 4:
                    quaternion = attitude_quaternion

            return {
                'checkpoint_id': self.__get_checkpoint_id(checkpoint),
                'position': [float(position[0]), float(position[1]), float(position[2])],
                'quaternion': [float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3])],
            }
        except Exception as e:
            # Any parsing failure (bad JSON, wrong types, etc.) falls back
            # to the safe default rather than propagating an exception.
            rospy.logwarn('Failed to parse checkpoints payload: %s', e)

        return default_target

    # =====================================================================
    # MOTION
    # =====================================================================

    def __move_to_checkpoint(self, checkpoint_target):
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        # Build the ROS pose message the controller expects: 3D position +
        # orientation as a quaternion (x, y, z, w). PoseStamped also carries
        # a header (timestamp/frame_id) — left at defaults here.
        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        # CtlStatusType.MOVE_TO_ABSOLUTE_TARGET tells the controller
        # "fly to this exact pose in the world frame", as opposed to other
        # possible modes (relative movement, velocity control, etc.).
        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)

        self.__action_active = True   # so __stop_processing knows to cancel this if asked
        try:
            self.__command.send_goal(goal)
            # Same poll-every-0.2s pattern as __notify_checkpoint_passed,
            # so a shutdown mid-move can be handled gracefully.
            while not rospy.is_shutdown():
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    # TERMINATE_SUCCESS is the "the robot arrived / completed
                    # cleanly" result code; anything else counts as failure.
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
            if rospy.is_shutdown():
                self.__command.cancel_goal()
                rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            # Always clear the flag, whether the move succeeded, failed,
            # or was cancelled.
            self.__action_active = False

    # =====================================================================
    # MISSION FLOW — THIS IS WHAT YOU EDIT
    # =====================================================================

    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return  # backend rejected/failed the start notification — abort

            # ***********************************************
            # USER EDIT AREA
            # Replace this block with your real mission logic: e.g. loop
            # over ALL checkpoints in order (not just the first/lowest one),
            # run vision/scanning tasks at each stop, handle failures with
            # retries, etc. As written, the template only ever visits ONE
            # checkpoint before finishing.
            checkpoint_target = self.__resolve_first_checkpoint_target()
            if not self.__move_to_checkpoint(checkpoint_target):
                return
            if not self.__notify_checkpoint_passed(checkpoint_target['checkpoint_id']):
                return
            # ***********************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            # Catch-all so an unhandled exception in your mission logic
            # doesn't silently kill the node with no log trace.
            rospy.logerr('user flow failed: %s', exc)

    # =====================================================================
    # CALLBACKS
    # =====================================================================

    def __callback_start(self, msg):
        # Guard against the /ib2_user/start message being (re)published more
        # than once — the mission should only ever run through once.
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    # Optional subscriber callback examples (paired with the commented-out
    # subscribers above __init__). Uncomment both together if you need them.
    # def __callback_nav(self, data):
    #     position = data.pose.pose.position
    #     orientation = data.pose.pose.orientation
    #     rospy.loginfo('current pose: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
    #                   position.x, position.y, position.z,
    #                   orientation.x, orientation.y, orientation.z, orientation.w)
    #
    # def __callback_camera_image(self, data):
    #     rospy.loginfo('camera image: width=%d height=%d encoding=%s',
    #                   data.width, data.height, data.encoding)

    def __stop_processing(self, request):
        # Called by the platform (via the /ib2_user/stop service) to force
        # an in-progress run to halt — e.g. time limit reached.
        try:
            if self.__action_active:
                self.__command.cancel_goal()  # abort any in-flight move
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        # Hands control to ROS's event loop: blocks here, dispatching
        # subscriber/service/action callbacks as messages arrive, until
        # the node is shut down.
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
