#!/usr/bin/python3
# -*- coding:utf-8 -*-
import json
import rospy
import actionlib
from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType

# Optional IF examples. Uncomment only what your program uses.
# from std_msgs.msg import Time
# from platform_msgs.msg import UserNodeStatus
# from sensor_msgs.msg import Image
# from ib2_msgs.msg import Navigation


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action client
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service client
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE

        # Optional subscriber examples.
        # /sensor_fusion/navigation provides the current Int-Ball2 pose, velocity and acceleration.
        # self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)
        # /camera_main/image_raw provides the main camera image.
        # self.__subscriber_camera = rospy.Subscriber('/camera_main/image_raw', Image, self.__callback_camera_image)

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing) # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        rospy.set_param('/ib2_user/ready', True) # DO NOT REMOVE THIS LINE

    def __notify_mission_start(self):
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
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        self.__checkpoint_client.wait_for_server()
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_client.send_goal(goal)
        while not rospy.is_shutdown():
            if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result.success
        if rospy.is_shutdown():
            self.__checkpoint_client.cancel_goal()
            rospy.logwarn('checkpoint_passed action cancelled by node shutdown')
        return False

    def __get_checkpoint_id(self, checkpoint):
        for key in ('checkpoint_id', 'checkpointid', 'id'):
            if key in checkpoint:
                return int(checkpoint[key])
        return None

    def __resolve_first_checkpoint_target(self):
        default_target = {
            'checkpoint_id': 1,
            'position': [0.0, 0.0, 0.0],
            'quaternion': [0.0, 0.0, 0.0, 1.0],
        }
        rospy.wait_for_service('/mission/get_checkpoints')
        res = self.__checkpoints()
        rospy.loginfo('get_checkpoints result: success=%s message=%s', res.success, res.message)
        if not res.success:
            return default_target

        try:
            payload = json.loads(res.message)
            checkpoints = payload.get('checkpoints', payload.get('Checkpoints', []))
            checkpoints = [cp for cp in checkpoints if isinstance(cp, dict) and self.__get_checkpoint_id(cp) is not None]
            if not checkpoints:
                rospy.logwarn('No checkpoints found in payload')
                return default_target

            checkpoint = min(checkpoints, key=self.__get_checkpoint_id)
            position = checkpoint.get('position', default_target['position'])

            if not (isinstance(position, list) and len(position) >= 3):
                rospy.logwarn('Invalid checkpoint position format')
                return default_target

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
            rospy.logwarn('Failed to parse checkpoints payload: %s', e)

        return default_target

    def __move_to_checkpoint(self, checkpoint_target):
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            while not rospy.is_shutdown():
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
            if rospy.is_shutdown():
                self.__command.cancel_goal()
                rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            self.__action_active = False

    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return

            # USER EDIT AREA: define route planning and game-specific flow here.
            # ***********************************************
            checkpoint_target = self.__resolve_first_checkpoint_target()
            if not self.__move_to_checkpoint(checkpoint_target):
                return
            if not self.__notify_checkpoint_passed(checkpoint_target['checkpoint_id']):
                return
            # ***********************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    # Optional subscriber callback examples.
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
        try:
            if self.__action_active:
                self.__command.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
