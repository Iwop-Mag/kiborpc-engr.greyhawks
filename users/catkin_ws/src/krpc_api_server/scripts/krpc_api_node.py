#!/usr/bin/python3
# -*- coding:utf-8 -*-

import json
import rospy
import actionlib
import os
from std_msgs.msg import String
from cheat_sheet_loader import CheatSheetLoader

JSON_STATUS = "status"
JSON_STATUS_START = "Mission Start"

# Service
from krpc_ib2_msgs.srv import (
    CheckPoints,
    MissionStart,
    MissionFinished,
)

# Action
from krpc_ib2_msgs.msg import (
    CheckpointPassedAction,
    CheckpointPassedResult,
    CheckpointPassedFeedback,
)


class KrpcApiNode:
    def __init__(self):
        rospy.init_node("krpc_api_node")

        fallback_cheat_sheet_path = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "../cheat_sheet.json"
            )
        )

        # Primary: read from ROS parameter or latest symlink
        checkpoint_source_param = "~checkpoint_source_path"
        if rospy.has_param(checkpoint_source_param):
            configured_path = rospy.get_param(checkpoint_source_param)
            if not isinstance(configured_path, str) or not configured_path.strip():
                raise RuntimeError(
                    "~checkpoint_source_path is set but empty. "
                    "Specify a valid cheat_sheet path."
                )
            primary_path = os.path.abspath(
                os.path.expanduser(configured_path.strip())
            )
        else:
            primary_path = None

        # Determine which path to use: primary → fallback
        if primary_path and os.path.exists(primary_path):
            self._cheat_sheet_path = primary_path
            source_label = "parameter"
        elif os.path.exists(fallback_cheat_sheet_path):
            self._cheat_sheet_path = fallback_cheat_sheet_path
            source_label = "fallback"
            if primary_path:
                rospy.logwarn(
                    f"Primary cheat_sheet path not found: {primary_path}. "
                    f"Using fallback: {fallback_cheat_sheet_path}"
                )
        else:
            raise FileNotFoundError(
                f"cheat_sheet.json not found. "
                f"Tried primary: {primary_path} and fallback: {fallback_cheat_sheet_path}"
            )

        rospy.loginfo(f"checkpoint source path ({source_label}): {self._cheat_sheet_path}")
        self._cheat_sheet = CheatSheetLoader.from_path(self._cheat_sheet_path)

        # Topic
        self._gs_data_pub = rospy.Publisher("/gs/data", String, queue_size=10)

        # Services
        rospy.Service("/mission/get_checkpoints", CheckPoints, self.get_checkpoints)
        rospy.Service("/event/mission_start", MissionStart, self.mission_start)
        rospy.Service("/event/mission_finished", MissionFinished, self.mission_finished)

        # Action
        self._checkpoint_server = actionlib.SimpleActionServer(
            "/event/checkpoint_passed",
            CheckpointPassedAction,
            execute_cb=self.checkpoint_passed,
            auto_start=False
        )
        self._checkpoint_server.start()

        rospy.loginfo("KrpcApiNode initialized (sim mode)")

    # ==========================================================
    # 共通: 採点モジュール通知
    # ==========================================================

    # 受信イベントを /gs/data に転送する（sim専用）。
    def publish_event(self, payload):
        payload = dict(payload)
        msg = json.dumps(payload, ensure_ascii=False)
        self._gs_data_pub.publish(String(data=msg))
        rospy.loginfo(f"/gs/data: {msg}")
        return True, "accepted"

    # ==========================================================
    # Service
    # ==========================================================
    # チェックポイント一覧情報取得
    def get_checkpoints(self, req):
        rospy.loginfo("get_checkpoints")
        res = CheckPoints._response_class()
        try:
            payload = self._build_checkpoint_payload_from_cheat_sheet()
            if hasattr(res, "success"):
                res.success = True
            if hasattr(res, "message"):
                res.message = json.dumps(payload, ensure_ascii=False)
        except Exception as e:
            rospy.logerr(f"get_checkpoints failed: {e}")
            if hasattr(res, "success"):
                res.success = False
            if hasattr(res, "message"):
                res.message = json.dumps({
                    "error": "checkpoint_load_failed",
                    "detail": str(e)
                }, ensure_ascii=False)

        return res

    # cheat_sheetのチェックポイント定義をサービス応答用フォーマットへ変換する。
    def _build_checkpoint_payload_from_cheat_sheet(self):
        return self._cheat_sheet.checkpoint_payload()

    # ミッション開始通知
    def mission_start(self, req):
        rospy.loginfo("mission_start")

        payload = {
            "status": "Mission Start"
        }

        success, message = self.publish_event(payload)

        res = MissionStart._response_class()
        if hasattr(res, "success"):
            res.success = success
        if hasattr(res, "message"):
            res.message = message

        return res

    # ミッション完了通知
    def mission_finished(self, req):
        rospy.loginfo("mission_finished")

        success, message = self.publish_event({
            "status": "Mission Finish"
        })

        res = MissionFinished._response_class()
        if hasattr(res, "success"):
            res.success = success
        if hasattr(res, "message"):
            res.message = message

        return res

    # ==========================================================
    # Action
    # ==========================================================
    # チェックポイント通過通知
    def checkpoint_passed(self, goal):
        rospy.loginfo(f"checkpoint_passed start: {goal.checkpoint_id}")

        # ---- Feedback（開始）
        fb = CheckpointPassedFeedback()
        fb.status = "processing"
        self._checkpoint_server.publish_feedback(fb)

        # ---- 採点イベント通知
        success, message = self.publish_event({
            "status": "CheckPointPassed",
            "checkpoint_id": goal.checkpoint_id
        })

        # ---- Feedback（完了）
        fb.status = "completed"
        self._checkpoint_server.publish_feedback(fb)

        # ---- Result
        result = CheckpointPassedResult()
        result.success = success
        result.message = message

        # 判定の不成立は業務上の正常系として succeeded で返す。
        # 例外・内部異常のみ aborted とする。
        if message.startswith("ros shutdown"):
            self._checkpoint_server.set_aborted(result)
        else:
            self._checkpoint_server.set_succeeded(result)

    # ==========================================================
    def run(self):
        rospy.loginfo("KrpcApiNode started")
        rospy.spin()


if __name__ == "__main__":
    KrpcApiNode().run()
