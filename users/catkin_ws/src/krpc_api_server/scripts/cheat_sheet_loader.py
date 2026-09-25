#!/usr/bin/python3
# -*- coding:utf-8 -*-

import json


class CheatSheetLoader:
    """Load and normalize the shared cheat_sheet.json contract once."""

    def __init__(self, data, path):
        self.path = path
        self.data = data
        self.checkpoints = self._normalize_checkpoints(data)
        self.goal = self._normalize_goal(data)

    @classmethod
    def from_path(cls, path):
        with open(path, mode="r", encoding="utf-8") as stream:
            data = json.load(stream)
        if not isinstance(data, dict):
            raise ValueError("cheat_sheet root must be an object")
        return cls(data, path)

    @staticmethod
    def _normalize_checkpoints(data):
        checkpoints = data.get("Checkpoints")
        if checkpoints is None:
            checkpoints = data.get("checkpoints", [])
        if not isinstance(checkpoints, list):
            raise ValueError("Checkpoints must be an array")

        normalized = []
        for index, item in enumerate(checkpoints, start=1):
            if not isinstance(item, dict):
                raise ValueError("each checkpoint must be an object")
            checkpoint = dict(item)
            checkpoint_id = item.get("checkpointid")
            if checkpoint_id is None:
                checkpoint_id = item.get("checkpoint_id")
            if checkpoint_id is None:
                checkpoint_id = item.get("id", index)
            checkpoint["checkpoint_id"] = checkpoint_id
            normalized.append(checkpoint)
        return normalized

    @staticmethod
    def _normalize_goal(data):
        goal = data.get("Goal")
        if goal is None:
            goal = data.get("goal", {})
        if not isinstance(goal, dict):
            raise ValueError("Goal must be an object")
        normalized = dict(goal)
        normalized["checkpoint_id"] = goal.get("checkpoint_id")
        return normalized

    def checkpoint_payload(self):
        checkpoints = []
        for item in self.checkpoints:
            checkpoints.append({
                "checkpoint_id": item["checkpoint_id"],
                "position": item.get("position"),
                "marker_pose": item.get("marker_pose"),
                "score_bands": item.get("score_bands", item.get("bands")),
                "attitude": item.get("attitude"),
            })
        return {
            "checkpoint_count": len(checkpoints),
            "checkpoints": checkpoints,
            "goalpoint_id": self.goal.get("checkpoint_id"),
        }
