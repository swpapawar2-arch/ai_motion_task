#!/usr/bin/env python3
import json
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node

ALLOWED_ACTIONS = {"forward", "left", "right", "stop"}
LINEAR_SPEED = 0.20
ANGULAR_SPEED = 0.55
PUBLISH_PERIOD = 0.1
MIN_DURATION = 0.5
MAX_DURATION = 4.0

class MotionPlanRunner(Node):
    def __init__(self):
        super().__init__("motion_plan_runner")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        package_share = Path(get_package_share_directory("ai_motion_task"))
        plan_path = package_share / "config" / "motion_plan.json"
        self.actions = self.load_and_validate(plan_path)
        self.index = 0
        self.current_action = None
        self.action_end_time = 0.0
        self.finished = False
        self.completion_logged = False
        self.timer = self.create_timer(PUBLISH_PERIOD, self.tick)
        self.get_logger().info(f"Loaded {len(self.actions)} actions from {plan_path}")

    def load_and_validate(self, plan_path):
        try:
            with plan_path.open(encoding="utf-8") as file:
                plan = json.load(file)
        except FileNotFoundError as error:
            raise RuntimeError(f"motion_plan.json not found: {plan_path}") from error
        except json.JSONDecodeError as error:
            raise RuntimeError(
                f"Invalid JSON line={error.lineno} column={error.colno}: {error.msg}"
            ) from error
        actions = plan.get("actions")
        if not isinstance(actions, list) or not actions:
            raise RuntimeError("actions must be a non-empty list")
        validated = []
        for index, item in enumerate(actions, start=1):
            if not isinstance(item, dict):
                raise RuntimeError(f"Action {index} must be an object")
            action = item.get("action")
            duration = item.get("duration")
            if action not in ALLOWED_ACTIONS:
                raise RuntimeError(f"Unknown action at {index}: {action}")
            if not isinstance(duration, (int, float)):
                raise RuntimeError(f"Duration at {index} is not numeric: {duration}")
            duration = float(duration)
            if not MIN_DURATION <= duration <= MAX_DURATION:
                raise RuntimeError(f"Duration at {index} is out of range: {duration}")
            validated.append({"action": action, "duration": duration})
        return validated

    def command_for(self, action):
        command = Twist()
        if action == "forward":
            command.linear.x = LINEAR_SPEED
        elif action == "left":
            command.angular.z = ANGULAR_SPEED
        elif action == "right":
            command.angular.z = -ANGULAR_SPEED
        return command

    def publish_stop(self):
        self.publisher.publish(Twist())

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1_000_000_000
        if self.finished:
            self.publish_stop()
            if not self.completion_logged:
                self.get_logger().info("Motion plan completed. Robot stopped.")
                self.completion_logged = True
            return
        if self.current_action is not None and now < self.action_end_time:
            self.publisher.publish(self.command_for(self.current_action))
            return
        if self.current_action is not None:
            self.publish_stop()
            self.current_action = None
        if self.index >= len(self.actions):
            self.finished = True
            self.publish_stop()
            return
        item = self.actions[self.index]
        self.index += 1
        self.current_action = item["action"]
        self.action_end_time = now + item["duration"]
        self.get_logger().info(
            f"Executing {self.current_action} for {item['duration']:.2f} seconds"
        )

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotionPlanRunner()
        rclpy.spin(node)
    except RuntimeError as error:
        print(f"ERROR: {error}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
