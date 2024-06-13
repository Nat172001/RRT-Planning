#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from math import pow, sqrt, atan2, pi
from nav_msgs.msg import Path
from std_msgs.msg import String

class togoal(Node):
    def __init__(self):
        super().__init__("togoal")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(Pose2D, "/pose", self.set_pose_variable, rclpy.qos.qos_profile_sensor_data)
        self.sub_new_goal = self.create_subscription(Path, "/path", self.new_goal_callback, 10)
        self.create_subscription(String, "/environment",self.stop_callback,10)
        self.pub2 = self.create_publisher(Pose2D,"/gp",10)
        self.pose = Pose2D()

        self.linear_tol = 0.001
        self.angular_tol = 0.01 
        self.flag = True

        self.goalpose = Pose2D()
        self.gt = []
        self.counter = 0

        self.timer = self.create_timer(0.05, self.movetogoal)

    def stop_callback(self, msg:String):
        self.stop_movement()  # Stop the robot before changing the path
        pose = Pose2D()
        pose.x = self.pose.x
        pose.y = self.pose.y
        self.pub2.publish(pose)

    def new_goal_callback(self, msg: Path):
        self.gt = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        self.counter = 0
        if self.gt:
            self.getgoalpose()  # Update to the new starting goal pose

    def set_pose_variable(self, msg: Pose2D):
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = (msg.theta + pi) % (2 * pi) - pi

    def getgoalpose(self):
        if self.gt:
            self.goalpose.x = self.gt[self.counter][0]
            self.goalpose.y = self.gt[self.counter][1]
            self.counter += 1

    def movetogoal(self):
        if not self.gt:
            return  # No path received yet

        pub_msg = Twist()
        kv = 1  # Velocity gain
        kw = 4  # Angular velocity gain

        distance = sqrt(pow(self.goalpose.x - self.pose.x, 2) + pow(self.goalpose.y - self.pose.y, 2))
        velocity = kv * distance
        steering_angle = atan2(self.goalpose.y - self.pose.y, self.goalpose.x - self.pose.x)
        angle_diff = (steering_angle - self.pose.theta + pi) % (2 * pi) - pi

        angular_velocity = kw * angle_diff

        if abs(angle_diff) > self.angular_tol:
            pub_msg.linear.x = 0.0
            pub_msg.angular.z = angular_velocity
        else:
            pub_msg.angular.z = 0.0
            if distance > self.linear_tol:
                pub_msg.linear.x = velocity
            else:
                if self.counter < len(self.gt):
                    self.getgoalpose()
                else:
                    self.stop_movement()  # Stop if no more goals are present

        self.pub.publish(pub_msg)

    def stop_movement(self):
        # Publish a zero velocity to stop the robot
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.gt=[]
        self.pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = togoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
