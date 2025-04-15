from peregrine.pipelines.ros2.utils import ros2_env_setup
ros2_env_setup()
import rclpy
if not rclpy.ok():
    rclpy.init()
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

from peregrine.twins import ScenarioScript
class AMRPathPlanning(ScenarioScript):
    def begin_play(self):
        super().begin_play()
        self.amr_subscriber = AMRSubscriber()
        self.amr = self.get_world().get_actors_by_tag("DuScope:/Simulation/Characters/Turtlebot3Waffle_0")[0]

    def tick(self, delta_time):
        super().tick(delta_time)
        rclpy.spin_once(self.amr_subscriber, timeout_sec=1/60)
        try:
            move_step = {"X": self.amr_subscriber.amr_vel.x*delta_time, "Y": self.amr_subscriber.amr_vel.y*delta_time, "Z": 0.0}
            rotator = {"Pitch": 0.0, "Yaw": -self.amr_subscriber.amr_ang_vel*delta_time*180/math.pi, "Roll": 0.0}
            self.visual_correction(self.amr_subscriber.amr_vel.x,self.amr_subscriber.amr_vel.y,self.amr_subscriber.amr_ang_vel)
            self.amr.K2_AddActorWorldOffset(move_step)
            self.amr.K2_AddActorWorldRotation(rotator)
        except:
            print("Missing ROS Data")

    def end_play(self):
        self.amr_subscriber.destroy_node()
        super().end_play()

    def visual_correction(self, dx, dy, dtheta):
        if dtheta == 0:
            if dx>0 and dy>=0:
                angle = math.atan(dy/dx)*180/math.pi
            elif dx<0 and dy>=0:
                angle = math.atan(dy/dx)*180/math.pi+180
            elif dx<0 and dy<=0:
                angle = math.atan(dy/dx)*180/math.pi-180
            elif dx>0 and dy<=0:
                angle = math.atan(dy/dx)*180/math.pi
            elif dx==0 and dy<0:
                angle = -90.0
            elif dx==0 and dy>0:
                angle = 90.0
            print(f"dx: {dx}, dy: {dy}, dtheta: {dtheta}, angle: {angle}")
            self.amr.K2_SetActorRotation({"Pitch": 0.0, "Yaw": angle, "Roll": 0.0})


class AMRSubscriber(Node):
    def __init__(self):
        super().__init__('amr_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.amr_vel = Twist().linear
        self.amr_ang_vel = Twist().angular.z

    def listener_callback(self, msg: Twist):
        self.amr_vel = msg.linear
        self.amr_ang_vel = msg.angular.z