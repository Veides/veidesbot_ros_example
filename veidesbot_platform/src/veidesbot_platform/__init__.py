import rospy
from geometry_msgs.msg import Twist
from veides_agent_ros.msg import Fact, Action, Trail
from veides_agent_ros.srv import (
    TrailsRequest,
    EventRequest,
)
from veidesbot_platform.states import (
    NoObstacle,
    FrontObstacle,
    MoveForward,
    Stop,
    TurnLeft,
    TurnRight,
)


PI = 3.1415926535897


class VeidesBotPlatform:
    obstacle_state = NoObstacle()
    current_action_state = Stop()

    def __init__(self, twist, veides_facts, veides_trails, veides_event):
        self.twist = twist
        self.veides_facts = veides_facts
        self.veides_trails = veides_trails
        self.veides_event = veides_event

        self.last_range = None

        # send ready event
        self.veides_event('ready_to_rock')
        # send initial facts
        self.veides_facts([
            Fact(name='battery_level', value='full')
        ])

    def on_tick(self, seconds):
        msg = TrailsRequest()
        msg.trails = [
            Trail(name='up_time', value=str(seconds))
        ]
        # send example trail (to see the value you need to setup dashboard first)
        self.veides_trails(msg)

    def stop(self, *_):
        msg = self._create_twist()

        self.twist.publish(msg)

    def move_forward(self, *_):
        speed = 0.3

        msg = self._create_twist()
        msg.linear.x = speed

        self.twist.publish(msg)

    def turn_left(self, *_):
        self._turn(-1)
        self.current_action_state = Stop()

    def turn_right(self, *_):
        self._turn(1)
        self.current_action_state = Stop()

    def on_distance_updated(self, updated_range):
        new_range = round(updated_range, 1)

        new_state = self.obstacle_state.on_event('distance_sensor', new_range)

        if new_state != self.obstacle_state:
            try:
                # send facts
                self.veides_facts([
                    Fact(name=new_state.get_type(), value=new_state.get_name())
                ])
            except Exception as e:
                rospy.logerr(e)

        self.obstacle_state = new_state

        if new_range != self.last_range:
            msg = TrailsRequest()
            msg.trails = [
                Trail(name='distance_sensor', value=str(new_range))
            ]
            # send trail (to see the value you need to setup dashboard first)
            self.veides_trails(msg)

            self.last_range = new_range

    def on_action(self, data, entities):
        new_state = self.current_action_state.on_event('current_action', data)
        result = None

        if new_state != self.current_action_state:
            self.current_action_state = new_state

            if hasattr(self, data):
                action = getattr(self, data)
                result = action(entities)

            try:
                # send facts when bot state changed
                self.veides_facts([
                    Fact(name=new_state.get_type(), value=new_state.get_name())
                ])
            except Exception as e:
                rospy.logerr(e)

        return result

    def _turn(self, direction, angle=90):
        # TODO: reimplement to be odometry based
        self.stop()

        speed = 60

        angular_speed = speed * PI / 180

        msg = self._create_twist()
        msg.angular.z = direction * angular_speed

        start_time = rospy.Time.now()
        current_angle = 0

        self.twist.publish(msg)

        while current_angle < angle:
            duration = rospy.Time.now() - start_time
            current_angle = (speed * duration.to_sec())

        self.stop()

    def _create_twist(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        return msg
