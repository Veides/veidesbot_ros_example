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
        self.speed_level = 0

        # Send initial facts
        self.veides_facts([
            Fact(name='battery_level', value='full'),
            Fact(name='charging', value='no'),
        ])

        self.set_normal_speed()

        # Send ready event
        self.veides_event('ready_to_rock')

    def on_tick(self, seconds):
        # Send a trail
        self.veides_trails([
            Trail(name='uptime', value=str(seconds))
        ])

    def stop(self, *_):
        msg = self._create_twist()

        self.twist.publish(msg)

    def move_forward(self, *_):
        msg = self._create_twist()
        msg.linear.x = self.speed_level

        self.twist.publish(msg)

    def turn_left(self, *_):
        self._turn(-1)
        self.current_action_state = Stop()

    def turn_right(self, *_):
        self._turn(1)
        self.current_action_state = Stop()

    def set_low_speed(self, *_):
        self.speed_level = 0.15

        # Send a trail
        self.veides_trails([
            Trail(name='speed_level', value='low')
        ])

    def set_normal_speed(self, *_):
        self.speed_level = 0.3

        # Send a trail
        self.veides_trails([
            Trail(name='speed_level', value='normal')
        ])

    def on_distance_updated(self, updated_range):
        new_range = round(updated_range, 1)

        new_state = self.obstacle_state.on_event('distance_sensor', new_range)

        if new_state != self.obstacle_state:
            try:
                # Send facts update
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
            # Send a trail
            self.veides_trails(msg)

            self.last_range = new_range

    def on_set_action(self, action_name, entities):
        action = getattr(self, action_name, None)
        result = None

        if action is not None:
            result = action(entities)

        return result

    def on_action(self, action_name, entities):
        new_state = self.current_action_state.on_event('current_action', action_name)
        result = None

        if new_state != self.current_action_state:
            self.current_action_state = new_state

            if hasattr(self, action_name):
                action = getattr(self, action_name)
                result = action(entities)

            try:
                # Send facts update when bot state changed
                self.veides_facts([
                    Fact(name=new_state.get_type(), value=new_state.get_name())
                ])
            except Exception as e:
                rospy.logerr(e)

        return result

    def _turn(self, direction, angle=90):
        # TODO: reimplement to be odometry based
        self.stop()

        speed = self.speed_level * 200

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
