class State:
    def __eq__(self, other):
        return self.__class__ == other.__class__

    def get_name(self):
        raise NotImplementedError('get_name() not implemented')

    def get_type(self):
        raise NotImplementedError('get_type() not implemented')

    def on_event(self, name, data):
        raise NotImplementedError('on_event() not implemented')


class ObstacleState(State):
    def get_type(self):
        return 'obstacle'


class CurrentActionState(State):
    def get_type(self):
        return 'current_action'


class NoObstacle(ObstacleState):
    def get_name(self):
        return 'no_obstacle'

    def on_event(self, name, data):
        if name != 'distance_sensor' or not isinstance(data, float):
            return self

        if data <= 0.3:
            return FrontObstacle()

        return self


class FrontObstacle(ObstacleState):
    def get_name(self):
        return 'front_obstacle'

    def on_event(self, name, data):
        if name != 'distance_sensor' or not isinstance(data, float):
            return self

        if data > 0.4:
            return NoObstacle()

        return self


class Stop(CurrentActionState):
    def get_name(self):
        return 'stop'

    def on_event(self, name, data):
        if name != self.get_type() or not isinstance(data, str):
            return self

        if data == 'move_forward':
            return MoveForward()
        elif data == 'turn_left':
            return TurnLeft()
        elif data == 'turn_right':
            return TurnRight()

        return self


class MoveForward(CurrentActionState):
    def get_name(self):
        return 'move_forward'

    def on_event(self, name, data):
        if name != self.get_type() or not isinstance(data, str):
            return self

        if data == 'stop':
            return Stop()
        elif data == 'turn_left':
            return TurnLeft()
        elif data == 'turn_right':
            return TurnRight()

        return self


class TurnLeft(CurrentActionState):
    def get_name(self):
        return 'turn_left'

    def on_event(self, name, data):
        if name != self.get_type() or not isinstance(data, str):
            return self

        if data == 'move_forward':
            return MoveForward()
        elif data == 'stop':
            return Stop()
        elif data == 'turn_right':
            return TurnRight()

        return self


class TurnRight(CurrentActionState):
    def get_name(self):
        return 'turn_right'

    def on_event(self, name, data):
        if name != self.get_type() or not isinstance(data, str):
            return self

        if data == 'move_forward':
            return MoveForward()
        elif data == 'turn_left':
            return TurnLeft()
        elif data == 'stop':
            return Stop()

        return self
