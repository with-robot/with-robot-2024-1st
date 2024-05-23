import math
from auto_runner.lib.common import Dir, State, MessageHandler, TypeVar, StateData

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)


class RobotCntrolMange:
    """로봇의 방향조정과 목적위치까지 처리에 대한 책임을 갖는다."""

    # 목적지
    destination = None
    # 회전토크
    rotate_torque = [0.6, 0.6, 0.2]
    # 회전각도
    rot_angle_ccw = 0.85 * math.pi / 2.0
    # 직진토크
    fwd_torque = [1.0, 0.6, 0.2]
    # x,y좌표 (x는 전후, y는 좌우)
    dir_data: StateData
    angular_data: StateData
    pose_data: StateData
    state_data: StateData

    def __init__(self, node: LoggableNode, dir: Dir=Dir.UP) -> None:
        self.node = node
        self.dir_data = StateData(dir, dir)
        self.angular_data = StateData(0.0, 0.0)
        self.pose_data = StateData((0, 0), (0, 0))
        self.state_data = StateData(State.STOPPED, State.STOPPED)
        self.amend_h_count = 0

    # tf데이터 수신
    def sensordata(self, twistStamped: object):
        new_pose = (twistStamped.twist.linear.x, twistStamped.twist.linear.y)
        self.pose_data.shift(new_pose)
        self.angular_data.shift(twistStamped.twist.angular.z)

    # 목적위치를 입력받고, 다음동작을 정한다.
    def next_action(self, next_pose: tuple) -> tuple[float, float]:
        self.node._logging(f"cur_dir: {self.dir_data.new}")

        ROT_TORQUE = self.rotate_torque[1]
        FWD_TORQUE = self.fwd_torque[1]
        ROT_ANGLE = self.rot_angle_ccw

        x0, y0 = self.pose_data.new
        x1, y1 = next_pose
        
        _cur_dir = self.dir_data.new

        if _cur_dir == Dir.UP:
            if y1 > y0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir.RIGHT
            elif y1 < y0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir.LEFT

        elif _cur_dir == Dir.DOWN:
            if y1 > y0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir.RIGHT
            elif y1 < y0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir.LEFT

        elif _cur_dir == Dir.RIGHT:
            if x1 > x0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir.UP
            elif x1 < x0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir.DOWN

        elif _cur_dir == Dir.LEFT:
            if x1 > x0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir.UP
            elif x1 < x0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir.DOWN

        else:
            _torq_ang = FWD_TORQUE, 0.0  # 직진
            _next_dir = self.dir_data.new            

        if _torq_ang != 0.0:
            self.state_data.shift(State.ROTATING)

        # 방향 및 목적위치
        self.dir_data.shift(_next_dir)
        self.pose_data.shift(next_pose)

        return _torq_ang

    # 처리완료 체크
    def check_complete(self):
        # 상태에 따른 처리를 모니터링한다.
        # 회전 및 전진
        pass

    # 회전상태
    def check_rotation(self):
        rotation_angle = self._angle_diff()
        if math.pi / 2 * 0.8 < rotation_angle:
            self.node._send_message(x=0.2)
            self.node._logging(f"angular_diff: {rotation_angle}")
            return True

        self.new_state = State.STOPPED
        return False

    # 수평상태
    def check_straight(self):
        amend_theta = self._get_deviation_radian(self.new_angular)
        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if self.amend_h_count % 3 == 0 and abs(amend_theta) > 3 / 180 * math.pi:
            self.node._send_message(x=0.6, z=amend_theta * 3 / 5)
            self.logger.info(
                f"[{self.new_dir} 위치보정] 보정 각:{amend_theta} / 기준:{2 / 180 * math.pi}"
            )
            self.amend_h_count = 0
        else:
            self.amend_h_count += 1

    # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self, _z: float) -> tuple[float, str]:
        # 유니티에서 음의 값으로 제공.
        angular_z = -_z % (math.pi * 2)
        cur_dir = self.dir_data.new
        if cur_dir == Dir.UP:
            amend_theta = angular_z - math.pi

        elif cur_dir == Dir.DOWN:
            angular_z = angular_z if angular_z > math.pi else math.pi * 2 + angular_z
            amend_theta = angular_z - math.pi * 2

        elif cur_dir == Dir.LEFT:
            amend_theta = angular_z - math.pi / 2

        elif cur_dir ==  Dir.RIGHT: #'-y'
            amend_theta = angular_z - math.pi * 3 / 2
        else:
            amend_theta = 0.0

        return amend_theta

    def do_stabilize(self):
        pass

    # 목표위치 도착여부를 체크, 처리 이벤트를 호출한다.
    # def arrived(self, pose:tuple, dest_pos:tuple):
    #     if pose == dest_pos:
    #         self.current_state = State.COMPLETE

    # 차랑을 정지시키다.
    def stop(self):
        pass

    # 위치이탈체크
    def _check_position_error(self):
        pass

    def _angle_diff(self):
        new_angular = self.angular_data.new
        if new_angular < self.angular_data.old:
            new_angular += 2 * math.pi

        return new_angular - self.angular_data.old
