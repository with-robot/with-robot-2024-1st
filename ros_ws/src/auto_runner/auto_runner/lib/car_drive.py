import math
from auto_runner.lib.common import Dir, State, MessageHandler, TypeVar, StateData

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)


class RobotController:
    """로봇의 방향조정과 목적위치까지 처리에 대한 책임을 갖는다."""

    # 목적지
    destination = None
    # 회전토크
    rotate_torque = [1.0, 0.6, 0.2]
    # 회전각도
    rot_angle_ccw = 1.33 #0.85 * math.pi / 2.0 => 76도
    # 직진토크
    fwd_torque = [1.0, 0.2, 0.2]
    # x,y좌표 (x는 전후, y는 좌우)
    dir_data: StateData
    angular_data: StateData
    pos_data: StateData
    state_data: StateData

    def __init__(self, node: LoggableNode, dir: Dir = Dir.X) -> None:
        self.node = node
        self.dir_data = StateData(dir, dir)
        self.angular_data = StateData(0.0, 0.0)
        self.pos_data = StateData((0, 0), (0, 0))
        self.state_data = StateData(State.ROTATE_STOP, State.ROTATE_STOP)
        self.amend_h_count = 0

    # tf데이터 수신
    def set_tfdata(self, twistStamped: object):
        new_pose = (twistStamped.twist.linear.x, twistStamped.twist.linear.y)
        self.pos_data.cur =new_pose
        self.angular_data.cur=twistStamped.twist.angular.z

    # 목적위치를 입력받고, 다음동작을 정한다.
    def next_action(self, cur_pos:tuple, next_pos: tuple) -> tuple[float, float]:
        self.node._logging(f"next_pose: {next_pos}")

        ROT_TORQUE = self.rotate_torque[0]
        FWD_TORQUE = self.fwd_torque[1]
        ROT_ANGLE = self.rot_angle_ccw

        x0, y0 = cur_pos
        x1, y1 = next_pos

        _next_dir=_cur_dir=self.dir_data.cur
        
        _torq_ang = FWD_TORQUE, 0.0  # 직진
        if _cur_dir == Dir.X:
            if y1 > y0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir.Y
            elif y1 < y0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir._Y

        elif _cur_dir == Dir._X:
            if y1 > y0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir.Y
            elif y1 < y0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir._Y

        elif _cur_dir == Dir.Y:
            if x1 > x0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir.X
            elif x1 < x0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir._X

        elif _cur_dir == Dir._Y:
            if x1 > x0:
                _torq_ang = ROT_TORQUE, ROT_ANGLE  # 좌회전
                _next_dir = Dir.X
            elif x1 < x0:
                _torq_ang = ROT_TORQUE, -ROT_ANGLE  # 우회전
                _next_dir = Dir._X

        if _torq_ang[1] != 0.0:
            self.state_data.shift(State.ROTATING)

        # 방향 및 목적위치
        self.dir_data.shift(_next_dir)

        self.angular_data.old=self.angular_data.cur
        self.pos_data.old=self.pos_data.cur

        self.node._logging(
            f"{self.dir_data}\n{self.state_data}\n{self.pos_data}\n{self.angular_data}"
        )
        return _torq_ang

    # 회전상태 여부
    def is_rotate_state(self):
        return self.state_data.cur == State.ROTATING
    
    #막다른 골목위치 체크
    def check_obstacle(self):
        # 후진모드설정
        pass

    # 회전이 종료되면 True반환
    def check_rotation_complete(self):
        self.node._logging(f"<<is_rotating>> dir_data: {self.dir_data}, angular_data: {self.angular_data}")

        def __angle_diff():
            _diff = self.angular_data.cur - self.angular_data.old
            if _diff >  2*math.pi:
                _diff -= 2 * math.pi
            elif _diff < -2*math.pi:
                _diff += 2 * math.pi
            
            self.node._logging(f"angular_diff: {_diff}")
            return math.fabs(_diff)

        if 0.3 < __angle_diff() < math.pi / 2 * 0.7:
            self.node._send_message(title="회전처리", x=0.2)
            return False

        self.state_data.shift(State.ROTATE_STOP)
        return True

    # def is_same_block(self, comparator: callable):
    #     self.node._logging(f"<<is_same_block>> pos_data: {self.pos_data}")
    #     return comparator(self.pos_data.cur) == comparator(self.pos_data.old)

    # 수평상태
    def adjust_body(self):
        amend_theta = self._get_deviation_radian()
        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if self.amend_h_count % 5 == 0 and abs(amend_theta) > 3 / 180 * math.pi:
            self.node._send_message(title="수평보정", x=0.2, theta=amend_theta * 2 / 5)
            self.node._logging(
                f"[{self.dir_data.cur} 위치보정] 보정 각:{amend_theta} / 기준:{2 / 180 * math.pi}"
            )
            self.amend_h_count = 0
        else:
            self.amend_h_count += 1

    # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self) -> tuple[float, str]:
        # 유니티에서 음의 값으로 제공.
        angular_z = math.fabs(self.angular_data.cur % (math.pi * 2))

        cur_dir = self.dir_data.cur
        if cur_dir == Dir.X:
            amend_theta = angular_z - math.pi

        elif cur_dir == Dir._X:
            angular_z = angular_z if angular_z > math.pi else math.pi * 2 + angular_z
            amend_theta = angular_z - math.pi * 2

        elif cur_dir == Dir.Y:
            amend_theta = angular_z - math.pi / 2

        elif cur_dir == Dir._Y:  #'-y'
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
    def stop_car(self):
        self.node._logging("<<stop_car>>")

    # 위치이탈체크
    def _check_position_error(self):
        pass
