import math
from auto_runner.lib.common import Orient, State, MessageHandler, TypeVar, StateData, DirType
from auto_runner.lib.path_location import PathFinder

LoggableNode = TypeVar("LoggableNode", bound=MessageHandler)


class RobotController:
    """로봇의 방향조정과 목적위치까지 처리에 대한 책임을 갖는다."""

    path_finder: PathFinder
    # 회전토크
    # 회전각도
    # 직진토크
    rotate_torque = 0.3
    rot_angle_ccw = 1.3  # 0.85 * math.pi / 2.0 => 76도
    fwd_torque = 0.5

    # x,y좌표 (x는 전후, y는 좌우)
    dir_data: StateData
    angular_data: StateData
    pos_data: StateData
    state_data: StateData

    def __init__(
        self, node: LoggableNode, path_finder: PathFinder, dir: Orient = Orient.X
    ) -> None:
        self.node = node
        self.path_finder = path_finder
        self.dir_data = StateData(dir, dir)
        self.angular_data = StateData(0.0, 0.0)
        self.pos_data = StateData((0, 0), (0, 0))
        self.state_data = StateData(State.ROTATE_STOP, State.ROTATE_STOP)
        self.amend_h_count = 0
        self.__t_diff = None

    # tf데이터 수신
    def update_pos(self, twistStamped: object):
        new_pose = (twistStamped.twist.linear.x, twistStamped.twist.linear.y)
        self.pos_data.cur = new_pose
        self.angular_data.cur = twistStamped.twist.angular.z

        # 좌표설정
        self.path_finder.set_cur_pos(self.pos_data.cur)

    def check_arrival(self, dest: tuple = None, finish: bool = False):
        # 도착처리_다음 목적지 설정
        return self.path_finder.check_arrival(self.dir_data.cur, dest, finish)

    def update_map(self, grid_map: list):
        self.path_finder.update_map(grid_map)

    # 목적위치를 입력받고, 다음동작을 정한다.
    def next_action(self) -> tuple[float, float]:
        cur_pos = self.path_finder.cur_pos

        # 정상괘도 이탈시에만 경로 재검색.
        # next_pos = self.path_finder.check_and_dest(self.dir_data.cur)
        if self.path_finder._check_pose_error(cur_pos):
            next_pos = self.path_finder.check_and_dest(self.dir_data.cur)
        else:
            next_pos = self.path_finder.get_next_pos(cur_pos)

        if not next_pos:
            self.stop_car()
            # TODO: 차량 초기화 처리
            raise Exception("controller stopped")

        self.node.print_log(f"next_pose: {cur_pos} => {next_pos}")

        LEFT_TURN = (self.rotate_torque, self.rot_angle_ccw)
        RIGHT_TURN = (self.rotate_torque, -self.rot_angle_ccw)
        MOVE_FWD = (self.fwd_torque, 0.0)
        # MOVE_BWD = (-self.fwd_torque, 0.0)
        x0, y0 = cur_pos
        x1, y1 = next_pos

        _next_dir = _cur_dir = self.dir_data.cur

        _torq_ang = MOVE_FWD  # 직진
        _next_dir = self.dir_data.cur
        if _cur_dir == Orient.X:
            if y1 > y0:
                _torq_ang, _next_dir = LEFT_TURN, Orient.Y  # 좌회전
            elif y1 < y0:
                _torq_ang, _next_dir = RIGHT_TURN, Orient._Y  # 우회전

        elif _cur_dir == Orient._X:
            if y1 > y0:
                _torq_ang, _next_dir = RIGHT_TURN, Orient.Y  # 우회전
            elif y1 < y0:
                _torq_ang, _next_dir = LEFT_TURN, Orient._Y  # 좌회전

        elif _cur_dir == Orient.Y:
            if x1 > x0:
                _torq_ang, _next_dir = RIGHT_TURN, Orient.X  # 우회전
            elif x1 < x0:
                _torq_ang, _next_dir = LEFT_TURN, Orient._X  # 좌회전

        elif _cur_dir == Orient._Y:
            if x1 > x0:
                _torq_ang, _next_dir = LEFT_TURN, Orient.X  # 좌회전
            elif x1 < x0:
                _torq_ang, _next_dir = RIGHT_TURN, Orient._X  # 우회전

        if _torq_ang[1] != 0.0:
            self.state_data.shift(State.ROTATE_READY)

        # 방향 및 목적위치
        self.dir_data.shift(_next_dir)

        self.angular_data.old = self.angular_data.cur
        self.pos_data.old = self.pos_data.cur
        self.torq_ang = _torq_ang

        self.node.print_log(
            f"{self.dir_data}\n{self.state_data}\n{self.pos_data}\n{self.angular_data}"
        )
        return _torq_ang

    # 회전상태 여부
    def is_rotate_state(self):
        # cond1:bool = math.fabs(self.angular_data.cur - self.angular_data.old) > 0.01
        cond2: bool = self.state_data.cur in [State.ROTATE_START, State.ROTATE_READY]
        return cond2

    # 막다른 골목위치 체크
    def check_obstacle(self):
        # 후진모드설정
        pass

    # 회전이 종료되면 True반환
    def check_rotate_state(self):

        if not self.is_rotate_state():
            return False

        self.node.print_log(
            f"<<on_rotating>> dir_data: {self.dir_data}, angular_data: {self.angular_data}"
        )

        def __angle_diff(cur, old):
            old %= (2*math.pi)
            diff = math.fabs(cur - old)
            return diff if diff < math.pi * 3 / 2 else 2 * math.pi - diff

        # __t_diff = __angle_diff(self.angular_data.cur, self.angular_data.old)
        orient = self.dir_data.old
        dir = DirType.LEFT if math.copysign(1, self.torq_ang[1]) >0 else DirType.RIGHT
        __t_diff = __angle_diff(self._get_target_angle(orient, dir), self.angular_data.cur)
        
        self.node.print_log(f"angular_diff: {__t_diff}, cur_stat:{self.state_data.cur}")

        if __t_diff < 0.2 and self.state_data.cur==State.ROTATE_START:
            self.state_data.shift(State.ROTATE_STOP)
            self.node._send_message(title="회전토크 0.01", x=0.01, theta=self.torq_ang[1])
            return False
        else:
            if __t_diff >= 0.2:
                self.state_data.cur = State.ROTATE_START
            self.node._send_message(title="회전토크 0.15", x=0.15, theta=self.torq_ang[1])
            return True
        
    
    # 목표 각도 설정
    def _get_target_angle(self, orient: Orient, dir: DirType) -> float:
        if (orient==Orient.X and dir==DirType.LEFT) or (orient==Orient._X and dir==DirType.RIGHT):
            return math.pi/2
        elif (orient==Orient.Y and dir==DirType.LEFT) or (orient==Orient._Y and dir==DirType.RIGHT):
            return math.pi
        elif (orient==Orient.X and dir==DirType.RIGHT) or (orient==Orient._Y and dir==DirType.LEFT):
            return 3*math.pi/2
        else:
            return 2*math.pi

    # PID 제어
    def _get_break_torque(
        self,
        old_angle: float,
        new_angle: float,
        target_angle: float,
        max_torque: float = 0.2,
        min_torque: float = 0.01,
    ) -> float:
        """
        현재 회전각도에 따라 브레이크 토크의 크기를 결정한다.
        회전각도가 90도에 가까워질수록 토크 크기로 줄여나간다.
        """
        # der_torque = math.fabs((new_angle-old_angle)/old_angle)
        # if der_torque * 100 < 30:
        # return 0
        _log_ratio = (math.log(new_angle + 1.0)) / math.log(target_angle + 1.0)

        return -1 * max(max_torque * _log_ratio, min_torque)

    def _calc_dir(self):
        right_angle = math.pi / 2
        cur_angle = self.angular_data.cur % (2 * math.pi)

        if (
            0 <= cur_angle <= right_angle * 1 / 2
            or right_angle * 7 / 2 <= cur_angle <= 4 * right_angle
        ):
            return Orient.X
        elif right_angle * 1 / 2 < cur_angle <= right_angle * 3 / 2:
            return Orient.Y
        elif right_angle * 3 / 2 < cur_angle <= right_angle * 5 / 2:
            return Orient._X
        elif right_angle * 5 / 2 < cur_angle <= right_angle * 7 / 2:
            return Orient._Y

    # 수평상태
    def adjust_body(self):
        amend_theta = self._get_deviation_radian()
        # 각도가 매우 틀어진 경우, 몸체의 방향을 변경한다.
        # if math.fabs(amend_theta) > 0.6:
        #     dir_ = self._calc_dir()
        #     self.dir_data.shift(dir_)

        #     self.node.print_log(
        #         f"[방향 변경] amend_theta:{amend_theta}: {self.dir_data.old} => {self.dir_data.cur}"
        #     )
        #     return True

        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if abs(amend_theta) > 3 / 180 * math.pi:
            self.node.print_log(
                f"[{self.dir_data.cur} 위치보정] {self.dir_data} >> 보정 각:{amend_theta} / 기준:{3 / 180 * math.pi}"
            )
            self.node._send_message(title="수평보정", x=0.5, theta=amend_theta)
            # self.amend_h_count = 0
            return self.pos_data.cur == self.pos_data.old

        # self.amend_h_count += 1
        return False

    # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self) -> tuple[float, str]:
        # 유니티에서 로봇의 기본회전상태가 180 CCW상태, 즉 음의 값.
        # 회전방향 수치를 양수로 정리한다.
        # -0.1 => 6.23
        _cur_angle = self.angular_data.cur % (math.pi * 2)
        _cur_dir = self.dir_data.cur

        if _cur_dir == Orient.X:
            amend_theta = (
                -_cur_angle if _cur_angle < math.pi / 2 else 2 * math.pi - _cur_angle
            )

        elif _cur_dir == Orient._X:
            amend_theta = math.pi - _cur_angle

        elif _cur_dir == Orient.Y:
            # -180 ~ -270
            # 차이량에 대해 음수면 바퀴가 우측방향으로 돌게된다. (우회전으로 보정)
            amend_theta = math.pi / 2 - _cur_angle

        elif _cur_dir == Orient._Y:
            amend_theta = math.pi * 3 / 2 - _cur_angle

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
        self.node.print_log("<<stop_car>>")
