import math
from auto_runner.lib.common import Dir, State


class RobotCntrolMange:
    # 목적지
    destination = None
    # 회전토크
    rotate_torque = [0.6, 0.6, 0.2]
    # 회전각도
    rotate_ccw_angle = 0.85 * math.pi / 2.0
    # 직진토크
    fwd_torque = [1.0, 0.6, 0.2]
    # x,y좌표 (x는 전후, y는 좌우)
    cur_dir: Dir
    old_dir: Dir
    new_angular: float
    old_angular: float
    new_pose: tuple
    old_pose: tuple
    current_state: State

    def __init__(self, node:object, dir: Dir, logger) -> None:
        self.node = node
        self.cur_dir = self.old_dir = dir
        self.current_state = State.IDLE
        self.new_pose = self.old_pose = (0, 0)
        self.new_angular = self.old_angular = 0.0
        self.amend_h_count = 0
        self.logger = logger

    # 목적위치를 입력받고, 다음동작을 정한다.
    def next_action(self, next_pose: tuple) -> tuple[float, float]:
        self.logger.info(f"_get_torq_theta_dir: {self.cur_dir}")
        self.current_state = State.IDLE
        x0, y0 = self.new_pose
        x1, y1 = next_pose
        _next_dir = self.cur_dir
        
        output = self.fwd_torque[1], 0.0  # 직진

        if self.cur_dir == Dir.UP:
            if y1 > y0:
                output = self.rotate_torque[1], self.rotate_ccw_angle  # 좌회전
                _next_dir = Dir.RIGHT
            elif y1 < y0:
                output = self.rotate_torque[1], -self.rotate_ccw_angle  # 우회전
                _next_dir = Dir.LEFT

        elif self.cur_dir == Dir.DOWN:
            if y1 > y0:
                output = self.rotate_torque[1], -self.rotate_ccw_angle  # 우회전
                _next_dir = Dir.RIGHT
            elif y1 < y0:
                output = self.rotate_torque[1], self.rotate_ccw_angle  # 좌회전
                _next_dir = Dir.LEFT

        elif self.cur_dir == Dir.RIGHT:
            if x1 > x0:
                output = self.rotate_torque[1], -self.rotate_ccw_angle  # 우회전
                _next_dir = Dir.UP
            elif x1 < x0:
                output = self.rotate_torque[1], self.rotate_ccw_angle  # 좌회전
                _next_dir = Dir.DOWN

        elif self.cur_dir == Dir.LEFT:
            if x1 > x0:
                output = self.rotate_torque[1], self.rotate_ccw_angle  # 좌회전
                _next_dir = Dir.UP
            elif x1 < x0:
                output = self.rotate_torque[1], -self.rotate_ccw_angle  # 우회전
                _next_dir = Dir.DOWN

        self.cur_dir = _next_dir
        self.old_pose = self.new_pose
        self.old_angular = self.new_angular

        if _next_dir in [Dir.LEFT, Dir.RIGHT]:
            self.current_state = State.ROTATING
        return output

    # 처리완료 체크
    def check_complete(self):
        # 상태에 따른 처리를 모니터링한다.
        # 회전 및 전진
        pass

    # 회전상태
    def check_rotation(self):
        rotation_angle = self.__angle_diff()
        if math.pi / 2 * 0.8 < rotation_angle:
            self.node.send_message(x=0.2)
            self.logger.info(f"angular_diff: {rotation_angle}")
            return True
        
        self.current_state = State.IDLE
        return False

    #수평상태
    def check_straight(self):
        amend_theta = self._get_deviation_radian(self.new_angular)
        # 간격을 두어 보정한다.
        # 잦은 조정에의한 좌우 흔들림 방지.
        if self.amend_h_count % 3 == 0 and abs(amend_theta) > 3 / 180 * math.pi:
            self.node.send_message(x=0.6,z=amend_theta * 3 / 5)
            self.logger.info(
                f"[{self.cur_dir} 위치보정] 보정 각:{amend_theta} / 기준:{2 / 180 * math.pi}"
            )
            self.amend_h_count = 0
        else:        
            self.amend_h_count += 1

     # 각도를 입력받아, 진행방향과 벗어난 각도를 반환한다.
    def _get_deviation_radian(self, _z: float) -> tuple[float, str]:
        # 유니티에서 음의 값으로 제공.
        angular_z = -_z % (math.pi * 2)

        if self.dir == "x":
            amend_theta = angular_z - math.pi

        elif self.dir == "-x":
            angular_z = angular_z if angular_z > math.pi else math.pi * 2 + angular_z
            amend_theta = angular_z - math.pi * 2

        elif self.dir == "y":
            amend_theta = angular_z - math.pi / 2

        elif self.dir == "-y":
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
    def check_position_error(self):
        pass

    def __angle_diff(self, angle1, angle2):
        new_angular = self.new_angular
        if new_angular < self.old_angular:
            new_angular += 2 * math.pi

        return new_angular - self.old_angular

    # 다음위치
