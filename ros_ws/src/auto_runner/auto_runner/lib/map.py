from auto_runner.lib.common import DirType, Orient

# 맵을 입력받아 좌표를 계산하고 이동경로를 저장한다.
# 후진이 필요한 경우 이에 필요한 경로를 제공한다.
class Map:
    _data:list[list[tuple]] = None
    _move_path:list[tuple]

    @property
    def data(self):
        return self._data
    @map.setter
    def data(self, data):
        self._data = data

    def update_pos(self, pos):
        if self._move_path[-1] == pos:
            return
        self._move_path.append(pos)

    def get_back_path(self):
        return self._move_path[::-1]

    def is_intrap(self, pos:tuple, orient:Orient):
        x, y = pos
        _trap_map = {            
            orient.Y: self.map[x][y-1],
            orient._Y: self.map[x][y+1],
            orient.X: self.map[x-1][y],
            orient._X: self.map[x+1][y],
        }
        
        return all( v>0 for k,v in _trap_map.pop(orient).items())        