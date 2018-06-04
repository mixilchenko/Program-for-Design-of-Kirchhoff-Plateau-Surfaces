from pyglet.gl import *
from pyglet import graphics as gr
from shapely.geometry import asPolygon

from geom import Point, Line


class Boundary:
    mouse_motion_color = (0, 1, 0, 1)

    def __init__(self, control_points=[]):
        # массив контрольных точек
        self.control_points = control_points
        # массив линий
        self.lines = []
        # матрица индексов связанных точек
        self.edges = []

        # индексы точек для покраски
        self._mouse_motion_index = None
        self._mouse_press_index = None
        self.changed = True

    points_len = property(lambda self: len(self.control_points))

    def print(self):
        print(self.control_points)
        print(self.lines)

    @property
    def mouse_motion_index(self):
        return self._mouse_motion_index

    @mouse_motion_index.setter
    def mouse_motion_index(self, v):
        # если ничего не изменилось
        if self._mouse_motion_index == v:
            return
        self.changed = True
        # очищаем старый цвет, если есть
        if self._mouse_motion_index is not None:
            self.lines[self._mouse_motion_index].color = Line.default_color
        # ставим новый цвет
        self._mouse_motion_index = v
        if v is not None:
            self.lines[v].color = (0, 1, 0, 1)

    @property
    def mouse_press_index(self):
        return self._mouse_press_index

    @mouse_press_index.setter
    def mouse_press_index(self, v):
        # если ничего не изменилось
        if self._mouse_press_index == v:
            return
        self.changed = True
        # очищаем старый цвет, если есть
        if self._mouse_press_index is not None:
            self.control_points[self._mouse_press_index].color = Point.default_color
        # ставим новый цвет
        self._mouse_press_index = v
        if v is not None:
            self.control_points[v].color = (1, 0, 0, 1)

    def get_point_index(self, point):
        for i in range(self.points_len):
            if self.control_points[i].is_near(point):
                return i
        return None

    def get_line_index(self, point, with_near=False):
        if with_near:
            for i, line in enumerate(self.lines):
                if line.is_near(point):
                    return i
        # в первую очередь замкнутые
        closed = [(i, l) for i, l in enumerate(self.lines) if l.closed]
        for i, line in closed:
            if line.has_point(point):
                return i
        # если нет, то первая попавшаяся линия
        for i, line in enumerate(self.lines):
            if line.has_point(point):
                return i
        return None

    def delete(self):
        # если нет активных точек ничего не делаем
        if self.mouse_press_index is None:
            return
        to_delete = self.mouse_press_index
        self.mouse_press_index = None
        # определяем точку для удаления
        point = self.control_points[to_delete]
        # убираем из линий
        self.delete_lines(point)
        # убираем из рёбер
        for i in self.edges[to_delete]:
            self.edges[i].remove(to_delete)
        self.edges.pop(to_delete)
        for i, edges in enumerate(self.edges):
            self.edges[i] = set([v - 1 * (v > to_delete) for v in edges])
        # убираем из списка точек
        self.control_points.pop(to_delete)

    def delete_lines(self, point):
        pi = self.get_line_index(point)
        # если не принадлежит линиям
        if pi is None:
            return
        # удаление линии если
        # - линия из двух точек
        # - линия из трёх точек, и удаляется средняя
        predicate = self.lines[pi].length == 2
        predicate |= self.lines[pi].length == 3 and \
                     not self.lines[pi].is_bound_point(point)
        if predicate:
            self.mouse_motion_index = None
            self.lines.pop(pi)
            return
        # сокращение или разделение незамкнутой линии
        if not self.lines[pi].closed:
            line = self.lines[pi].delete(point)
            if line is not None:
                self.lines.append(line)
            return
        # массив линий для размыкания
        to_disclose = [self.lines[pi]]
        while len(to_disclose) > 0:
            disc = to_disclose.pop(0)
            # ищем линии, которые замкнуты данной линией
            for l in self.lines:
                if disc in l.closed_with:
                    l.disclose_with(disc)
                    to_disclose.append(l)
        line = self.lines[pi].delete(point)
        if line is not None:
            self.lines.append(line)

    def mouse_motion(self, point):
        self.mouse_motion_index = self.get_line_index(point, with_near=True)

    def mouse_press(self, point):
        i = self.get_point_index(point)
        # нет активной точки
        if self.mouse_press_index is None:
            # попали в существующую точку - активируем
            if i is not None:
                self.mouse_press_index = i
            # не попали в точку - создаём и активируем
            else:
                self.control_points.append(point)
                self.edges.append(set())
                self.mouse_press_index = self.points_len - 1
            return
        # попали в активную точку - отключаем
        if i == self.mouse_press_index:
            self.mouse_press_index = None
            return
        # активная точка и её линия
        active = self.control_points[self.mouse_press_index]
        ai = self.get_line_index(active)
        # не попали в существующую точку - создаём новую
        if i is None:
            self.control_points.append(point)
            self.edges.append(set())
            i = self.points_len - 1
        # точки уже соединены
        if self.mouse_press_index in self.edges[i]:
            self.mouse_press_index = i
            return
        # уточняем point и его линию
        point = self.control_points[i]
        pi = self.get_line_index(point)
        # добавляем рёбра
        self.edges[self.mouse_press_index].add(i)
        self.edges[i].add(self.mouse_press_index)
        # активируем point
        self.mouse_press_index = i
        # строем линии
        self.build_lines(ai, active, pi, point)

    def build_lines(self, ai, active, pi, point):
        """
        Функция построения линий для текущей конфигурации Boundary
        :param ai: либо индекс линии, которой принадлежит active,
                   либо None если active не имеет рёбер
        :param active: активная точка
        :param pi: либо индекс линии, которой принадлежит point,
                   либо None если point не имеет рёбер
        :param point: точка, к которой перейдёт активность
        """
        # просто переключаем точку если
        # - хотя бы одна точка не является крайней для своей незамкнутой линии при том,
        #   что линии разные (2 случая)
        predicate = ai is not None and ai != pi and \
                    not self.lines[ai].closed and \
                    not self.lines[ai].is_bound_point(active)
        predicate |= pi is not None and ai != pi and \
                     not self.lines[pi].closed and \
                     not self.lines[pi].is_bound_point(point)
        if predicate:
            return
        # новая линия если
        # - точки не имеют линий
        # - обе точки лежат на замкнутых линиях (могут на одной, а могут на разных
        predicate = ai is None and pi is None
        predicate |= ai is not None and pi is not None and \
                     self.lines[ai].closed and self.lines[pi].closed
        if predicate:
            self.lines.append(Line(active, point))
            return
        # новая линия с частичным замыканием если
        # - одна точка лежит на замкнутой линии, а другая не имеет линии (2 случая)
        predicate = ai is None and pi is not None and self.lines[pi].closed
        predicate |= pi is None and ai is not None and self.lines[ai].closed
        if predicate:
            self.lines.append(Line(active, point))
            if ai is not None:
                self.lines[-1].closed_with[0] = self.lines[ai]
            else:
                self.lines[-1].closed_with[1] = self.lines[pi]
        # продолжение линии если
        # - одна из точек является крайней для своей линии, а вторая не имеет линии (2 случая)
        predicate = pi is None and ai is not None and self.lines[ai].is_bound_point(active)
        predicate |= ai is None and pi is not None and self.lines[pi].is_bound_point(point)
        if predicate:
            if pi is None:
                self.lines[ai].add(point, active)
            else:
                self.lines[pi].add(active, point)
            return
        # продолжение линии с частичным замыканием если
        # - одна из точек является крайней для своей линии, а вторая лежит на замкнутой (2 случая)
        predicate = pi is not None and ai is not None and \
                    self.lines[pi].closed and self.lines[ai].is_bound_point(active)
        predicate |= ai is not None and pi is not None and \
                     self.lines[ai].closed and self.lines[pi].is_bound_point(point)
        if predicate:
            if self.lines[pi].closed:
                self.lines[ai].add(point, active, self.lines[pi])
            else:
                self.lines[pi].add(active, point, self.lines[ai])
            return
        # соединение линий если
        # - обе точки лежат на краях разных линий
        predicate = ai is not None and pi is not None and ai != pi and \
                    self.lines[ai].is_bound_point(active) and \
                    self.lines[pi].is_bound_point(point)
        if predicate:
            self.lines[ai].connect_line(self.lines.pop(pi), active, point)
            return
        # замыкание линии если
        # - обе точки являются краями одной линии
        predicate = ai is not None and pi is not None and ai == pi and \
                    self.lines[ai].is_bound_point(active) and \
                    self.lines[pi].is_bound_point(point)
        if predicate:
            self.lines[ai].close()
            return
        # замыкание линии и создание линии от остатка если
        # - обе точки на одной линии и только одна из них крайняя (2 случая)
        predicate = ai is not None and ai == pi and self.lines[ai].is_bound_point(active)
        predicate |= pi is not None and pi == ai and self.lines[pi].is_bound_point(point)
        if predicate:
            line = self.lines[ai].close(active, point)
            self.lines.append(line)
            return

    def mouse_drag(self, x, y, dx, dy):
        if self.mouse_press_index is None:
            self.mouse_press_index = self.get_point_index(Point(x - dx, y - dy))
        if self.mouse_press_index is None:
            return
        self.control_points[self.mouse_press_index].x = x
        self.control_points[self.mouse_press_index].y = y
        for line in self.lines:
            line.recount_points()
        self.changed = True

    def clear(self):
        # массив контрольных точек
        self.control_points = []
        # массив линий
        self.lines = []
        # матрица индексов связанных точек
        self.edges = []
        # индексы точек для покраски
        self._mouse_motion_index = None
        self._mouse_press_index = None
        self.changed = True
