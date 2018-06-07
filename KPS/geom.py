import numpy as np
from pyglet.gl import glPointSize
from shapely.geometry import asPoint, asLineString, asPolygon


class Point:
    radius = 10
    default_color = (1, 1, 0, 1)

    def __init__(self, x, y, z=0, color=default_color):
        self.p = np.array([x, y, z])
        self.geom = asPoint(self.p)
        self.color = color

    @property
    def data(self):
        return self.p

    @property
    def x(self):
        return self.p[0]

    @x.setter
    def x(self, v):
        self.p[0] = v

    @property
    def y(self):
        return self.p[1]

    @property
    def xy(self):
        return self.p[:2]

    @y.setter
    def y(self, v):
        self.p[1] = v

    @property
    def z(self):
        return self.p[2]

    @z.setter
    def z(self, v):
        self.p[2] = v

    def is_near(self, p):
        return self.geom.buffer(self.radius).intersects(
                p.geom.buffer(self.radius))

    @classmethod
    def set_radius(cls, value=10):
        if value < 5:
            value = 5
        if value > 10:
            value = 10
        glPointSize(value * 2)
        cls.radius = value

    def __eq__(self, other):
        return (self.p == other.p).all()

    def __gt__(self, other):
        if self.x != other.x:
            return self.x > other.x
        if self.y != other.y:
            return self.y > other.y
        if self.z != other.z:
            return self.z > other.z
        return False

    def __add__(self, other):
        return Point(*(self.p + other.p))

    def __sub__(self, other):
        return Point(*(self.p - other.p))

    def __repr__(self):
        return str(self.data)


class Line:
    default_color = (0.6, 0.6, 0.6, 1)

    def __init__(self, p0, p1, *points):
        self.control_points = [p0, p1] + list(points)
        self.color = self.default_color
        self.points = None
        self.geom = None
        self.mesh = None
        self.closed = False
        self.closed_with = [None, None]
        self.recount_points()

    length = property(lambda self: len(self.control_points))

    def is_bound_point(self, p):
        if self.closed:
            return False
        if self.control_points[0].is_near(p):
            return True
        if self.control_points[-1].is_near(p):
            return True
        return False

    def has_point(self, p):
        for i in range(self.length):
            if self.control_points[i].is_near(p):
                return True
        return False

    def are_neighbours(self, p0, p1):
        i0 = self.control_points.index(p0)
        i1 = self.control_points.index(p1)
        if abs(i1 - i0) == 1:
            return True
        if i0 == 0 and i1 == self.length - 1 or i1 == 0 and i0 == self.length - 1:
            return True
        return False

    def add(self, p, to, closed_with_line=None):
        if to == self.control_points[0]:
            if p == self.control_points[-1]:
                return
            self.control_points = [p] + self.control_points
            if closed_with_line is not None:
                self.closed_with[0] = closed_with_line
        elif to == self.control_points[-1]:
            if p == self.control_points[0]:
                return
            self.control_points.append(p)
            if closed_with_line is not None:
                self.closed_with[1] = closed_with_line
        if None not in self.closed_with:
            self.closed = True
        self.recount_points()

    def delete(self, p):
        i = self.control_points.index(p)
        new_line = None
        # если линия замкнута сама по себе
        if self.closed and self.closed_with == [None, None]:
            if i == 0 or i == self.length - 1:
                self.control_points.pop(i)
            else:
                self.control_points = self.control_points[i+1:] + \
                                      self.control_points[:i]
            self.closed = False
            self.recount_points()
            return None
        # необходимо разомкнуть
        if i < 2 and self.closed_with[0] is not None:
            self.closed_with[0] = None
            self.closed = False
        elif i > self.length - 3 and self.closed_with[-1] is not None:
            self.closed_with[-1] = None
            self.closed = False
        # удаляется 2 точки
        if i == 1:
            self.control_points = self.control_points[i+1:]
        elif i == self.length - 2:
            self.control_points = self.control_points[:i]
        # удаляется только одна точка
        if i == 0 or i == self.length - 1:
            self.control_points.pop(i)
        # если удаляется в середине
        if 1 < i < self.length - 2:
            # если есть замыкание, то оставляем с замыканием старую
            if self.closed_with[0] is not None:
                new_line = Line(*self.control_points[i+1:])
                new_line.closed_with[-1] = self.closed_with[-1]
                self.control_points = self.control_points[:i]
            elif self.closed_with[-1] is not None:
                new_line = Line(*self.control_points[:i])
                new_line.closed_with[0] = self.closed_with[0]
                self.control_points = self.control_points[i+1:]
        self.recount_points()
        return new_line

    def connect_line(self, line, this, that):
        # развернём линии так, чтобы дл своих линий this было последней точкой,
        # а that первой
        if self.control_points[0] == this:
            self.control_points = self.control_points[::-1]
        if line.control_points[-1] == that:
            line.control_points = line.control_points[::-1]
        self.control_points += line.control_points
        self.recount_points()

    def close(self, p0=None, p1=None):
        # замыкаем крайние точки
        if p0 is None and p1 is None:
            self.closed = True
            self.recount_points()
            return
        # меняем p0 и p1 с точек на индексы
        p0 = self.control_points.index(p0)
        p1 = self.control_points.index(p1)
        # пусть крайний индекс лежит в po
        if p1 in (0, self.length - 1):
            p0, p1 = p1, p0
        # делим линии
        if p0 == 0:
            new_line = Line(*self.control_points[p1:])
            self.control_points = self.control_points[:p1+1]
        else:  # p0 == self.length - 1
            new_line = Line(*self.control_points[:p1+1])
            self.control_points = self.control_points[p1:]
        self.closed = True
        self.recount_points()
        return new_line

    def disclose_with(self, line):
        if self.closed_with[0] == line:
            self.delete(self.control_points[0])
        if self.closed_with[-1] == line:
            self.delete(self.control_points[-1])

    def recount_points(self, step=0.1):
        """
        Функция расчёта точек сплайна для линии
        type(self) == Line
        """
        self.points = []
        # если линия замкнутая
        if self.closed:
            # для каждой точки pi
            for i in range(self.length):
                # строим точки [pi, pi+1) и последовательно добавляем их в points
                self.points.extend(
                        [catmul_rom4(s,
                                     self.control_points[i - 1],
                                     self.control_points[i],
                                     self.control_points[(i + 1) % self.length],
                                     self.control_points[(i + 2) % self.length])
                         for s in np.arange(0, 1, step)])
        # если линия из одного ребра, ничего достраивать не нужно
        elif self.length == 2:
            self.points = self.control_points[:]
        # если линия не замкнута и содержит больше двух рёбер
        else:
            # строим [p0, p1) с производной, совпадающей с ребром
            self.points.extend([catmul_rom3(s,
                                            *self.control_points[:3])
                                for s in np.arange(0, 1, step)])
            # строим [p1, p-2)
            for i in range(1, self.length - 2):
                self.points.extend([catmul_rom4(s,
                                                *self.control_points[i-1:i+3])
                                    for s in np.arange(0, 1, step)])
            # строим [p-1, p-2] с производной, совпадающей с ребром
            # разворачиваем при добавлении в points
            self.points.extend([catmul_rom3(s,
                                            *self.control_points[:-4:-1])
                                for s in np.arange(0, 1 + step, step)][::-1])
        self.recount_geom()

    def recount_geom(self):
        # если не замкнута, просто LineString
        if not self.closed:
            self.geom = asLineString([p.data for p in self.points])
            return
        self.geom = asPolygon([p.data for p in self.points])
        # если замкнута с другими линиями, нужно вычесть полигоны
        if None not in self.closed_with:
            self.geom -= self.closed_with[0].geom.union(self.closed_with[1].geom)
            self.points = [Point(*p) for p in np.array(self.geom.boundary)]
        # self.mesh = Mesh(self.geom)
        # self.mesh.update_points()

    def is_near(self, p):
        line_string = self.geom.boundary if self.closed else self.geom
        return line_string.distance(asPoint(p.xy)) < 5

    def __repr__(self):
        return str(self.geom)


class MeshPoint(Point):
    def __init__(self, x, y, z, index, up=None, right=None, left=None, down=None,
                 is_boundary=False):
        super(MeshPoint, self).__init__(x, y, z)
        self.up = up
        self.right = right
        self.left = left
        self.down = down
        self.is_boundary = is_boundary
        self.index = index

    def update(self):
        """
        Функция для обновления координаты z расчётом взвешенного среднего
        """
        if self.is_boundary:
            return
        # числитель
        zs = 0
        # знаменатель
        ds = 0
        for p in (self.up, self.right, self.left, self.down):
            # Эвклидово расстояние между точками
            d = self.distance_with(p)
            # чем ближе, тем больше вес
            # поэтому обратнопропорционально расстоянию
            zs += p.z / d
            ds += 1 / d
        self.z = zs / ds

    def distance_with(self, p):
        """
        :param p: точка
        :return: Эвклидово расстояние в (x,y)
        """
        vec = self.xy - p.xy
        return np.sqrt(vec.dot(vec))

    def set_up(self, p):
        # чтобы не уйти в цикл
        if self.up == p:
            return
        self.up = p
        p.set_down(self)

    def set_right(self, p):
        # чтобы не уйти в цикл
        if self.right == p:
            return
        self.right = p
        p.set_left(self)

    def set_left(self, p):
        # чтобы не уйти в цикл
        if self.left == p:
            return
        self.left = p
        p.set_right(self)

    def set_down(self, p):
        # чтобы не уйти в цикл
        if self.down == p:
            return
        self.down = p
        p.set_up(self)

    def has_triangles(self):
        return self.up is not None and self.left is not None or \
               self.down is not None and self.right is not None

    def triangles(self):
        res = []
        # верхний левый треугольник
        if self.up is not None and self.left is not None:
            res.append([self.index, self.up.index, self.left.index])
        # нижний правый треугольник
        if self.down is not None and self.right is not None:
            res.append([self.index, self.down.index, self.right.index])
        # если точка в нижнем левом углу,
        # добавляем нижний левый треугольник
        predicate = self.down is not None and not self.down.has_triangles()
        predicate &= self.left is not None and not self.left.has_triangles()
        if predicate:
            res.append([self.index, self.down.index, self.left.index])
        # если точка в верхнем правом углу,
        # добавляем верхний правый треугольник
        predicate = self.up is not None and not self.up.has_triangles()
        predicate &= self.right is not None and not self.right.has_triangles()
        if predicate:
            res.append([self.index, self.up.index, self.right.index])
        return res


class Mesh:
    def __init__(self, geom):
        self.geom = geom
        # границы меша
        self.min_x = min(self.geom.boundary.coords, key=lambda v: v[0])[0]
        self.min_y = min(self.geom.boundary.coords, key=lambda v: v[1])[1]
        self.max_x = max(self.geom.boundary.coords, key=lambda v: v[0])[0]
        self.max_y = max(self.geom.boundary.coords, key=lambda v: v[1])[1]
        # расстояния между вершинами меша
        self.dx = (self.max_x - self.min_x) / 100
        self.dy = (self.max_x - self.min_x) / 100
        # среднее значение z каркаса
        self.mean_z = np.mean([p[2] for p in self.geom.boundary.coords])
        # все MeshPoints меша
        self.points = []
        self.init_mesh_points()
        # массив треугольников
        # элемент - тройка индексов вершин
        self.triangles = []
        self.init_triangles()
        # площадь вершинами меша
        self.area = 0
        self.count_area()

    def init_mesh_points(self):
        # ищем точку в пределах границ
        for x in np.arange(self.min_x, self.max_x, self.dx):
            for y in np.arange(self.min_y, self.max_y, self.dy):
                # если точка лежит внутри geom
                if self.geom.contains(asPoint([x, y])):
                    # инициализируем точку и остальные от неё
                    self.init_mesh_point(x, y)
                    return

    def init_mesh_point(self, x, y):
        # ищем точку в созданных или создаём
        f = list(filter(lambda p: (p.xy == [x, y]).all(), self.points))
        if len(f) == 0:
            point = MeshPoint(x, y, self.mean_z, index=len(self.points))
            self.points.append(point)
        else:
            point = f[0]
        # точка сверху
        if point.up is None:
            # если соседняя точка лежит вне области
            if not self.geom.contains(asPoint([x, y + self.dy])):
                # перессекаем boundary с вертикальной прямой
                xyz = np.array(self.geom.boundary.intersection(
                        asLineString([[x, y], [x, self.max_y]])))
                # если xyz содержит несколько точек, берём ближнюю
                if xyz.shape != (3,):
                    xyz = sorted(xyz, key=lambda v: v[1])[0]
                # создаём точку
                up = MeshPoint(*xyz, index=len(self.points), is_boundary=True)
                self.points.append(up)
            else:
                up = self.init_mesh_point(x, y + self.dy)
            point.set_up(up)
        # точка справа
        if point.right is None:
            # если соседняя точка лежит вне области
            if not self.geom.contains(asPoint([x + self.dx, y])):
                # перессекаем boundary с горизонтальной прямой
                xyz = np.array(self.geom.boundary.intersection(
                        asLineString([[x, y], [self.max_x, y]])))
                # если xyz содержит несколько точек, берём ближнюю
                if xyz.shape != (3,):
                    xyz = sorted(xyz, key=lambda v: v[0])[0]
                # создаём точку
                right = MeshPoint(*xyz, index=len(self.points), is_boundary=True)
                self.points.append(right)
            else:
                right = self.init_mesh_point(x + self.dx, y)
            point.set_right(right)
        # точка снизу
        if point.down is None:
            # если соседняя точка лежит вне области
            if not self.geom.contains(asPoint([x, y - self.dy])):
                # перессекаем boundary с вертикальной прямой
                xyz = np.array(self.geom.boundary.intersection(
                        asLineString([[x, self.min_y], [x, y]])))
                # если xyz содержит несколько точек, берём ближнюю
                if xyz.shape != (3,):
                    xyz = sorted(xyz, key=lambda v: v[1], reversed=True)[0]
                # создаём точку
                down = MeshPoint(*xyz, index=len(self.points), is_boundary=True)
                self.points.append(down)
            else:
                down = self.init_mesh_point(x, y - self.dy)
            point.set_down(down)
        # точка слева
        if point.left is None:
            # если соседняя точка лежит вне области
            if not self.geom.contains(asPoint([x - self.dx, y])):
                # перессекаем boundary с горизонтальной прямой
                xyz = np.array(self.geom.boundary.intersection(
                        asLineString([[self.min_x, y], [x, y]])))
                # если xyz содержит несколько точек, берём ближнюю
                if xyz.shape != (3,):
                    xyz = sorted(xyz, key=lambda v: v[0], reversed=True)[0]
                # создаём точку
                left = MeshPoint(*xyz, index=len(self.points), is_boundary=True)
                self.points.append(left)
            else:
                left = self.init_mesh_point(x - self.dx, y)
            point.set_left(left)
        return point

    def init_triangles(self, mp=None, made=[]):
        """
        Рекурсивная функция для инициализации треугольников по вершинам меша
        :param mp: текущая точка
        :param made: индексы учтённых точек
        """
        if mp is None:
            mp = self.points[0]
        # если точка уже учтена
        if mp.index in made:
            return
        # получение треугольников для точки
        for tr in mp.triangles():
            self.triangles.append(tr)
        made.append(mp.index)
        # рекурсивно отправляем в соседние точки
        if mp.up is not None:
            self.init_triangles(mp.up, made)
        if mp.left is not None:
            self.init_triangles(mp.left, made)
        if mp.down is not None:
            self.init_triangles(mp.down, made)
        if mp.rigth is not None:
            self.init_triangles(mp.right, made)

    def update_points(self, eps=1):
        previous = self.area
        while True:
            for p in self.points:
                p.update()
            self.count_area()
            if abs(self.area - previous) < eps:
                break
            previous = self.area

    def triangle_area(self, mps):
        # 2 вектора
        v1 = self.points[mps[1]].data - self.points[mps[0]].data
        v2 = self.points[mps[2]].data - self.points[mps[0]].data
        # векторное произведение
        cross = np.cross(v1, v2)
        # площадь = половина длины векторного произведения
        return np.sqrt(cross.dot(cross)) / 2

    def count_area(self):
        # self.area = sum(list(map(self.triangle_area, self.triangles)))
        self.area = 0
        for mps in self.triangles:
            self.area += self.triangle_area(mps)


def catmul_rom3(s, p0, p1, p2, tau=0.5):
    """
    Функция расчёта координат точки по трём опорным точкам
    """
    S = np.array([s ** i for i in range(0, 4)])
    M = np.array([[1, 0, 0],
                  [-tau, tau, 0],
                  [3 * tau - 3, 3 - 2 * tau, -tau],
                  [2 - 2 * tau, tau - 2, tau]])
    # p.data == [p.x, p.y, p.z]
    P = np.array([p0.data, p1.data, p2.data])
    return Point(*S.dot(M).dot(P))


def catmul_rom4(s, p0, p1, p2, p3, tau=0.5):
    """
    Функция расчёта координат точки по четырём опорным точкам
    """
    S = np.array([s ** i for i in range(0, 4)])
    M = np.array([[0, 1, 0, 0],
                  [-tau, 0, tau, 0],
                  [2 * tau, tau - 3, 3 - 2 * tau, -tau],
                  [-tau, 2 - tau, tau - 2, tau]])
    # p.data == [p.x, p.y, p.z]
    P = np.array([p0.data, p1.data, p2.data, p3.data])
    return Point(*S.dot(M).dot(P))

