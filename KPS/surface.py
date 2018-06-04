import numpy as np
from pyglet.gl import *
from pyglet import graphics as gr

from geom import Point


class Surface:
    def __init__(self, boundary=[]):
        # точки на каркасе
        self.boundary = boundary
        self.enabled = None
        self.closed = False
        # точки на поверхности
        self.points = None
        # self.subsurfaces = [self]

    @property
    def boundary_len(self):
        return len(self.boundary)

    @property
    def boundary_data(self):
        return sum((list(p.data) for p in self.boundary), [])

    def draw(self, three_d=False):

        # glMatrixMode(GL_MODELVIEW)
        # glLoadIdentity()
        # gluPerspective(70, self.width/self.height, 0.05, 1000)
        # glMatrixMode(GL_PROJECTION)
        # glLoadIdentity()
        point_colors = sum((self.boundary[i].color(self.enabled == i) for i in range(self.boundary_len)), ())
        gr.draw(self.boundary_len, GL_POINTS,
                ('v3f', self.boundary_data),
                ('c3f', point_colors))

        # self.count_bezier()
        # data = sum((list(p.data) for p in self.bezier), [])
        # gr.draw(len(self.bezier), GL_LINE_STRIP,
        #         ('v3f', data),
        #         ('c3f', (0, 0, 1) * len(self.bezier)))

    # def count_bezier(self, step=0.01):
    #     if len(self.bezier) > 0:
    #         return
    #     if self.boundary_len < 2:
    #         return
    #     for i in range(0, self.boundary_len, 3):
    #         ps = self.boundary[i:i+4]
    #         if i + 4 >= self.boundary_len and self.closed:
    #             ps.append(self.boundary[0])
    #         # матрица координат
    #         ps_m = np.array([p.data for p in ps])
    #         b_m = self.bezier_basis(len(ps))
    #         # находим результат произведения матрицы координат на базисную матрицу кривой Безье
    #         temp = np.dot(ps_m.T, b_m)
    #         # считаем точки
    #         cur = ps[0]
    #         self.bezier.append(cur)
    #         for t in np.arange(step, 1, step):
    #             # следующая точка
    #             [[x], [y], [z]] = np.dot(temp, self.pows(len(ps), t))
    #             self.bezier.append(Point(x, y, z))
    #         self.bezier.append(ps[-1])

    # def pows(self, n, t):
    #     m = np.empty([n, 1])
    #     val = 1.0
    #     for i in range(n):
    #         m[i, 0] = val
    #         val *= t
    #     return m
    #
    # def bezier_basis(self, n):
    #     C = lambda N, K: 1 if (N == 0 or K % N == 0) else (C(N - 1, K) + C(N - 1, K - 1))
    #     m = np.empty([n, n])
    #     for i in range(n):
    #         for j in range(n):
    #             m[i, j] = 0 if j < i else pow(-1, j - i) * C(n - 1, j) * C(j, i)
    #     return m

    def add_point(self, p):
        # если таскаем точки, добавлять не нужно
        if self.enabled is not None:
            return
        # замкнули, больше не добавляем
        if self.closed:
            return
        # иногда нужно добавлять дополнительныые точки
        if (self.boundary_len - 1) % 3 == 0 and self.boundary_len > 1:
            p3 = self.boundary[-1]
            p2 = self.boundary[-2]
            self.boundary.append(p3 + p3 - p2)
        # добавляем текущую точку
        self.boundary.append(p)
        self.bezier = []

    def enable_point(self, p):
        for i in range(self.boundary_len):
            if self.boundary[i].is_near(p):
                self.enabled = i
                break
        self.bezier = []

    def disable_point(self):
        self.enabled = None
        self.bezier = []

    def move_enabled(self, x, y, dx, dy):
        if self.enabled is None:
            return
        new = Point(x, y)
        e = self.enabled
        # если меняется касающаяся
        if e % 3 == 0:
            # крайние только при замкнутости
            if 1 < self.enabled < self.boundary_len - 1 or self.closed:
                # обновляем следующую за ней
                self.boundary[e + 1] = new + new - self.boundary[e - 1]
        # если меняется следующа за касающейся (но не в первой части)
        elif (e - 1) % 3 == 0:
            # первая только при замкнутости
            if e > 3 or self.closed:
                # обновляем ту, что перед касающейся
                self.boundary[e - 2] = self.boundary[e - 1] + self.boundary[e - 1] - new
        # если меняется предшествующая касающейся
        elif (e + 1) % 3 == 0:
            # последняя только при замыкании
            if e < self.boundary_len - 1 or self.closed:
                # обновляем ту, что после касающейся
                self.boundary[(e + 2) % self.boundary_len] = self.boundary[(e + 1) % self.boundary_len] + \
                                                             self.boundary[(e + 1) % self.boundary_len] - new
        self.boundary[e] = new
        self.bezier = []

    # def close(self):
    #     self.bezier = []
    #     if (self.boundary_len - 1) % 3 == 0:
    #         # продолжаем конец
    #         self.boundary.append(self.boundary[-1] + self.boundary[-1] - self.boundary[-2])
    #         # продолжаем начало
    #         self.boundary.append(self.boundary[0] + self.boundary[0] - self.boundary[1])
    #         self.closed = True

    def divide(self):
        pass

    def count_area(self):
        return 0
