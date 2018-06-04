import ctypes
import numpy as np
from sklearn.preprocessing import normalize
import pyglet
import glooey
import autoprop
from math import cos, sin, pi
from vecrec import Rect
from pyglet.gl import *
import pyglet.graphics as gr
from pyquaternion import Quaternion

from geom import Point


class Container(glooey.Frame):
    margin = 10

    def __init__(self, window_size, coef, rgba):
        super(Container, self).__init__()

        self.window_size = window_size
        self.coef = coef
        self.rgba = rgba

    def set_window_size(self, window_size):
        self.window_size = window_size

    def set_coef(self, coef):
        self.coef = coef

    # viewport bounds
    left_v = property(lambda self: self.margin)
    bottom_v = property(lambda self: self.margin)
    width_v = property(lambda self: self.window_size[0] * self.coef // 2 - int(1.5 * self.margin))
    height_v = property(lambda self: self.window_size[1] * self.coef - 2 * self.margin)
    right_v = property(lambda self: self.left_v + self.width_v)
    up_v = property(lambda self: self.bottom_v + self.height_v)
    # window bounds
    left_w = property(lambda self: self.margin)
    bottom_w = property(lambda self: self.margin)
    width_w = property(lambda self: self.window_size[0] // 2 - int(1.5 * self.margin))
    height_w = property(lambda self: self.window_size[1] - 2 * self.margin)
    right_w = property(lambda self: self.left_w + self.width_w)
    up_w = property(lambda self: self.bottom_w + self.height_w)

    def isin(self, x, y):
        return (self.left_w <= x <= self.right_w) and (self.bottom_w <= y <= self.up_w)

    def recount_xy(self, x, y):
        x = x - self.left_w
        y = y - self.bottom_w
        return x, y

    def draw(self, boundary):
        glViewport(self.left_v, self.bottom_v, self.width_v, self.height_v)
        gr.draw(4, GL_POLYGON,
                ('v3f', (-1, -1, .99, -1, 1, .99, 1, 1, .99, 1, -1, .99)),
                ('c4f', (0., 0., 0.2, 1) * 4))


class Container2D(Container):
    def __init__(self, position, size, rgba):
        super(Container2D, self).__init__(position, size, rgba)

    def transform(self, points):
        if len(points) < 1:
            return []
        # переделываем в np.array
        points = np.array([p.data for p in points])
        # делаем координаты однородными и транспонируем матрицу
        points = np.append(points, [[1]] * len(points), axis=1).T
        # --- PROJECTION ---
        l, r, b, t, n, f = 0, self.width_w, 0, self.height_w, 0, self.height_w
        P = np.array([[2./(r-l), 0, 0, (r+l)/(l-r)],
                      [0, 2./(t-b), 0, (t+b)/(b-t)],
                      [0, 0, 2./(n-f), (f+n)/(n-f)],
                      [0, 0, 0, 1]])
        points = P.dot(points)
        # превращаем в v2f для draw
        points = sum([list(p[:2]) for p in points.T], [])
        return points

    def draw(self, boundary):
        super(Container2D, self).draw(boundary)

        if boundary.changed:
            self.recount_saved(boundary)

        # draw points
        gr.draw(len(self.ps) // 2, GL_POINTS,
                ('v2f', self.ps),
                ('c4f', self.pcs))

        # draw lines
        for i in range(len(self.ls)):
            gr.draw(len(self.ls[i]) // 2, GL_LINE_LOOP if self.closed[i] else GL_LINE_STRIP,
                    ('v2f', self.ls[i]),
                    ('c4f', self.lcs[i]))

        # draw polygons
        for i in range(len(self.pols)):
            gr.draw(len(self.pols[i]) // 2, GL_POLYGON,
                    ('v2f', self.pols[i]))

        # print('2D')
        # a = (ctypes.c_int   *  4)(); glGetIntegerv(GL_VIEWPORT, a); print(np.asarray(a))
        # a = (ctypes.c_float * 16)(); glGetFloatv(GL_PROJECTION_MATRIX, a); print(np.asarray(a).reshape((4, 4)).T)
        # a = (ctypes.c_float * 16)(); glGetFloatv(GL_MODELVIEW_MATRIX, a); print(np.asarray(a).reshape((4, 4)).T)

    def recount_saved(self, boundary):
        # points
        self.pcs = sum([p.color for p in boundary.control_points], ())
        self.ps = self.transform(boundary.control_points)

        # lines
        self.ls = []
        self.lcs = []
        self.closed = []

        # polygons
        self.pols = []
        for line in boundary.lines:
            self.closed.append(line.closed)
            # add lines
            self.ls.append(self.transform(line.points))
            self.lcs.append(line.color * (len(self.ls[-1]) // 2))

            # add polygons
            if line.closed:
                self.pols.append(self.ls[-1])


class Container3D(Container):
    def __init__(self, position, size, rgba):
        super(Container3D, self).__init__(position, size, rgba)
        self.q = Quaternion()
        self.rotated = False
        self.center = np.array([[self.width_w/2, self.height_w/2, 0.]])

    # viewport bounds
    left_v = property(lambda self: int(0.5 * self.margin) + self.window_size[0] * self.coef // 2)
    # window bounds
    left_w = property(lambda self: int(0.5 * self.margin) + self.window_size[0] // 2)

    def move_eye(self, dx, dy):
        if dx == dy == 0:
            return
        # вектор поворота
        rotation = np.array([dx, dy, 0])
        # векторное произведение - вектор, вокруг которого вращается
        cross = np.cross([0., 0., 1.], rotation)
        # кватернион вращения
        q = Quaternion(axis=cross, degrees=np.sqrt(rotation.dot(rotation)))
        # print(rotation, q.axis, q.degrees, end='\n\n')
        self.q *= q
        self.rotated = True

    def transform(self, points):
        if len(points) < 1:
            return []
        # переделываем в np.array
        points = np.array([p.data for p in points], dtype=np.float)
        # сдвиг центра масс в ноль
        points -= self.center
        # вращение кватернионом
        points = np.apply_along_axis(self.q.rotate, 1, points)
        # делаем координаты однородными и транспонируем матрицу
        points = np.append(points, [[1]] * len(points), axis=1).T
        # --- PROJECTION ---
        l, r, b, t, n, f = -self.width_w/2, self.height_w/2, -self.width_w/2, self.height_w/2, -self.height_w, self.height_w
        P = np.array([[2./(r-l), 0, 0, (r+l)/(l-r)],
                      [0, 2./(t-b), 0, (t+b)/(b-t)],
                      [0, 0, 2./(n-f), (f+n)/(n-f)],
                      [0, 0, 0, 1]])
        points = P.dot(points)
        # превращаем в v3f для draw
        points = sum([list(p[:3]) for p in points.T], [])
        return points

    def draw(self, boundary):
        super(Container3D, self).draw(boundary)

        if boundary.changed or self.rotated:
            self.recount_saved(boundary)

        # draw points
        gr.draw(len(self.ps) // 3, GL_POINTS,
                ('v3f', self.ps),
                ('c4f', self.pcs))

        # draw lines
        for i in range(len(self.ls)):
            gr.draw(len(self.ls[i]) // 3, GL_LINE_LOOP,
                    ('v3f', self.ls[i]),
                    ('c4f', self.lcs[i]))

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, (GLfloat * 4)(0, 0, 1, 1))
        glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.0)
        glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.6)
        glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.7)

        # draw polygons
        for i in range(len(self.pols)):
            gr.draw(len(self.pols[i]) // 3, GL_POLYGON,
                    ('v3f', self.pols[i]))
        glDisable(GL_LIGHT0)
        glDisable(GL_LIGHTING)

    def recount_saved(self, boundary):
        lines = [l for l in boundary.lines if l.closed]
        # points
        points = np.unique(sum([l.control_points for l in lines], []))
        self.recount_center(points)
        self.pcs = sum([p.color for p in points], ())
        self.ps = self.transform(points)

        # lines
        self.ls = []
        self.lcs = []

        # polygons
        self.pols = []
        for line in lines:
            # add lines
            self.ls.append(self.transform(line.points))
            self.lcs.append(line.color * (len(self.ls[-1]) // 3))

            # add polygons
            self.pols.append(self.ls[-1])
        self.rotated = False

    def recount_center(self, points):
        if len(points) == 0:
            self.center = np.array([[self.width_w/2, self.height_w/2, 0.]])
        else:
            # центр в центр масс точек
            self.center = np.array([np.array([p.data for p in points]).mean(axis=0)])

    def clear(self):
        self.q = Quaternion()
        self.rotated = True
        self.center = np.array([[self.width_w/2, self.height_w/2, 0.]])


