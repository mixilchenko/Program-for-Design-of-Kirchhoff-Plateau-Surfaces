from datetime import datetime
from random import randint
import pyglet
from pyglet.gl import *
from pyglet.window import key, mouse

from geom import Point
from boundary import Boundary
from container import Container, Container2D, Container3D


class App(pyglet.window.Window):
    """
    https://pyglet.readthedocs.io/en/pyglet-1.3-maintenance/
    """

    def __init__(self, *args, **kwargs):
        super(App, self).__init__(caption='Kirchhoff-Plateau Surfaces',
                                  resizable=True, width=950, height=480,
                                  *args, **kwargs)

        self.set_minimum_size(950, 480)
        # инициализируются в on_resize
        self.container2D, self.container3D = None, None
        # создаём Boundary
        self.boundary = Boundary()
        # для рисования
        Point.set_radius(5)
        glEnable(GL_POINT_SMOOTH)
        glEnable(GL_LINE_SMOOTH)
        glEnable(GL_DEPTH_TEST)
        # чёрный фон
        glClearColor(0., 0., 0., 1)

    def init_containers(self, width, height):
        # 2 for retina
        coef = self.get_viewport_size()[0] // width
        # 2D
        if self.container2D is None:
            self.container2D = Container2D((width, height), coef,
                                           (1, 0, 0, 1))
        else:
            self.container2D.set_window_size((width, height))
            self.container2D.set_coef(coef)
        # 3D
        if self.container3D is None:
            self.container3D = Container3D((width, height), coef,
                                           (0, 1, 0, 1))
        else:
            self.container3D.set_window_size((width, height))
            self.container3D.set_coef(coef)
        self.boundary.changed = True

    def update_viewport(self):
        w, h = self.get_viewport_size()
        glViewport(0, 0, w, h)

    def on_key_press(self, symbol, modifiers):
        # выход из приложения
        if symbol == key.ESCAPE:
            self.close()
        # удаление точки
        elif symbol == key.DELETE:
            self.boundary.delete()
        # уменьшение точек
        elif symbol == key.DOWN:
            Point.set_radius(Point.radius - 1)
        # увеличение точек
        elif symbol == key.UP:
            Point.set_radius(Point.radius + 1)
        # очищение boundary
        elif symbol == key.C:
            self.boundary.clear()
            self.container3D.clear()
        elif symbol == key.A:
            self.container3D.move_eye(-180, 0)
        elif symbol == key.W:
            self.container3D.move_eye(0, 180)
        elif symbol == key.D:
            self.container3D.move_eye(180, 0)
        elif symbol == key.S:
            self.container3D.move_eye(0, -180)
        elif symbol == key.Q:
            self.container3D.clear()
        elif symbol == key.P:
            print(datetime.now())

    def on_mouse_motion(self, x, y, dx, dy):
        if self.container2D.isin(x, y):
            x, y = self.container2D.recount_xy(x, y)
            self.boundary.mouse_motion(Point(x, y))

    def on_mouse_press(self, x, y, buttons, modifiers):
        # левая для создания точек
        if buttons & mouse.LEFT:
            if self.container2D.isin(x, y):
                x, y = self.container2D.recount_xy(x, y)
                z = randint(0, self.container2D.height_w)
                self.boundary.mouse_press(Point(x, y, z))

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & mouse.LEFT:
            if self.container2D.isin(x, y):
                x, y = self.container2D.recount_xy(x, y)
                self.boundary.mouse_drag(x, y, dx, dy)
            if self.container3D.isin(x, y):
                self.container3D.move_eye(dx, dy)

    def on_resize(self, width, height):
        self.width = height * 2 - Container.margin
        self.update_viewport()
        self.init_containers(self.width, height)

    def on_draw(self):
        self.clear()
        self.container2D.draw(self.boundary)
        self.container3D.draw(self.boundary)
        self.boundary.changed = False
