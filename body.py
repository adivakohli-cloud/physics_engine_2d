from geometry import box_vertices, box_axes
from vector import Vec2
import math


class Body:
    def __init__(self, pos, mass, vel=None, radius=None, width=None, height=None):
        self.pos = pos
        self.vel = vel if vel else Vec2(0, 0)
        self.mass = mass

        self.angle = 0.0
        self.ang_vel = 0.0
        self.torque = 0.0

        # INCREASE these to stop balls faster
        self.angular_damping = 2.5
        self.linear_damping = 0.2

        self.force = Vec2(0, 0)

        self.radius = radius
        self.width = width
        self.height = height

        self.shape = "circle" if radius is not None else "box"

        if mass <= 0:
            self.inv_mass = 0.0
            self.inv_inertia = 0.0
        else:
            self.inv_mass = 1.0 / mass
            if self.shape == "circle":
                # Circle inertia: 0.5 * m * r^2
                inertia = 0.5 * mass * radius ** 2
            else:
                # Box inertia: m * (w^2 + h^2) / 12
                inertia = (mass * (width ** 2 + height ** 2)) / 12
            self.inv_inertia = 1.0 / inertia

    def apply_force(self, f):
        if self.inv_mass == 0:
            return
        self.force += f

    def apply_torque(self, t):
        self.torque += t

    def integrate(self, dt):
        if self.inv_mass == 0:
            return

        # Linear Integration
        acc = self.force * self.inv_mass
        self.vel += acc * dt

        # Apply Linear Damping (Air resistance)
        self.vel *= max(0.0, 1.0 - self.linear_damping * dt)

        self.pos += self.vel * dt
        self.force = Vec2(0, 0)

        # Angular Integration
        ang_acc = self.torque * self.inv_inertia
        self.ang_vel += ang_acc * dt

        # Apply Angular Damping (Rolling resistance approx)
        self.ang_vel *= max(0.0, 1.0 - self.angular_damping * dt)

        self.angle += self.ang_vel * dt
        self.torque = 0.0


    def get_vertices(self):
        if self.shape != "box":
            return []
        return box_vertices(self)

    def get_axes(self):
        if self.shape != "box":
            return []
        verts = box_vertices(self)
        return box_axes(verts)

    def is_circle(self):
        return hasattr(self, "radius") and self.radius is not None

    def is_box(self):
        return hasattr(self, "width") and hasattr(self, "height")
