import pygame


class Constraint:
    def pre_solve(self, dt):
        pass

    def solve(self):
        pass


class RopeConstraint(Constraint):
    def __init__(self, a, b, length, break_threshold=None):
        self.a = a
        self.b = b
        self.length = length
        self.break_threshold = break_threshold
        self.broken = False

    def solve(self):
        if self.broken:
            return

        delta = self.b.pos - self.a.pos
        dist = delta.length()
        if dist <= self.length:
            return

        stretch = dist - self.length

        # break condition
        if self.break_threshold is not None and stretch > self.break_threshold:
            self.broken = True
            return

        if dist == 0:
            return

        n = delta * (1 / dist)

        inv_mass_sum = self.a.inv_mass + self.b.inv_mass
        if inv_mass_sum == 0:
            return

        corr = stretch
        self.a.pos += n * corr * (self.a.inv_mass / inv_mass_sum)
        self.b.pos -= n * corr * (self.b.inv_mass / inv_mass_sum)

    def draw(self, screen, world_to_screen):
        if self.broken:
            return

        p1 = world_to_screen(self.a.pos)
        p2 = world_to_screen(self.b.pos)

        pygame.draw.line(screen, (255, 0, 0), p1, p2, 2)


class DistanceJoint(Constraint):
    def __init__(self, a, b, length, stiffness=1.0):
        self.a = a
        self.b = b
        self.length = length
        self.stiffness = stiffness

    def solve(self):
        delta = self.b.pos - self.a.pos
        dist = delta.length()
        if dist == 0:
            return

        error = dist - self.length
        n = delta * (1 / dist)

        inv_mass_sum = self.a.inv_mass + self.b.inv_mass
        if inv_mass_sum == 0:
            return

        correction = n * (error * self.stiffness / inv_mass_sum)
        self.a.pos += correction * self.a.inv_mass
        self.b.pos -= correction * self.b.inv_mass

    def draw(self, screen, world_to_screen):
        p1 = world_to_screen(self.a.pos)
        p2 = world_to_screen(self.b.pos)

        pygame.draw.line(screen, (255, 200, 50), p1, p2, 2)


class Spring:
    def __init__(self, a, b, k, c, rest):
        self.a = a
        self.b = b
        self.k = k
        self.c = c
        self.rest = rest

    def apply(self):
        d = self.b.pos - self.a.pos
        l = d.length()
        if l == 0:
            return

        n = d.normalized()
        vrel = (self.b.vel - self.a.vel).dot(n)

        f = -self.k * (l - self.rest) - self.c * vrel
        force = n * f

        self.a.apply_force(-force)
        self.b.apply_force(force)

    def draw_spring(self, screen, world_to_screen, spring, color=(200, 200, 200), width=2):
        p0 = world_to_screen(spring.a.pos)
        p1 = world_to_screen(spring.b.pos)

        pygame.draw.line(screen, color, p0, p1, width)
