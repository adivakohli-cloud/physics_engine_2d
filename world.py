import math

from vector import Vec2
from collision import (
    resolve_ground_contact,
    resolve_circle_circle,
    resolve_box_box,
    resolve_box_ground_contact
)


class World:
    def __init__(self):
        self.bodies = []
        self.constraints = []
        self.springs = []
        self.gravity = Vec2(0, -9.81)
        self.iterations = 10  # Increased for stability
        self.substeps = 8  # Increased for better precision

    def step(self, dt):
        dt_sub = dt / self.substeps

        for _ in range(self.substeps):
            #  APPLY FORCES
            for b in self.bodies:
                if b.inv_mass == 0:
                    continue
                # Apply gravity
                b.apply_force(self.gravity * b.mass)

            # Apply springs
            for s in self.springs:
                s.apply()

            #  INTEGRATE VELOCITY & POSITION
            # CRITICAL FIX: This now calls the Body's integrate method
            # so that damping (air resistance/rolling friction) is applied.
            for b in self.bodies:
                b.integrate(dt_sub)

            #  COLLISION SOLVER (ITERATIVE)
            for _ in range(self.iterations):
                MAX_ANG_VEL = 50

                # a) Ground contacts (Ground fixed at y = -3.0)
                for b in self.bodies:
                    if b.shape == "circle":
                        resolve_ground_contact(b, ground_y=-3.0)
                        # Clamp angular velocity to prevent explosion
                        b.ang_vel = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, b.ang_vel))

                    elif b.shape == "box":
                        for _ in range(1):
                            resolve_box_ground_contact(b, ground_y=-3.0, restitution=0.2, mu=0.8)

                # b) Body-body collisions
                n = len(self.bodies)
                for i in range(n):
                    for j in range(i + 1, n):
                        a = self.bodies[i]
                        b = self.bodies[j]

                        if a.shape == "circle" and b.shape == "circle":
                            resolve_circle_circle(a, b)
                        elif a.shape == "box" and b.shape == "box":
                            resolve_box_box(a, b)

                # c) Solve constraints
                for c in self.constraints:
                    c.solve()

            #  Remove broken constraints

            self.constraints = [c for c in self.constraints if not hasattr(c, "broken") or not c.broken]
