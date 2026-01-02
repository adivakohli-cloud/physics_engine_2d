import math
import pygame
from geometry import box_vertices
from vector import Vec2

PPM = 100  # pixels per meter
SCREEN_W = 1000
SCREEN_H = 800
GROUND_Y=-3
def to_screen(v):
    return int(v.x * PPM), int(SCREEN_H - v.y * PPM)


def world_to_screen(pos):
    x_screen = int(SCREEN_W / 2 + pos.x * PPM)
    y_screen = int(SCREEN_H / 2 - pos.y * PPM)
    return x_screen, y_screen


class Renderer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
        pygame.display.set_caption("Physics Engine Demo")

    def clear(self):
        self.screen.fill((0, 0, 0))

    def draw_circle(self, body):
        x, y = world_to_screen(body.pos)
        # print("Ball Pos=", (x, y))

        r = int(body.radius * PPM)
        if r <= 0:
            return

        pygame.draw.circle(
            self.screen,
            (200, 200, 255),
            (int(x), int(y)),
            r,
            2
        )

        # orientation line
        end_x = x + int(r * math.cos(body.angle))
        end_y = y - int(r * math.sin(body.angle))

        pygame.draw.line(
            self.screen,
            (255, 50, 50),
            (int(x), int(y)),
            (int(end_x), int(end_y)),
            2
        )

    def draw_box(self, body):
        verts = box_vertices(body)
        pts = [world_to_screen(v) for v in verts]

        pygame.draw.polygon(self.screen,(200, 200, 255),pts,2)

        # orientation axis
        center = world_to_screen(body.pos)
        axis_len = body.width * 0.5 * PPM

        end_x = center[0] + int(axis_len * math.cos(body.angle))
        end_y = center[1] - int(axis_len * math.sin(body.angle))

        pygame.draw.line(self.screen,(255, 50, 50),center,(end_x, end_y),2)

    def draw_body(self, body):
        if body.shape == "circle":
            self.draw_circle(body)
        elif body.shape == "box":
            self.draw_box(body)

    def draw_rope(self, a, b):
        ax, ay = world_to_screen(a.pos)
        bx, by = world_to_screen(b.pos)
        pygame.draw.line(self.screen,(200, 200, 200),(ax, ay),(bx, by),2)

    def draw_ground(self, ground_y=GROUND_Y):
        visual_ground = ground_y  # <-- your ball radius

        p1 = world_to_screen(Vec2(-20, visual_ground))
        p2 = world_to_screen(Vec2(20, visual_ground))

        pygame.draw.line(self.screen, (0, 255, 0), p1, p2, 3)

    def present(self):
        # draw world origin crosshair
        pygame.draw.line(self.screen, (255, 255, 0),
                         (SCREEN_W // 2 - 10, SCREEN_H // 2),
                         (SCREEN_W // 2 + 10, SCREEN_H // 2), 2)

        pygame.draw.line(self.screen, (255, 255, 0),
                         (SCREEN_W // 2, SCREEN_H // 2 - 10),
                         (SCREEN_W // 2, SCREEN_H // 2 + 10), 2)

        pygame.display.flip()
