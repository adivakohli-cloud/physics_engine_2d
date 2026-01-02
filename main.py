import pygame
from constraints import DistanceJoint, RopeConstraint, Spring
from world import World
from body import Body
from vector import Vec2
from render import Renderer, world_to_screen

world = World()
renderer = Renderer()
clock = pygame.time.Clock()


# Create circular bodies
'''
a = Body(pos=Vec2(-1, 3), radius=0.25, mass=1, vel=Vec2(2,-1))
b = Body(pos=Vec2(-1, 2.1), radius=0.25, mass=1, vel=Vec2(2,0))

world.bodies.extend([a,b])
'''

'''
# Ball 1: Starts on the Left side, slightly below center
ball1 = Body(
    pos=Vec2(-2.0, 2.0),
    radius=0.4,
    mass=2.0,
    vel=Vec2(4.0, -5.0)     # Throw it UP and RIGHT
)

# Ball 2: Starts on the Right side, slightly below center
ball2 = Body(
    pos=Vec2(2.0, 2.0),
    radius=0.4,
    mass=2.0,
    vel=Vec2(-6.0, -5.0)    # Throw it UP and LEFT
)

ball3 = Body(
    pos=Vec2(-0.5, 2.69),
    radius=0.4,
    mass=5.0,
    vel=Vec2(3.0, -4.5)
)
# Add bodies to the world
world.bodies.append(ball1)
world.bodies.append(ball2)
world.bodies.append(ball3)'''

# Box test 1: falling box onto static box
'''
# Static ground box
box1 = Body(
    pos=Vec2(0, -3),
    mass=0,              # static on ground
    width=6,
    height=1
)

# Falling box
box2 = Body(
    pos=Vec2(0, 0),
    mass=1,
    vel=Vec2(0, 0),
    width=1,
    height=1
)

world.bodies.extend([box1, box2])'''

#box-box  impulse test
'''
box1 = Body(
    pos=Vec2(-2, -2),
    mass=1,
    vel=Vec2(7, 4),
    width=1,
    height=1
)

box2 = Body(
    pos=Vec2(1, -2),
    mass=1,
    vel=Vec2(0, 2),
    width=1,
    height=1
)

world.bodies.extend([box1, box2])'''

#offset drop/rotation test
'''
box1 = Body(
    pos=Vec2(0, -3),
    mass=0,
    width=1,
    height=1
)

box2 = Body(
    pos=Vec2(0.8, 0),   # off-center
    mass=1,
    vel=Vec2(0, 0),
    width=1,
    height=1
)

world.bodies.extend([box1, box2])'''

# distance joint test
'''
a = Body(pos=Vec2(-2, 0), width=1, height=1, mass=1) # creating 2 boxes
b = Body(pos=Vec2( 2, 0), width=1, height=1, mass=1)

a.vel = Vec2(2, 1)   
b.vel = Vec2(0, -2)

world.bodies.extend([a,b])
joint = DistanceJoint(a, b, length=4.0, stiffness=1.0)
world.constraints.append(joint)

print("Distance between centres of boxes=",(b.pos - a.pos).length())'''
#rope constraint test
'''
anchor = Body(pos=Vec2(0, 3),width=0.6,height=0.6,mass=0)

b1 = Body(Vec2(0, 2), width=0.6, height=0.6, mass=1)
b2 = Body(Vec2(0, 1), width=0.3,height= 0.3, mass=0.5)
b3 = Body(Vec2(0, 0), width=1, height=1, mass=5)

world.bodies += [anchor, b1, b2, b3]

world.constraints += [RopeConstraint(anchor, b1, 1.0, break_threshold=0.15),RopeConstraint(b1, b2, 1.0, break_threshold=0.15),RopeConstraint(b2, b3, 1.0, break_threshold=0.15)]
'''

# vertical spring demo (damped SHM)
'''
anchor=Body(pos=Vec2(0,3),mass=0,radius=0.1)
box=Body(pos=Vec2(0,1),mass=4,width=1,height=1)
spring=Spring(a=anchor,b=box,k=20.0,c=2.0,rest=2.0)
world.bodies.append(anchor)
world.bodies.append(box)
world.springs.append(spring)'''

#bouncing mass spring damper demo
'''
anchor = Body(pos=Vec2(-2.0, 0.0),mass=0,width=0.2,height=0.2)
box=Body(pos=Vec2(2.0, 0.0),mass=2,width=1,height=1)
world.bodies.extend([anchor,box])
spring = Spring(a=anchor,b=box,k=20.0,c=2.0,rest=1.0)
world.springs.append(spring)'''

dt = 1 / 60  # FPS timestep
running = True

print("Simulation started. Press Close button to exit.")

while running:
    clock.tick(60)  # Limit to 60 FPS

    # 1. Handle Input
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 2. Step Physics
    world.step(dt)

    # 3. Render
    renderer.clear()

    # Draw the bodies
    for b in world.bodies:
        renderer.draw_body(b)

    for c in world.constraints:
        c.draw(renderer.screen, world_to_screen)

    for s in world.springs:
        if hasattr(s, "draw_spring"):
            s.draw_spring(renderer.screen, world_to_screen, s)

    # Draw the ground line (visual reference)
    renderer.draw_ground(ground_y=-3.0)

    # Display the frame
    renderer.present()

pygame.quit()