# 2D Rigid Body Physics Engine

## Project Overview

This project is a 2D rigid body physics engine implemented in Python using Pygame for visualization. The engine simulates rigid body dynamics with collision handling, constraints, and basic physical interactions commonly found in game and simulation engines.

The goal of the project was to build a physics system from first principles, without relying on external physics libraries, in order to understand the mathematics and numerical methods behind real-time physics simulation.

---

## Features Implemented

### 1. Rigid Bodies

The engine supports two types of rigid bodies:

* Circles (balls)
* Axis-aligned / oriented boxes

Each rigid body has the following properties:

* Position (`Vec2`)
* Velocity
* Orientation (angle)
* Angular velocity
* Mass and inverse mass
* Moment of inertia and inverse inertia
* Shape-specific parameters (radius, width, height)

Static bodies are represented using zero inverse mass.

---

### 2. Collision Detection

#### a) Ball–Ball Collision

* Distance-based collision detection
* Collision normal computed from center separation
* Penetration depth used for positional correction

#### b) Ball–Ground Collision

* Ground modeled as an infinite horizontal plane
* Collision occurs when the bottom of the ball penetrates the ground level

#### c) Box–Box Collision

* Separating Axis Theorem (SAT)
* Projection of oriented box vertices onto candidate axes
* Minimum penetration axis selected as collision normal

#### d) Box–Ground Collision

* Box vertices tested against ground plane
* Penetration depth resolved using positional correction

---

### 3. Collision Resolution

Collisions are resolved using impulse-based dynamics:

* Normal impulse for restitution (bounciness)
* Angular impulse using moment of inertia
* Positional correction to prevent sinking and jitter

The engine supports:

* Linear momentum conservation
* Angular momentum changes
* Coefficient of restitution

---

### 4. Ground Friction Model

A stable ground friction model was implemented for balls:

* **Static friction**: completely stops motion below a velocity threshold
* **Dynamic friction**: applies a tangential impulse proportional to normal impulse

This prevents:

* Infinite rolling
* Numerical jitter
* Energy gain due to discretization

---

### 5. Constraints System

The engine includes a modular constraint solver executed every simulation step.

#### a) Distance Joints

* Maintain a fixed distance between two bodies
* Useful for rods and rigid connections
* Solved using positional correction based on inverse masses

#### b) Rope Constraints

* Similar to distance joints, but only resist stretching
* Can go slack when compressed
* Optional break threshold to simulate snapping

#### c) Spring–Mass System (Damped SHM)

* Hooke’s law based spring force
* Includes damping proportional to relative velocity
* Produces stable oscillatory motion

---

### 6. Constraint Breaking

Rope constraints support dynamic breaking:

* Stretch beyond a threshold permanently breaks the rope
* Enables realistic failure scenarios
* Used to demonstrate tension accumulation and snapping

---

### 7. Numerical Integration

* Semi-implicit (symplectic) Euler integration
* Stable for real-time simulations
* Separate integration for linear and angular quantities

Multiple solver iterations per frame are used to improve constraint stiffness and collision stability.

---

### 8. Rendering System

Rendering is handled using Pygame:

* World-to-screen coordinate transformation
* Support for camera-centered coordinates
* Visual indicators for:

  * Body orientation
  * Constraint connections
  * Ground plane

---

## Coordinate System

* World coordinates: right-handed system

  * +x → right
  * +y → upward
* Screen coordinates:

  * Origin at screen center
  * y-axis inverted for rendering

---

## Project Structure

```
PhysicsEngine_RBD_SimProject/
│
├── main.py              # Simulation loop
├── world.py             # World container & stepping
├── body.py              # Rigid body definitions
├── geometry.py          # Shape math & SAT helpers
├── constraints.py       # Distance, rope, spring constraints
├── collision.py         # Collision detection & resolution
├── render.py            # Pygame rendering
└── vector.py              # 2D vector math
```

---

## Key Learnings

* Practical implementation of rigid body dynamics
* Impulse-based collision resolution
* Numerical stability issues and solutions
* Importance of positional correction
* Constraint solvers and stiffness tuning
* Differences between forces and impulses

---

## Limitations

* No continuous collision detection (CCD)
* No broad-phase collision acceleration
* Simple friction model
* No sleeping / deactivation of bodies

These were intentionally excluded to keep the project basic and focused.

---

## Conclusion

This project demonstrates a complete 2D physics engine pipeline, including rigid body dynamics, collision handling, constraints, and rendering. It serves as a strong foundation for understanding real-world game physics engines and provides a base for further experimentation and optimization.


---
