from vector import Vec2
from geometry import box_vertices, box_axes


# -------------------------------
# Helper functions
# -------------------------------
def project(vertices, axis):
    dots = [v.dot(axis) for v in vertices]
    return min(dots), max(dots)


def overlap(minA, maxA, minB, maxB):
    return min(maxA, maxB) - max(minA, minB)


def sat_box_box(a, b):
    axes = a.get_axes() + b.get_axes()
    min_overlap = float('inf')
    collision_normal = None

    for axis in axes:
        axis_n = axis.normalized()
        minA, maxA = project(a.get_vertices(), axis_n)
        minB, maxB = project(b.get_vertices(), axis_n)

        o = overlap(minA, maxA, minB, maxB)

        if o <= 0:
            return None
        if o < min_overlap:
            min_overlap = o
            collision_normal = axis_n

    d = b.pos - a.pos
    if d.dot(collision_normal) < 0:
        collision_normal = collision_normal * -1

    return collision_normal, min_overlap


def point_inside_box(p, box):
    verts = box_vertices(box)
    axes = box_axes(verts)
    for axis in axes:
        min_proj, max_proj = min(v.dot(axis) for v in verts), max(v.dot(axis) for v in verts)
        proj_p = p.dot(axis)
        # Using a small epsilon handles "touching" cases better
        if proj_p < min_proj - 1e-5 or proj_p > max_proj + 1e-5:
            return False
    return True


# -------------------------------
# Circle-ground collision
# -------------------------------
def resolve_ground_contact(body, ground_y, restitution=0.3, mu=0.6):
    if body.inv_mass == 0:
        return

    bottom = body.pos.y - body.radius
    if bottom > ground_y:
        return

    n = Vec2(0, 1)  # Ground points UP
    contact = Vec2(body.pos.x, ground_y)
    r = contact - body.pos

    v_contact = body.vel + r.perp() * body.ang_vel
    vn = v_contact.dot(n)

    jn = 0.0
    if vn < 0:
        ra_cn = r.perp().dot(n)
        inv_mass = body.inv_mass + ra_cn ** 2 * body.inv_inertia
        jn = -(1 + restitution) * vn / inv_mass
        body.vel += n * (jn * body.inv_mass)
        body.ang_vel += body.inv_inertia * r.perp().dot(n * jn)

    # Friction
    tangent = Vec2(1, 0)
    vt = v_contact.dot(tangent)

    # Check simple friction condition
    if abs(vt) > 0.001 and jn > 0:
        inv_mass = body.inv_mass + (body.radius ** 2) * body.inv_inertia
        if body.is_circle():
            # Rolling friction model
            target_vt = body.ang_vel * body.radius
            slip = vt - target_vt
            jt = -slip / inv_mass
            max_f = mu * jn
            jt = max(-max_f, min(max_f, jt))

            body.vel += tangent * (jt * body.inv_mass)
            body.ang_vel -= (jt * body.radius) * body.inv_inertia
        else:
            # Sliding friction
            jt = -vt / inv_mass
            max_f = mu * jn
            jt = max(-max_f, min(max_f, jt))
            body.vel += tangent * (jt * body.inv_mass)
            body.ang_vel += body.inv_inertia * r.perp().dot(tangent * jt)

    # Positional Correction
    penetration = ground_y - bottom
    if penetration > 0:
        body.pos.y += penetration


# -------------------------------
# Circle-circle collision
# -------------------------------
def resolve_circle_circle(a, b, restitution=0.6, mu=0.5):
    if a.inv_mass == 0 and b.inv_mass == 0:
        return

    delta = b.pos - a.pos
    dist = delta.length()
    min_dist = a.radius + b.radius

    if dist == 0 or dist > min_dist:
        return

    n = delta * (1 / dist)
    contact = a.pos + n * a.radius
    ra = contact - a.pos
    rb = contact - b.pos

    va = a.vel + ra.perp() * a.ang_vel
    vb = b.vel + rb.perp() * b.ang_vel
    rv = vb - va
    vn = rv.dot(n)
    if vn > 0: return

    ra_cn = ra.dot(n.perp())
    rb_cn = rb.dot(n.perp())
    inv_mass_sum = a.inv_mass + b.inv_mass + ra_cn ** 2 * a.inv_inertia + rb_cn ** 2 * b.inv_inertia

    j = -(1 + restitution) * vn / inv_mass_sum
    impulse = n * j

    a.vel -= impulse * a.inv_mass
    b.vel += impulse * b.inv_mass
    a.ang_vel -= a.inv_inertia * ra.dot(impulse.perp())
    b.ang_vel += b.inv_inertia * rb.dot(impulse.perp())

    # Friction
    t = (rv - n * vn)
    if t.length() > 1e-8:
        t = t.normalized()
        vt = rv.dot(t)
        jt = -vt / inv_mass_sum
        jt = max(-mu * j, min(mu * j, jt))
        f_impulse = t * jt
        a.vel -= f_impulse * a.inv_mass
        b.vel += f_impulse * b.inv_mass
        a.ang_vel -= a.inv_inertia * ra.dot(f_impulse.perp())
        b.ang_vel += b.inv_inertia * rb.dot(f_impulse.perp())

    pen = min_dist - dist
    if pen > 0:
        corr = n * (pen * 0.5)
        a.pos -= corr
        b.pos += corr


# -------------------------------
# Box-ground collision
# -------------------------------
def resolve_box_ground_contact(box, ground_y, restitution=0.4, mu=0.5):
    if box.inv_mass == 0: return
    vertices = box.get_vertices()
    bottom_vertices = [v for v in vertices if v.y <= ground_y + 0.05]
    if not bottom_vertices: return

    normal = Vec2(0, 1)

    # 1. Positional Correction First
    min_y = min(v.y for v in vertices)
    if min_y < ground_y:
        box.pos.y += (ground_y - min_y)

    # 2. Impulse Resolution
    for v in bottom_vertices:
        r = v - box.pos
        vel_at_contact = box.vel + r.perp() * box.ang_vel
        vn = vel_at_contact.dot(normal)
        jn = 0.0

        if vn < 0:
            r_cn = r.perp().dot(normal)
            inv_mass_sum = box.inv_mass + r_cn ** 2 * box.inv_inertia
            jn = -(1 + restitution) * vn / inv_mass_sum
            # Divide impulse by contact count to avoid double-energy explosion
            jn /= len(bottom_vertices)

            impulse = normal * jn
            box.vel += impulse * box.inv_mass
            box.ang_vel += r.perp().dot(impulse) * box.inv_inertia

        # Friction
        vt_vec = vel_at_contact - normal * vn
        if vt_vec.length() > 0.01:
            t = vt_vec.normalized()
            r_ct = r.perp().dot(t)
            inv_mass_sum = box.inv_mass + r_ct ** 2 * box.inv_inertia
            jt = -vel_at_contact.dot(t) / inv_mass_sum
            jt = max(-mu * abs(jn), min(mu * abs(jn), jt))

            f_impulse = t * jt
            box.vel += f_impulse * box.inv_mass
            box.ang_vel += r.perp().dot(f_impulse) * box.inv_inertia


# -------------------------------
# Box-box collision (REWRITTEN)
# -------------------------------
def resolve_box_box(a, b, restitution=0.3, mu=0.5, iterations=5):
    # 1. Generate Manifold ONCE
    result = sat_box_box(a, b)
    if result is None:
        return
    normal, penetration = result

    if penetration <= 0:
        return

    # 2. Apply Position Correction ONCE
    inv_mass_sum = a.inv_mass + b.inv_mass
    if inv_mass_sum == 0:
        return

    correction_mag = penetration / inv_mass_sum * 0.8  # 80% correction
    correction = normal * correction_mag
    a.pos -= correction * a.inv_mass
    b.pos += correction * b.inv_mass

    # 3. Find Contact Points
    verts_a = a.get_vertices()
    verts_b = b.get_vertices()
    contacts = [v for v in verts_a if point_inside_box(v, b)] + \
               [v for v in verts_b if point_inside_box(v, a)]

    # Fallback for perfect edge-edge alignment
    if not contacts:
        contacts = [(a.pos + b.pos) * 0.5]

    num_contacts = len(contacts)
    if num_contacts == 0: return

    # 4. Iteratively Solve Velocities
    # We use a fixed tangent for boxes (surface slide), not velocity tangent
    tangent = normal.perp()

    for _ in range(iterations):
        for contact in contacts:
            ra = contact - a.pos
            rb = contact - b.pos

            # Relative Velocity
            va = a.vel + ra.perp() * a.ang_vel
            vb = b.vel + rb.perp() * b.ang_vel
            rv = vb - va

            vel_along_normal = rv.dot(normal)

            # Do not resolve if separating
            if vel_along_normal > 0:
                continue

            ra_cn = ra.perp().dot(normal)
            rb_cn = rb.perp().dot(normal)
            denom = a.inv_mass + b.inv_mass + ra_cn ** 2 * a.inv_inertia + rb_cn ** 2 * b.inv_inertia

            # Normal Impulse
            j = -(1 + restitution) * vel_along_normal / denom
            j /= num_contacts  # Distribute impulse across contacts

            impulse = normal * j

            a.vel -= impulse * a.inv_mass
            a.ang_vel -= ra.perp().dot(impulse) * a.inv_inertia
            b.vel += impulse * b.inv_mass
            b.ang_vel += rb.perp().dot(impulse) * b.inv_inertia

            # Friction Impulse
            # Re-calculate relative velocity after normal push
            va = a.vel + ra.perp() * a.ang_vel
            vb = b.vel + rb.perp() * b.ang_vel
            rv = vb - va

            vt = rv.dot(tangent)
            ra_ct = ra.perp().dot(tangent)
            rb_ct = rb.perp().dot(tangent)
            denom_t = a.inv_mass + b.inv_mass + ra_ct ** 2 * a.inv_inertia + rb_ct ** 2 * b.inv_inertia

            jt = -vt / denom_t
            jt /= num_contacts

            # Clamp friction
            max_jt = mu * abs(j)
            jt = max(-max_jt, min(max_jt, jt))

            f_impulse = tangent * jt

            a.vel -= f_impulse * a.inv_mass
            a.ang_vel -= ra.perp().dot(f_impulse) * a.inv_inertia
            b.vel += f_impulse * b.inv_mass
            b.ang_vel += rb.perp().dot(f_impulse) * b.inv_inertia