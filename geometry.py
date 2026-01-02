from vector import Vec2
import math

def rotate(vec,angle):
    c,s=math.cos(angle),math.sin(angle)
    return Vec2(c*vec.x-s*vec.y,s*vec.x+c*vec.y)

def box_vertices(body):
    hw,hh=body.width/2,body.height/2
    local=[Vec2(-hw, -hh),Vec2(hw, -hh),Vec2(hw, hh),Vec2(-hw, hh)]
    return [body.pos +rotate(v,body.angle) for v in local]

def box_axes(vertices):
    axes=[]
    for i in range(len(vertices)):
        edge=vertices[(i+1)%4]-vertices[i]
        axes.append(edge.perp().normalized())
    return axes