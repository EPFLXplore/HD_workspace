import math
from geometry_msgs.msg import Quaternion, Point


def normalize(axis):
    n = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    axis = (axis[0]/n, axis[1]/n, axis[2]/n)
    return axis


def quat(axis, angle):
    orientation = Quaternion()
    axis = normalize(axis)
    orientation.w = math.cos(angle/2)
    orientation.x = axis[0]*math.sin(angle/2)
    orientation.y = axis[1]*math.sin(angle/2)
    orientation.z = axis[2]*math.sin(angle/2)
    return orientation


def nb_to_quat(x):
    q = Quaternion()
    q.w = x
    q.x = q.y = q.z = 0
    return q


def point_to_quat(p):
    q = Quaternion()
    q.w = 0
    q.x = p.x
    q.y = p.y
    q.z = p.z
    return q


def quat_to_point(q):
    # only if q.w is 0
    p = Point()
    p.x = q.x
    p.y = q.y
    p.z = q.z
    return p


def list_to_quat(l):
    q = Quaternion()
    q.x = l[0]
    q.y = l[1]
    q.z = l[2]
    q.w = l[3]


def list_to_point(l):
    p = Point()
    p.x = l[0]
    p.y = l[1]
    p.z = l[2]
    return p


def make_quat(x):
    if isinstance(x, Quaternion):
        return x
    if isinstance(x, (float, int)):
        return nb_to_quat(x)
    elif isinstance(x, Point):
        return point_to_quat(x)
    elif isinstance(x, (tuple, list)):
        return list_to_quat(x)


def make_point(x):
    if isinstance(x, Point):
        return x
    if isinstance(x, Quaternion):
        return quat_to_point(x)
    if isinstance(x, (tuple, list)):
        return list_to_point(x)


def inv(q):
    # only if norm of q is 1
    q = make_quat(q)
    q_ = Quaternion()
    q_.w = q.w
    q_.x = -q.x
    q_.y = -q.y
    q_.z = -q.z
    return q_


def add(q1, q2):
    q1 = make_quat(q1)
    q2 = make_quat(q2)
    ans = Quaternion()
    ans.w = q1.w + q2.w
    ans.x = q1.x + q2.x
    ans.y = q1.y + q2.y
    ans.z = q1.z + q2.z
    return ans


def mul(q1, q2):
    q1 = make_quat(q1)
    q2 = make_quat(q2)
    ans = Quaternion()
    ans.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
    ans.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
    ans.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x
    ans.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    return ans


def point_image(point, q):
    point = make_point(point)
    p = point_to_quat(point)
    q_ = inv(q)
    p = mul(mul(q, p), q_)
    return quat_to_point(p)
