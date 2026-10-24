import numpy as np

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy(r,p,y):
    # URDF/tf2 fixed-axis convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return Rz(y) @ Ry(p) @ Rx(r)

def T(xyz, rpy_tuple):
    M = np.eye(4)
    M[:3,:3] = rpy(*rpy_tuple)
    M[:3,3] = xyz
    return M

def axis_angle(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x,y,z = axis
    c, s, C = np.cos(theta), np.sin(theta), 1-np.cos(theta)
    R = np.array([
        [x*x*C+c,   x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c,   y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c],
    ])
    M = np.eye(4)
    M[:3,:3] = R
    return M

# Chain, root(base_link) -> tip(link_5), from urdf/arm.urdf
joints = [
    dict(name="joint_1", xyz=(0,0,0.03),    rpy=(0,0,0),        axis=(0,0,1), actuated=True),
    dict(name="joint_2", xyz=(0,0,0.065),   rpy=(0,-1.57,0),    axis=(0,1,0), actuated=True),
    dict(name="joint_3", xyz=(0,0,0.34),    rpy=(0,0,0),        axis=(0,1,0), actuated=True),
    dict(name="joint_intermediate", xyz=(0.029,0,0), rpy=(0,3.1412,0), axis=None, actuated=False),
    dict(name="joint_4", xyz=(0,0,0.376),   rpy=(0,-1.8791,0),  axis=(0,1,0), actuated=True),
    dict(name="joint_5", xyz=(0,0,0.336),   rpy=(0,1.81,0),     axis=(0,1,0), actuated=True),
]

def fk(q):  # q: 5 actuated joint values in order
    P = np.eye(4)
    qi = 0
    for j in joints:
        P = P @ T(j["xyz"], j["rpy"])
        if j["actuated"]:
            P = P @ axis_angle(j["axis"], q[qi])
            qi += 1
    return P

def report(name, q):
    P = fk(q)
    pos = P[:3,3]
    R = P[:3,:3]
    print(f"--- {name} ---  q={q}")
    print("position:", np.array2string(pos, precision=9, separator=', '))
    print("rotation matrix:")
    print(np.array2string(R, precision=9, separator=', '))
    print()

report("zero", [0,0,0,0,0])
report("nontrivial", [0.3, 0.5, -0.2, 0.4, -0.3])
report("j1_only_pi2", [np.pi/2, 0,0,0,0])
