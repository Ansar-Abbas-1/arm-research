"""Independent numpy model of the arm (URDF-composed FK, Jacobian, task Jacobian, Chiaverini step) used to cross-check the C++ library.
(Analysis script for docs/kinematics_report.md; numpy. Not part of the ROS build.)
"""
import numpy as np
LIM=np.array([(-2.745,2.736),(0,3.14),(-2.22,0),(0,4.505),(-3.62,0)])
OFF=dict(p2=-1.57,pi=3.1412,p4=-1.8791,p5=1.81)
def Rx(a):
    c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a):
    c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def Tm(R,p):
    M=np.eye(4);M[:3,:3]=R;M[:3,3]=p;return M
def chain(q,pi_exact=False,p2=None):
    """returns tip pose 4x4 and list of (origin, axis) for the 5 joints"""
    q1,q2,q3,q4,q5=q
    pi=np.pi if pi_exact else OFF['pi']; p2=-np.pi/2 if pi_exact else OFF['p2']
    P=np.eye(4);fr=[]
    def step(xyz,rpy_pitch,axis=None,qv=0.0):
        nonlocal P
        P=P@Tm(Ry(rpy_pitch),xyz)  # pure Ry fixed rotation
        if axis is not None:
            fr.append((P[:3,3].copy(),P[:3,:3]@np.array(axis,float)))
            R={'z':Rz,'y':Ry}[{(0,0,1):'z',(0,1,0):'y'}[tuple(axis)]](qv)
            P=P@Tm(R,[0,0,0])
    step([0,0,0.03],0,(0,0,1),q1)
    step([0,0,0.065],p2,(0,1,0),q2)
    step([0,0,0.34],0,(0,1,0),q3)
    step([0.029,0,0],pi)
    step([0,0,0.376],OFF['p4'],(0,1,0),q4)
    step([0,0,0.336],OFF['p5'],(0,1,0),q5)
    step([0,0,0.105],0)
    return P,fr
def fk(q,**k): return chain(q,**k)[0]
def jac(q):
    P,fr=chain(q);pe=P[:3,3]
    J=np.array([np.r_[np.cross(z,pe-p),z] for p,z in fr]).T
    Jt=np.vstack([J[:3],P[:3,1]@J[3:]])
    return J,Jt,P
def rq(rng): return rng.uniform(LIM[:,0],LIM[:,1])
def rotvec(R):
    c=(np.trace(R)-1)/2;th=np.arccos(np.clip(c,-1,1))
    if th<1e-12:return np.zeros(3)
    if abs(th-np.pi)<1e-6:
        w,v=np.linalg.eigh((R+R.T)/2);u=v[:,-1];return th*u
    return th/(2*np.sin(th))*np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])
def chiaverini(Jt,e,eps=0.05):
    G=Jt@Jt.T;s=np.sqrt(max(np.linalg.eigvalsh(G)[0],0))
    l2=0.0 if s>=eps else eps**2*(1-(s/eps)**2)
    return Jt.T@np.linalg.solve(G+l2*np.eye(4),e),s
