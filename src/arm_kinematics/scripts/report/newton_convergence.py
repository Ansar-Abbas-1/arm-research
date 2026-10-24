"""Fraction of position+pitch Newton runs (no restarts) that converge from a 0.3 rad seed offset.
(Analysis script for docs/kinematics_report.md; numpy. Not part of the ROS build.)
"""
import numpy as np,sys
import os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from kin import *
rng=np.random.default_rng(11)
fin=[]
for _ in range(300):
    qt=rq(rng);Pt=fk(qt);q=np.clip(qt+rng.normal(0,0.3,5),LIM[:,0],LIM[:,1])
    for it in range(40):
        P=fk(q);e=np.r_[Pt[:3,3]-P[:3,3],P[:3,1]@rotvec(Pt[:3,:3]@P[:3,:3].T)]
        if np.linalg.norm(e)<1e-10:break
        J,Jt,_=jac(q);dq,s=chiaverini(Jt,e);m=np.abs(dq).max();dq=dq*(0.2/m) if m>0.2 else dq
        q=np.clip(q+dq,LIM[:,0],LIM[:,1])
    fin.append((np.linalg.norm(e),it))
f=np.array(fin);print("converged (<1e-10) within 40 its: %d/300; median its among converged %.0f; stalled final |e| median %.3g"%((f[:,0]<1e-10).sum(),np.median(f[f[:,0]<1e-10,1]),np.median(f[f[:,0]>=1e-10,0]) if (f[:,0]>=1e-10).any() else 0))
