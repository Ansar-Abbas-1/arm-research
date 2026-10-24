"""Hand-eye calibration simulation with this arm's kinematics (cv2.calibrateHandEye, five methods): accuracy vs excitation, views and noise.
(Analysis script for docs/kinematics_report.md; numpy, OpenCV. Not part of the ROS build.)
"""
import numpy as np, cv2, sys, time
import os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from kin import *
rng=np.random.default_rng(5)
K=np.array([[1522,0,952],[0,1522,532],[0,0,1.]]);D=np.array([-0.375,0.173,0,0,0.0])
# ChArUco-like inner corner grid 8x6, 25 mm
gx,gy=np.meshgrid(np.arange(1,9)*0.025,np.arange(1,7)*0.025);obj=np.stack([gx.ravel(),gy.ravel(),np.zeros(48)],1).astype(np.float64)
# true mount: camera frame in ee frame: yaw +90deg about ee z, t=(-0.025,0,-0.06)
Xt=Tm(Rz(np.pi/2),[-0.025,0,-0.06])
# board pose in base: lying flat on the ground ahead of the arm
Tbb=Tm(Rz(np.radians(10)),[0.30,-0.10,-0.10])
def cam_board(q):
    Tb_c=fk(q)@Xt;return np.linalg.inv(Tb_c)@Tbb,Tb_c
def visible(Tcb):
    pts=(Tcb[:3,:3]@obj.T).T+Tcb[:3,3]
    if (pts[:,2]<0.15).any() or (pts[:,2]>0.7).any():return False
    uv=(K@pts.T).T;uv=uv[:,:2]/uv[:,2:3]
    return bool((uv[:,0]>150).all() and (uv[:,0]<1770).all() and (uv[:,1]>150).all() and (uv[:,1]<930).all()) and Tcb[:3,:3][2,2]!=0 and (Tcb[:3,:3]@np.array([0,0,1.]))[2]>0.3
# build a pool of visible configurations
pool=[]
t0=time.time()
while len(pool)<400 and time.time()-t0<600:
    q=rq(rng);Tcb,_=cam_board(q)
    if visible(Tcb):pool.append(q)
pool=np.array(pool);print("visible pool",len(pool),"of random samples; q1 range",np.degrees(pool[:,0].min()).round(0),np.degrees(pool[:,0].max()).round(0))
def pitch_total(q): return q[1:].sum()
def pick(n,mode):
    if mode=='full': idx=rng.choice(len(pool),n,replace=False);return pool[idx]
    if mode=='pitch_only':
        # views with (nearly) the same q1 -> rotation axes all parallel to tool y
        c=np.median(pool[:,0]);m=np.abs(pool[:,0]-c)<0.03
        sub=pool[m]; 
        return sub[rng.choice(len(sub),min(n,len(sub)),replace=len(sub)<n)]
    if mode=='yaw_only':
        # same total pitch (phi5) -> only q1 varies in rotation
        c=np.median([pitch_total(q) for q in pool]);m=np.array([abs(pitch_total(q)-c)<0.05 for q in pool])
        sub=pool[m];return sub[rng.choice(len(sub),min(n,len(sub)),replace=len(sub)<n)]
def view(q,sig_px,sig_q):
    Tcb,_=cam_board(q)
    uv,_j=cv2.projectPoints(obj,cv2.Rodrigues(Tcb[:3,:3])[0],Tcb[:3,3],K,D);uv=uv.reshape(-1,2)+rng.normal(0,sig_px,(48,2))
    ok,rv,tv=cv2.solvePnP(obj,uv.reshape(-1,1,2),K,D,flags=cv2.SOLVEPNP_IPPE);rv,tv=cv2.solvePnPRefineLM(obj,uv.reshape(-1,1,2),K,D,rv,tv)
    Tm_=np.eye(4);Tm_[:3,:3]=cv2.Rodrigues(rv)[0];Tm_[:3,3]=tv.ravel()
    qn=q+rng.normal(0,sig_q,5);return fk(qn),Tm_   # arm pose from noisy joint reading
METH={'TSAI':cv2.CALIB_HAND_EYE_TSAI,'PARK':cv2.CALIB_HAND_EYE_PARK,'HORAUD':cv2.CALIB_HAND_EYE_HORAUD,'ANDREFF':cv2.CALIB_HAND_EYE_ANDREFF,'DANIILIDIS':cv2.CALIB_HAND_EYE_DANIILIDIS}
def run(n,mode,sig_px=0.5,sig_q=2*np.pi/4096,trials=60):
    res={m:[] for m in METH}
    for _ in range(trials):
        qs=pick(n,mode);V=[view(q,sig_px,sig_q) for q in qs]
        Rg=[v[0][:3,:3] for v in V];tg=[v[0][:3,3] for v in V];Rt=[v[1][:3,:3] for v in V];tt=[v[1][:3,3] for v in V]
        for m,c in METH.items():
            try:
                R,t=cv2.calibrateHandEye(Rg,tg,Rt,tt,method=c)
                dR=R@Xt[:3,:3].T;ang=np.degrees(np.arccos(np.clip((np.trace(dR)-1)/2,-1,1)))
                Xe=Tm(R,t.ravel());p=np.array([0,0,0.4,1.0]);lat=np.linalg.norm(((Xe-Xt)@p)[:3])
                res[m].append((ang,np.linalg.norm(t.ravel()-Xt[:3,3])*1e3,lat*1e3))
            except Exception as e: res[m].append((np.nan,np.nan,np.nan))
    return {m:np.nanmedian(np.array(v),0) for m,v in res.items()},{m:np.nanpercentile(np.array(v),90,axis=0) for m,v in res.items()}
rows=[]
for mode in ('full','pitch_only','yaw_only'):
    for n in ((10,20) if mode=='full' else (20,)):
        med,p90=run(n,mode)
        print("\n== excitation",mode,"views",n,"(median / p90 of rot err [deg], trans err [mm], 0.4 m-point err [mm])")
        for m in METH:print("  %-10s rot %.2f / %.2f   trans %.1f / %.1f   pt@0.4m %.1f / %.1f"%(m,med[m][0],p90[m][0],med[m][1],p90[m][1],med[m][2],p90[m][2]))
# noise sweep for full excitation, TSAI/PARK
for sp,sq in ((0.25,0.0),(0.5,0.0),(1.0,0.0),(0.5,2*np.pi/4096),(0.5,6*np.pi/4096)):
    med,p90=run(20,'full',sig_px=sp,sig_q=sq,trials=40)
    print("noise px %.2f joint %.2f mrad: PARK rot %.2f deg trans %.1f mm pt %.1f mm | DANIILIDIS rot %.2f trans %.1f pt %.1f"%(sp,sq*1e3,med['PARK'][0],med['PARK'][1],med['PARK'][2],med['DANIILIDIS'][0],med['DANIILIDIS'][1],med['DANIILIDIS'][2]))
