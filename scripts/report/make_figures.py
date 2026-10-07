"""Generates the report's figures and statistics (DH check, planar workspace, sigma_min statistics, Newton convergence, self-motion family, error propagation). Usage: FIG_DIR=docs/figures python3 make_figures.py
(Analysis script for docs/kinematics_report.md; numpy, matplotlib. Not part of the ROS build.)
"""
import numpy as np, sys
import os
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from kin import *
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=os.environ.get('FIG_DIR','./figs')+'/'; os.makedirs(out,exist_ok=True)
rng=np.random.default_rng(11)
# ---- DH check
beta=np.arctan2(0.029+0.376*np.sin(3.1412),0.376*np.cos(3.1412)); rho=np.hypot(0.029+0.376*np.sin(3.1412),0.376*np.cos(3.1412))
th4=(3.1412-1.8791)-beta
dh=[(0,-np.pi/2,0.095,0),(0.34,0,0,-1.57-np.pi/2),(rho,0,0,beta),(0.336,0,0,th4),(0.105,0,0,1.81)]
def dhT(a,al,d,th):
    ct,st,ca,sa=np.cos(th),np.sin(th),np.cos(al),np.sin(al)
    return np.array([[ct,-st*ca,st*sa,a*ct],[st,ct*ca,-ct*sa,a*st],[0,sa,ca,d],[0,0,0,1]])
def fk_dh(q):
    P=np.eye(4)
    for i,(a,al,d,th) in enumerate(dh): P=P@dhT(a,al,d,th+q[i])
    # DH frame5 -> URDF end_effector: x_ee=y5, y_ee=z5, z_ee=x5, same origin
    Rc=np.array([[0,0,1],[1,0,0],[0,1,0]]).T  # columns y5,z5,x5 -> matrix with columns [y5,z5,x5] in frame5 coords
    Rc=np.column_stack([[0,1,0],[0,0,1],[1,0,0]])
    return P@Tm(Rc,[0,0,0])
err=0
for _ in range(500):
    q=rq(rng);A=fk(q);B=fk_dh(q);err=max(err,np.abs(A-B).max())
print("DH vs URDF FK max abs diff:",err,"beta",beta,"rho",rho,"theta4 off",th4,"theta2 off",-1.57-np.pi/2)
# ---- planar workspace occupancy
def planar(Q):
    q2,q3,q4,q5=Q.T
    s=lambda a:np.stack([np.sin(a),np.cos(a)],1);c=lambda a:np.stack([np.cos(a),-np.sin(a)],1)
    f2=-1.57+q2;f3=f2+q3;fi=f3+3.1412;f4=fi-1.8791+q4;f5=f4+1.81+q5
    return np.array([0,0.095])+0.34*s(f2)+0.029*c(f3)+0.376*s(fi)+0.336*s(f4)+0.105*s(f5)
cell=0.005;nr=int(2.8/cell);nz=int(2.0/cell);occ=np.zeros((nr,nz),bool)
lo,hi=LIM[1:,0],LIM[1:,1]
for _ in range(16):
    Q=rng.uniform(lo,hi,(2_000_000,4));p=planar(Q)
    i=np.floor((p[:,0]+1.4)/cell).astype(int);k=np.floor((p[:,1]+0.8)/cell).astype(int)
    ok=(i>=0)&(i<nr)&(k>=0)&(k<nz);occ[i[ok],k[ok]]=True
fig,ax=plt.subplots(figsize=(6,6))
ax.imshow(occ.T[::-1],extent=[-1.4,1.4,-0.8,1.2],cmap='Blues',aspect='equal')
t=np.linspace(0,2*np.pi,400);ax.plot(1.281*np.cos(t),0.095+1.281*np.sin(t)*0+0,alpha=0)
ax.add_patch(plt.Circle((0,0),1.189,fill=False,ls='--',color='k',lw=0.8,label='sampled max reach 1.189 m'))
ax.add_patch(plt.Circle((0,0),1.281,fill=False,ls=':',color='r',lw=0.8,label='geometric bound 1.281 m'))
ax.add_patch(plt.Rectangle((-0.2,-0.4),0.4,0.4,fill=False,ec='brown',lw=1.2,label='base box (0.4 m cube)'))
ax.plot(0,0.095,'k^',ms=5,label='joint_2 axis (0, 0.095)')
ax.set_xlabel('r [m] (signed radius in the arm plane)');ax.set_ylabel('z [m]');ax.legend(loc='lower center',fontsize=7)
ax.set_title('Planar workspace section W (4R chain, joint limits only)')
plt.tight_layout();plt.savefig(out+'fig_workspace_section.png',dpi=160);plt.close()
Aocc=occ.sum()*cell**2;print("planar area m^2",Aocc)
# ---- sigma_min / manipulability statistics
sm=[];wm=[];kc=[];cond=[]
for _ in range(30000):
    q=rq(rng);J,Jt,_P=jac(q);s=np.linalg.svd(Jt)[1];sm.append(s[-1]);wm.append(np.prod(s));kc.append(s[0]/s[-1])
sm=np.array(sm);wm=np.array(wm);kc=np.array(kc)
print("sigma_min pct 1,5,10,50,90:",np.percentile(sm,[1,5,10,50,90]).round(4),"frac<0.05",(sm<0.05).mean(),"frac<0.01",(sm<0.01).mean())
print("Yoshikawa w=prod sigma pct 5,50,95:",np.percentile(wm,[5,50,95]).round(4),"cond pct 50,90,99",np.percentile(kc,[50,90,99]).round(1))
fig,ax=plt.subplots(1,2,figsize=(9,3.4))
ax[0].hist(sm,bins=80,color='tab:blue');ax[0].axvline(0.05,color='r',ls='--',label='damping threshold $\\epsilon$=0.05');ax[0].set_xlabel('$\\sigma_{min}(J_{task})$');ax[0].set_ylabel('count (30k uniform configs)');ax[0].legend()
ax[1].hist(np.log10(kc),bins=80,color='tab:green');ax[1].set_xlabel('$\\log_{10}$ cond$(J_{task})$');
plt.tight_layout();plt.savefig(out+'fig_sigma_min_hist.png',dpi=160);plt.close()
# ---- DLS bias and Chiaverini damping curves
s=np.linspace(0,0.3,400);lam=0.05
fig,ax=plt.subplots(1,2,figsize=(9,3.4))
ax[0].plot(s,s**2/(s**2+lam**2),label='fixed $\\lambda$=0.05');l2=np.where(s>=lam,0,lam**2*(1-(s/lam)**2));ax[0].plot(s,s**2/(s**2+l2),label='Chiaverini variable');ax[0].set_xlabel('$\\sigma_i$');ax[0].set_ylabel('$\\sigma^2/(\\sigma^2+\\lambda^2)$ (tracking gain per direction)');ax[0].legend()
ax[1].plot(s,np.sqrt(l2));ax[1].set_xlabel('$\\sigma_{min}$');ax[1].set_ylabel('$\\lambda(\\sigma_{min})$');plt.tight_layout();plt.savefig(out+'fig_damping.png',dpi=160);plt.close()
# ---- Newton convergence (python port, damping Chiaverini, no clamp except 0.2)
def err_vec(q,tgt_p,tgt_pitchdir_R):
    P=fk(q);ep=tgt_p-P[:3,3];er=rotvec(tgt_pitchdir_R@P[:3,:3].T);return np.r_[ep,P[:3,1]@er],P
traces=[]
for _ in range(300):
    qt=rq(rng);Pt=fk(qt);q=np.clip(qt+rng.normal(0,0.3,5),LIM[:,0],LIM[:,1]);tr=[]
    for it in range(12):
        e,P=err_vec(q,Pt[:3,3],Pt[:3,:3]);tr.append(np.linalg.norm(e))
        J,Jt,_=jac(q);dq,_s=chiaverini(Jt,e);m=np.abs(dq).max();dq=dq*(0.2/m) if m>0.2 else dq
        q=np.clip(q+dq,LIM[:,0],LIM[:,1])
    traces.append(tr)
T=np.array(traces);med=np.median(T,0)
print("median |e| per iteration:",np.array2string(med,precision=2))
fig,ax=plt.subplots(figsize=(5.2,3.6))
for tr in T[:60]: ax.semilogy(tr,color='tab:blue',alpha=0.15)
ax.semilogy(med,'k-o',label='median (300 runs)');ax.set_xlabel('iteration');ax.set_ylabel('$\\|e_{task}\\|$');ax.legend();ax.set_ylim(1e-14,1)
plt.tight_layout();plt.savefig(out+'fig_newton_convergence.png',dpi=160);plt.close()
# ---- analytic IK scan / self-motion structure for one target
l2_,L4,L5=0.34,0.336,0.105;v0=np.array([0.029+0.376*np.sin(3.1412),0.376*np.cos(3.1412)]);rho=np.hypot(*v0);beta=np.arctan2(*v0)
def wrap(a,lo,hi):
    return [a+2*np.pi*k for k in (-2,-1,0,1,2) if lo-1e-9<=a+2*np.pi*k<=hi+1e-9]
def branches(r,z,Phi,f4):
    out=[]
    p5=np.array([r,z])-L5*np.array([np.sin(Phi),np.cos(Phi)]);p4=p5-L4*np.array([np.sin(f4),np.cos(f4)])
    d=p4-np.array([0,0.095]);c=(d@d-l2_**2-rho**2)/(2*l2_*rho)
    if abs(c)>1:return out
    for sg in (1,-1):
        q3=-beta+sg*np.arccos(c);f2=np.angle(complex(d[1],d[0]))-np.angle(l2_+rho*np.exp(1j*(q3+beta)))
        q2=f2+1.57;fi=f2+q3+3.1412;q4=f4-fi+1.8791;q5=Phi-f4-1.81
        c_=[wrap(v,LIM[i+1,0],LIM[i+1,1]) for i,v in enumerate((q2,q3,q4,q5))]
        if all(c_):out.append((sg,[x[0] for x in c_]))
    return out
q=np.array([0.3,0.5,-0.2,0.4,-0.3]);P=fk(q);r=np.hypot(*P[:2,3]);z=P[2,3];Phi=q[1:].sum()+1.5021
grid=np.linspace(-np.pi,np.pi,2881);fig,axs=plt.subplots(2,1,figsize=(7,5.4),sharex=True)
cols={1:'tab:blue',-1:'tab:orange'}
for sg in (1,-1):
    d=np.full((len(grid),4),np.nan)
    for i,f4 in enumerate(grid):
        for s_,qq in branches(r,z,Phi,f4):
            if s_==sg:d[i]=qq
    for j in range(4):axs[0].plot(np.degrees(grid),d[:,j],'.',ms=1.5,color=cols[sg])
    tilt=np.degrees(np.arccos(np.abs(np.cos(grid))));tilt[np.isnan(d[:,0])]=np.nan;axs[1].plot(np.degrees(grid),tilt,'.',ms=2,color=cols[sg],label='elbow branch %+d'%sg)
axs[0].set_ylabel('$q_2..q_5$ [rad] (4 curves/branch)');axs[1].set_ylabel('link_4 tilt from vertical [deg]');axs[1].set_xlabel('$\\varphi_4$ (link_4 angle) [deg]');axs[1].legend()
axs[0].set_title('Self-motion family of one target (position + pitch fixed)',fontsize=10)
plt.tight_layout();plt.savefig(out+'fig_self_motion.png',dpi=160);plt.close()
# number of solution components per target (intervals of feasible phi4 per elbow/base branch)
cnt=[];feas=[]
for _ in range(1500):
    q=rq(rng);P=fk(q);r=np.hypot(*P[:2,3])*np.sign(np.cos(np.arctan2(P[1,3],P[0,3])-q[0]));z=P[2,3];Phi=q[1:].sum()+1.5021
    comps=0;tot=0
    for sg in (1,-1):
        ok=np.array([any(s_==sg for s_,_ in branches(r,z,Phi,f4)) for f4 in grid])
        comps+=int(((ok[1:])&(~ok[:-1])).sum()+(ok[0] and not ok[-1])) if ok.any() else 0
        tot+=ok.mean()
    cnt.append(comps);feas.append(tot)
cnt=np.array(cnt);print("feasible-phi4 intervals on the same base branch: counts",{k:int((cnt==k).sum()) for k in np.unique(cnt)},"mean feasible fraction of phi4 range",np.mean(feas)/2)
# alternative base branch count (q1+pi, r->-r)
# ---- error propagation
tick=2*np.pi/4096;dp=[];dpr=[]
for _ in range(5000):
    q=rq(rng);P=fk(q);dq=rng.normal(0,tick,5);P2=fk(q+dq);dp.append(np.linalg.norm(P2[:3,3]-P[:3,3]))
    Pe=fk(q,pi_exact=True);dpr.append(np.linalg.norm(Pe[:3,3]-P[:3,3]))
dp=np.array(dp);dpr=np.array(dpr)
print("tip error for 1-tick (1.53 mrad) iid joint noise: mean %.2f mm, p95 %.2f mm"%(dp.mean()*1e3,np.percentile(dp,95)*1e3))
print("tip shift from exact pi/-pi/2 vs URDF-rounded constants: mean %.2f mm, p95 %.2f, max %.2f"%(dpr.mean()*1e3,np.percentile(dpr,95)*1e3,dpr.max()*1e3))
# lever arm: tip error per joint rad
print("mean |J_v| column norms [m/rad]:",np.mean([np.linalg.norm(jac(rq(rng))[0][:3],axis=0) for _ in range(2000)],0).round(3))

# ---- schematic of the planar chain
def pts_planar(q):
    q2,q3,q4,q5=q;s=lambda a:np.array([np.sin(a),np.cos(a)]);c=lambda a:np.array([np.cos(a),-np.sin(a)])
    f2=-1.57+q2;f3=f2+q3;fi=f3+3.1412;f4=fi-1.8791+q4;f5=f4+1.81+q5
    P=[np.array([0,0.0]),np.array([0,0.095])]
    P.append(P[-1]+0.34*s(f2));P.append(P[-1]+0.029*c(f3)+0.376*s(fi));P.append(P[-1]+0.336*s(f4));P.append(P[-1]+0.105*s(f5));return np.array(P)
fig,axs=plt.subplots(1,3,figsize=(11,4))
for ax,(q,t) in zip(axs,[([0,0,0,0],'$q=0$ (rest pose)'),([0.5,-0.2,0.4,-0.3],'$q_{2..5}=(0.5,-0.2,0.4,-0.3)$'),([1.6,-1.2,1.5,-1.2],'$q_{2..5}=(1.6,-1.2,1.5,-1.2)$')]):
    P=pts_planar(q);ax.plot(P[:,0],P[:,1],'-o',color='k',lw=2,ms=5)
    for i,nm in enumerate(['base','j2','j3','j4','j5','tip']):
        if 1<=i<=4: ax.annotate('joint_%d'%(i+1),P[i],textcoords='offset points',xytext=(5,5),fontsize=8)
    ax.annotate('tip',P[-1],textcoords='offset points',xytext=(5,-10),fontsize=8,color='r')
    ax.set_aspect('equal');ax.set_xlim(-0.8,0.8);ax.set_ylim(-0.1,0.9);ax.set_title(t,fontsize=9);ax.grid(alpha=0.3);ax.set_xlabel('r [m]')
axs[0].set_ylabel('z [m]');plt.tight_layout();plt.savefig(out+'fig_planar_chain.png',dpi=160);plt.close()
# ---- hand-eye simulation summary (values from he_sim.py)
labels=['full\n10 views','full\n20 views','pitch only\n20 views','yaw only\n20 views'];tr=[1.0,0.8,17.2,7.6];ro=[0.16,0.09,0.55,0.26]
fig,ax=plt.subplots(1,2,figsize=(9,3.3));ax[0].bar(labels,tr,color=['tab:green','tab:green','tab:red','tab:orange']);ax[0].set_ylabel('translation error of $X$ [mm] (median, PARK)');ax[0].set_yscale('log')
ax[1].bar(labels,ro,color=['tab:green','tab:green','tab:red','tab:orange']);ax[1].set_ylabel('rotation error of $X$ [deg] (median, PARK)');plt.tight_layout();plt.savefig(out+'fig_hand_eye_excitation.png',dpi=160);plt.close()
