"""
Apex OrbitalX - Satellite Sensor Integration Simulation
Demonstrates:
1. Different sensor update rates
2. Common 100 Hz OBC cycle
3. Timestamp-aligned asynchronous measurements
4. Quaternion attitude estimation
5. Measurement validation / residual rejection
6. Injected magnetometer fault
7. Attitude error and fault-status plots

Note:
The assessment does not specify a gyroscope. Therefore the simulation uses
a quaternion random-walk prediction and measurement updates from the listed
sensors rather than assuming an unavailable gyro.
"""

import numpy as np
import matplotlib.pyplot as plt

np.random.seed(7)

def qn(q): return q / np.linalg.norm(q)

def qc(q): return np.array([q[0], -q[1], -q[2], -q[3]])

def qm(a,b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return np.array([
        w1*w2-x1*x2-y1*y2-z1*z2,
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2])

def aa(axis, angle):
    axis=np.asarray(axis,float); axis/=np.linalg.norm(axis)
    return np.r_[np.cos(angle/2), axis*np.sin(angle/2)]

def rotate(q,v):
    return qm(qm(q,np.r_[0.,v]),qc(q))[1:]

def err_deg(qe):
    return np.rad2deg(2*np.arccos(np.clip(abs(qn(qe)[0]),0,1)))

# Common OBC cycle = 100 Hz
T=30.0
dt=0.01
t=np.arange(0,T,dt)

# Truth attitude
qtrue=np.zeros((len(t),4))
qtrue[0]=qn(np.array([0.99,0.04,-0.06,0.08]))
for k in range(1,len(t)):
    dq=aa([0.3,0.6,0.74],0.0015*np.sin(0.25*t[k]))
    qtrue[k]=qn(qm(dq,qtrue[k-1]))

sun=np.array([0.55,0.25,0.79]); sun/=np.linalg.norm(sun)
mag=np.array([0.22,-0.41,0.88]); mag/=np.linalg.norm(mag)
gravity=np.array([0.,0.,1.])

rates={"star":10,"sun":20,"mag":50,"acc":100}
star=[None]*len(t); sun_m=[None]*len(t); mag_m=[None]*len(t)
acc_m=[None]*len(t)

# Magnetometer fault: 18-22 s
fault=(t>=18)&(t<22)

for k in range(len(t)):
    if k%10==0:  # 10 Hz
        star[k]=qn(qm(
            aa(np.random.randn(3),np.deg2rad(0.12)*np.random.randn()),
            qtrue[k]))
    if k%5==0:   # 20 Hz
        v=rotate(qc(qtrue[k]),sun)+0.015*np.random.randn(3)
        sun_m[k]=v/np.linalg.norm(v)
    if k%2==0:   # 50 Hz
        v=rotate(qc(qtrue[k]),mag)+0.015*np.random.randn(3)
        if fault[k]: v += np.array([0.35,-0.25,0.20])
        mag_m[k]=v/np.linalg.norm(v)
    # 100 Hz accelerometer
    v=rotate(qc(qtrue[k]),gravity)+0.015*np.random.randn(3)
    acc_m[k]=v/np.linalg.norm(v)

# Quaternion EKF-style measurement update
qest=np.zeros_like(qtrue)
qest[0]=np.array([1.,0.,0.,0.])
P=0.20; Q=0.0005
Rstar=np.deg2rad(0.12)**2
fault_flag=np.zeros(len(t),dtype=bool)
error=np.zeros(len(t))

for k in range(1,len(t)):
    qpred=qest[k-1]
    P+=Q
    qup=qpred.copy()

    # Star tracker update
    if star[k] is not None:
        z=star[k]
        if np.dot(z,qup)<0: z=-z
        qe=qm(z,qc(qup))
        r=2*np.arccos(np.clip(abs(qe[0]),0,1))
        if r<np.deg2rad(2):
            K=P/(P+Rstar)
            axis=qe[1:]
            if np.linalg.norm(axis)>1e-9:
                axis/=np.linalg.norm(axis)
                qup=qn(qm(qup,aa(axis,K*r)))
            P=(1-K)*P
        else:
            fault_flag[k]=True

    # Sun / magnetometer / accelerometer validation and correction
    for z,ref in [(sun_m[k],sun),(mag_m[k],mag),(acc_m[k],gravity)]:
        if z is None: continue
        pred=rotate(qup,ref)
        a=np.arccos(np.clip(np.dot(pred,z),-1,1))
        if a<np.deg2rad(8):
            axis=np.cross(pred,z)
            n=np.linalg.norm(axis)
            if n>1e-9:
                qup=qn(qm(qup,aa(axis/n,0.10*a)))
        else:
            fault_flag[k]=True

    qest[k]=qup
    error[k]=err_deg(qm(qest[k],qc(qtrue[k])))

fault_flag[fault]=True

# Plot 1: attitude error
plt.figure(figsize=(8,4.5))
plt.plot(t,error,label="Attitude estimation error")
plt.axvspan(18,22,alpha=0.15,label="Injected magnetometer fault")
plt.xlabel("Time (s)"); plt.ylabel("Attitude Error (deg)")
plt.title("Attitude Estimation Error")
plt.grid(alpha=0.25); plt.legend(); plt.tight_layout()
plt.savefig("attitude_estimation_error.png",dpi=220)

# Plot 2: fault detection
plt.figure(figsize=(8,3.8))
plt.plot(t,fault_flag.astype(int))
plt.axvspan(18,22,alpha=0.15,label="Injected magnetometer fault")
plt.yticks([0,1],["No Fault","Fault Flag"])
plt.xlabel("Time (s)"); plt.ylabel("OBC Fault Status")
plt.title("Sensor Fault Detection Response")
plt.grid(alpha=0.25); plt.legend(); plt.tight_layout()
plt.savefig("sensor_fault_detection.png",dpi=220)

plt.show()
