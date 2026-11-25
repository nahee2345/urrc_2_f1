import numpy as np
import math

KMAX=1.85
CURVES=np.linspace(-KMAX,KMAX,25)
STEPS=np.arange(.06,1.51,.06)
OFFSETS=np.array([-.12,0,.12])

def clamp(x,a,b):
    return max(a,min(b,x))

def arc(k):
    yaw=k*STEPS
    if abs(k)<1e-9:
        x=STEPS
        y=np.zeros_like(STEPS)
    else:
        x=np.sin(yaw)/k
        y=(1-np.cos(yaw))/k
    return np.stack((
        x[:,None]+np.cos(yaw[:,None])*OFFSETS,
        y[:,None]+np.sin(yaw[:,None])*OFFSETS
    ),axis=-1)

ARCS=[arc(k) for k in CURVES]

def corridor(p):
    for x in [.85,.60,.35]:
        q=p[np.abs(p[:,0]-x)<.16]
        l=q[q[:,1]>.13,1]
        r=q[q[:,1]<-.13,1]

        if len(l)>=2 and len(r)>=2:
            a=float(np.median(l))
            b=float(np.median(r))
            if .25<a-b<5:
                y=(a+b)/2
                return clamp(
                    2*y/max(x*x+y*y,.12),
                    -KMAX,KMAX)

    q=p[(p[:,0]>-.12)&(p[:,0]<.45)]
    l=q[q[:,1]>.05,1]
    r=-q[q[:,1]<-.05,1]

    if len(l) and len(r):
        return clamp(
            1.2*(min(l)-min(r)),
            -KMAX,KMAX)

    if len(l) and min(l)<.32:
        return -.75
    if len(r) and min(r)<.32:
        return .75
    return 0.

def plan(p,previous=0.,closing=0.,max_speed=.85):
    p=np.asarray(p,dtype=float).reshape(-1,2)
    p=p[np.isfinite(p).all(axis=1)]
    p=p[np.linalg.norm(p,axis=1)<2.8]

    target=corridor(p)
    best=None

    for k,centers in zip(CURVES,ARCS):
        if len(p):
            d2=np.sum(
                (centers[:,:,None,:]
                 -p[None,None,:,:])**2,axis=-1)
            margins=np.sqrt(
                np.min(d2,axis=(1,2)))-.112
        else:
            margins=np.full(len(STEPS),2.)

        hits=np.flatnonzero(margins<.025)
        free=float(STEPS[hits[0]]-.06) \
            if len(hits) else float(STEPS[-1])

        if free<.18:
            continue

        n=max(1,int(min(free,.65)/.06))
        clearance=float(
            np.mean(np.minimum(margins[:n],.30)))

        score=(2.2*free+1.8*clearance
               -.44*abs(k-target)
               -.18*abs(k-previous))

        if best is None or score>best[0]:
            best=(score,float(k),free)

    if best is None:
        return 0.,0.,0.

    _,k,free=best
    reserve=.20+min(.32,max(0.,closing)*.20)

    v=min(
        max_speed,
        math.sqrt(2*1.15*max(0.,free-reserve)),
        math.sqrt(.55/max(abs(k),.1)),
        max(0.,(free-reserve)/.32)
    )

    if free<.24:
        v=0.

    return float(v),k,free
