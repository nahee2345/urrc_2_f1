"""Metric polyline operations in base_link; independent of the PP controller."""
import numpy as np

def basis(path):
    delta=np.gradient(path,axis=0);t=delta/np.maximum(np.linalg.norm(delta,axis=1)[:,None],1e-9)
    return t,np.column_stack((-t[:,1],t[:,0]))

def progress(path):
    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]

def normal_hit(point,normal,wall,side):
    a=wall[:-1];v=np.diff(wall,axis=0);q=a-point
    cross=lambda x,y:x[...,0]*y[...,1]-x[...,1]*y[...,0]
    det=cross(normal,v);safe=abs(det)>1e-9
    d=np.divide(cross(q,v),det,out=np.full(len(v),np.nan),where=safe)
    u=np.divide(cross(q,normal),det,out=np.full(len(v),np.nan),where=safe)
    good=safe&(u>=-1e-6)&(u<=1+1e-6)&(side*d>0)
    return float(d[np.flatnonzero(good)[np.argmin(abs(d[good]))]]) if good.any() else None

def resample_x(points,spacing=.1,horizon=5.):
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2:return None
    p=p[np.isfinite(p).all(axis=1)&(p[:,0]>0)&(p[:,0]<=horizon)]
    if len(p)<3:return None
    fold=np.flatnonzero(np.diff(p[:,0])<=1e-5)
    if len(fold):p=p[:fold[0]+1]
    if len(p)<3:return None
    x=np.arange(np.ceil(p[0,0]/spacing)*spacing,p[-1,0]+1e-8,spacing)
    return np.column_stack((x,np.interp(x,p[:,0],p[:,1]))) if len(x)>=3 else None

def same_x(paths,spacing=.1):
    if any(p is None or len(p)<3 for p in paths):return None
    lo=max(p[0,0] for p in paths);hi=min(p[-1,0] for p in paths)
    x=np.arange(np.ceil(lo/spacing)*spacing,hi+1e-8,spacing)
    return [np.column_stack((x,np.interp(x,p[:,0],p[:,1]))) for p in paths] if len(x)>=3 else None

def footprint_distance(path,obstacle):
    """Distance to an axis-aligned observed footprint, with dense segments."""
    delta_path=np.diff(path,axis=0)
    count=max(2,int(np.ceil(np.max(np.linalg.norm(delta_path,axis=1))/.025))+1)
    q=(path[:-1,None,:]+np.linspace(0.,1.,count)[None,:,None]*delta_path[:,None,:]).reshape(-1,2)
    delta=np.maximum(abs(q-[obstacle.x,obstacle.y])-[obstacle.depth/2,obstacle.width/2],0.)
    return float(np.min(np.linalg.norm(delta,axis=1)))

def project_point(path,point):
    a=path[:-1];v=np.diff(path,axis=0);u=np.clip(np.sum((point-a)*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-9),0,1)
    q=a+u[:,None]*v;i=int(np.argmin(np.linalg.norm(q-point,axis=1)));t=v[i]/np.linalg.norm(v[i]);n=np.array([-t[1],t[0]])
    return float(progress(path)[i]+u[i]*np.linalg.norm(v[i])),float((point-q[i])@n),i


def normal_distances(path,wall,side,where=None):
    _,normals=basis(path)
    # Subset the queries only after computing tangents on the complete path.
    if where is not None:path=path[where];normals=normals[where]
    v=np.diff(wall,axis=0);q=wall[None,:-1,:]-path[:,None,:]
    cross=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
    det=cross(normals[:,None,:],v[None,:,:]);safe=abs(det)>1e-9
    d=np.divide(cross(q,v[None,:,:]),det,out=np.full(det.shape,np.nan),where=safe)
    u=np.divide(cross(q,normals[:,None,:]),det,out=np.full(det.shape,np.nan),where=safe)
    good=safe&(u>=-1e-6)&(u<=1+1e-6)&(side*d>0)
    return side*np.min(np.where(good,side*d,np.inf),axis=1)


def observed_support(path,left,right):
    """Same finite-wall predicate, avoiding unused normal intersections."""
    common=x_ordered(left) and x_ordered(right)
    common=common&(path[:,0]>=max(left[0,0],right[0,0]))&(path[:,0]<=min(left[-1,0],right[-1,0]))
    interior=common&(path[:,1]>np.interp(path[:,0],right[:,0],right[:,1]))&(path[:,1]<np.interp(path[:,0],left[:,0],left[:,1]))
    good=interior.copy();missing=~interior
    if missing.any():
        a=normal_distances(path,left,1,missing);b=normal_distances(path,right,-1,missing)
        good[missing]=np.isfinite(a)&np.isfinite(b)
    return good


def _supported_prefix_once(path,left,right):
    """Keep cross-sections supported by finite walls, including curved ends."""
    if path is None or left is None or right is None:return None
    good=observed_support(path,left,right)
    indices=np.flatnonzero(good)
    if not len(indices):return None
    start=indices[0];end=np.flatnonzero(~good[start:]);result=path[start:start+end[0]] if len(end) else path[start:]
    return result if len(result)>=3 else None


def supported_prefix(path,left,right):
    # Cropping changes endpoint tangents. Recheck the cropped polyline until
    # its own normals are supported; each iteration removes observed ends.
    if path is None:return None
    current=path
    for _ in range(len(path)):
        result=_supported_prefix_once(current,left,right)
        if result is None:return None
        if len(result)==len(current):return result
        current=result
    return current


def drive_prefix(path):
    if path is None:return None
    steep=np.flatnonzero(abs(np.diff(path[:,1]))>=.35)
    return path[:steep[0]+1] if len(steep) else path


def corridor_membership(points,center,left,right):
    """Finite normal sections, not global-Y bands, for LiDAR classification."""
    count=len(points)
    if not count:return np.zeros(0,dtype=bool),np.zeros(0,dtype=bool),np.zeros(0)
    tangents,normals=basis(center);s=progress(center)
    dl=normal_distances(center,left,1);dr=normal_distances(center,right,-1)
    common=x_ordered(left) and x_ordered(right)
    common=common&(center[:,0]>=max(left[0,0],right[0,0]))&(center[:,0]<=min(left[-1,0],right[-1,0]))
    # At a finite observed endpoint, a normal may leave the measured wall.
    # The existing common-X section is a conservative endpoint alternative.
    yl=np.interp(center[:,0],left[:,0],left[:,1]);yr=np.interp(center[:,0],right[:,0],right[:,1])
    dl=np.where(~np.isfinite(dl)&common,(yl-center[:,1])*normals[:,1],dl)
    dr=np.where(~np.isfinite(dr)&common,(yr-center[:,1])*normals[:,1],dr)
    usable=np.isfinite(dl)&np.isfinite(dr)&(dl>0)&(dr<0)
    if np.count_nonzero(usable)<3:return np.zeros(count,dtype=bool),np.zeros(count,dtype=bool),np.zeros(count)
    a=center[:-1];v=np.diff(center,axis=0);length=np.linalg.norm(v,axis=1)
    delta=points[:,None,:]-a[None,:,:]
    u=np.clip(np.sum(delta*v[None,:,:],axis=2)/np.maximum(length*length,1e-9),0,1)
    projections=a[None,:,:]+u[:,:,None]*v[None,:,:]
    index=np.argmin(np.sum((points[:,None,:]-projections)**2,axis=2),axis=1);row=np.arange(count)
    nearest=projections[row,index];t=v[index]/length[index,None];normal=np.column_stack((-t[:,1],t[:,0]))
    local_s=s[index]+u[row,index]*length[index];lateral=np.sum((points-nearest)*normal,axis=1)
    lower=np.interp(local_s,s[usable],dr[usable]);upper=np.interp(local_s,s[usable],dl[usable])
    support=(local_s>=s[usable][0]-1e-6)&(local_s<=s[usable][-1]+1e-6)
    support&=((points-center[0])@tangents[0]>=-.02)&((points-center[-1])@tangents[-1]<=.02)
    return support,support&(lateral>lower)&(lateral<upper),upper-lower


def boundary_ok(path,horizon=5.):
    return bool(isinstance(path,np.ndarray) and path.ndim==2 and path.shape[1]==2 and len(path)>=3 and np.isfinite(path).all() and np.all(path[:,0]>0) and np.all(path[:,0]<=horizon+1e-5) and np.all(np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-8))

def x_ordered(path):
    return bool(path is not None and len(path)>=3 and np.all(np.diff(path[:,0])>1e-5))

def wall_tangent(point,wall):
    a=wall[:-1];v=np.diff(wall,axis=0);length=np.sum(v*v,axis=1)
    u=np.clip(np.sum((point-a)*v,axis=1)/np.maximum(length,1e-12),0,1)
    index=int(np.argmin(np.linalg.norm(a+u[:,None]*v-point,axis=1)))
    return v[index]/max(np.sqrt(length[index]),1e-9)
