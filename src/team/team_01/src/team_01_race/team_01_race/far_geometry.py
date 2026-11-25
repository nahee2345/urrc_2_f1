"""Distance-resolved curvature from measured wall prefixes, never map geometry."""
import math
import numpy as np


class PixelSupport:
    """Exact distinct-pixel range queries without repeated dense BEV sorting."""
    def __init__(self,wall,pixels):
        self.x=np.asarray(wall)[:,0]
        uv=np.rint(np.asarray(pixels)).astype(int)
        _,index=np.unique(uv,axis=0,return_inverse=True)
        self.tables=[[1<<int(i) for i in index]];step=1
        while 2*step<=len(index):
            previous=self.tables[-1]
            self.tables.append([previous[i]|previous[i+step] for i in range(len(index)-2*step+1)])
            step*=2
    def counts(self,lower,upper):
        lo=np.searchsorted(self.x,lower,'left');hi=np.searchsorted(self.x,upper,'right')
        size=hi-lo;levels=np.floor(np.log2(np.maximum(size,1))).astype(int)
        return np.array([0 if length<=0 else (self.tables[k][i]|self.tables[k][j-(1<<k)]).bit_count()
                         for i,j,length,k in zip(lo,hi,size,levels)])


def independent_pixels(wall,pixels,positions,lower,upper):
    support=pixels if isinstance(pixels,PixelSupport) else PixelSupport(wall,pixels)
    return support.counts(lower,upper)


def observation_windows(wall,pixels,positions,start,end,initial=.75):
    # Grow only inside actual observed support until enough independent image
    # samples constrain a local fit. The residual threshold is never relaxed.
    pixels=pixels if isinstance(pixels,PixelSupport) else PixelSupport(wall,pixels)
    span=np.full(len(positions),initial,dtype=float)
    for _ in range(14):
        lower=np.maximum(start,positions-span);upper=np.minimum(end,positions+span)
        ready=independent_pixels(wall,pixels,positions,lower,upper)>=6
        grow=(~ready)&(span<7.5)
        if not grow.any():break
        span[grow]=np.minimum(7.5,span[grow]+.5)
    return np.maximum(start,positions-span),np.minimum(end,positions+span)


def fit_windows(wall,positions,lower,upper,robust=False,fallback=False,pixels=None):
    """Batched local least squares; no Python fit loop blocking near frames."""
    wall=np.asarray(wall,dtype=float);wall=wall[np.argsort(wall[:,0])]
    dx=wall[None,:,0]-positions[:,None]
    mask=(wall[None,:,0]>=lower[:,None])&(wall[None,:,0]<=upper[:,None])
    if fallback:
        missing=mask.sum(axis=1)<5
        nearest=np.argsort(abs(dx[missing]),axis=1)[:,:min(7,len(wall))]
        rows=np.flatnonzero(missing)
        mask[rows]=False
        mask[rows[:,None],nearest]=True
    design=np.stack((np.ones_like(dx),dx,dx*dx),axis=2)
    def solve(weights):
        matrix=np.einsum('nmi,nmj,nm->nij',design,design,weights)
        target=np.einsum('nmi,m,nm->ni',design,wall[:,1],weights)
        rank_ok=abs(np.linalg.det(matrix))>1e-10
        matrix[~rank_ok]=np.eye(3);target[~rank_ok]=0.
        return np.linalg.solve(matrix,target[:,:,None])[:,:,0],rank_ok
    weights=mask.copy();coef,rank_ok=solve(weights)
    if robust:
        for _ in range(2):
            errors=abs(wall[None,:,1]-np.einsum('nmi,ni->nm',design,coef))
            median=np.nanmedian(np.where(weights,errors,np.nan),axis=1)
            keep=weights&(errors<=np.maximum(.04,3*median)[:,None])
            enough=keep.sum(axis=1)>=3;weights[enough]=keep[enough]
            coef,rank_ok=solve(weights)
    errors=abs(wall[None,:,1]-np.einsum('nmi,ni->nm',design,coef))
    count=mask.sum(axis=1);index=np.maximum(0,np.ceil(.9*(count-1)).astype(int))
    sorted_error=np.sort(np.where(mask,errors,np.inf),axis=1)
    p90=sorted_error[np.arange(len(positions)),index]
    minimum=np.min(np.where(mask,wall[None,:,0],np.inf),axis=1)
    maximum=np.max(np.where(mask,wall[None,:,0],-np.inf),axis=1)
    gap=np.any(mask[:,:-1]&mask[:,1:]&(np.diff(wall[:,0])[None,:]>.4),axis=1)
    valid=rank_ok&(count>=5)&(maximum-minimum>=.5)&~gap&(p90<=.12)
    if pixels is not None:
        support=pixels if isinstance(pixels,PixelSupport) else PixelSupport(wall,pixels)
        if len(support.x)!=len(wall):valid[:]=False
        else:
            # Warping one source pixel into many BEV rows does not create
            # independent tangent/curvature observations.
            valid &= support.counts(lower,upper)>=6
    return coef,valid


def local_boundary(samples,x,source_pixels=None):
    """Local quadratic fits preserve observed bend onset without extrapolation."""
    a=np.asarray(samples,dtype=float)
    if a.ndim!=2 or len(a)<3:return np.full(len(x),np.nan)
    order=np.argsort(a[:,0]);a=a[order]
    if source_pixels is not None:source_pixels=np.asarray(source_pixels)[order]
    anchors=np.unique(np.r_[x[::3],x[-1]])
    span=np.where(anchors<5.,.8,1.5)
    lower,upper=anchors-span,anchors+span
    if source_pixels is not None:
        source_pixels=PixelSupport(a,source_pixels)
        lower,upper=observation_windows(a,source_pixels,anchors,a[:,0].min(),a[:,0].max(),1.5)
    coef,_=fit_windows(a,anchors,lower,upper,robust=True,fallback=True)
    return np.interp(x,anchors,coef[:,0])


def analyze_preview(path,left_samples=None,right_samples=None,confidence=0.,source='CAMERA_PREVIEW',horizon=25.,source_pixels=None):
    empty=dict(supported_preview_m=0.,curvature_supported_preview_m=0.,preview_source=source,
        preview_confidence=float(confidence),far_left_wall_valid_length=0.,far_right_wall_valid_length=0.,
        far_centerline_valid_length=0.,far_curvature_supported_length=0.,
        distance_to_first_significant_curvature=None,segment_curvatures=[],segment_heading_changes=[],
        segment_support_ratios=[],curvature_samples=[],camera_far_valid=False,lidar_far_valid=False,fused_far_valid=False)
    p=np.asarray(path,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<5 or not np.isfinite(p).all() or np.any(np.diff(p[:,0])<=0):return empty
    left=np.asarray(left_samples if left_samples is not None else [],dtype=float)
    right=np.asarray(right_samples if right_samples is not None else [],dtype=float)
    if any(a.ndim!=2 or a.shape[1]!=2 or len(a)<5 or not np.isfinite(a).all() for a in (left,right)):return empty
    li=np.argsort(left[:,0]);ri=np.argsort(right[:,0])
    left=left[li];right=right[ri]
    if source_pixels is not None:source_pixels=(PixelSupport(left,np.asarray(source_pixels[0])[li]),PixelSupport(right,np.asarray(source_pixels[1])[ri]))
    start=max(left[0,0],right[0,0],p[0,0]);end=min(left[-1,0],right[-1,0],p[-1,0],horizon)
    empty.update(supported_preview_m=float(end),far_left_wall_valid_length=float(left[-1,0]),
        far_right_wall_valid_length=float(right[-1,0]),far_centerline_valid_length=float(end))
    x=np.arange(math.ceil(start*10)/10,end-.75+1e-8,.1)
    if len(x)==0:return empty
    first=np.maximum(start,x-.75);last=x+.75
    lower,upper=first-.15,last+.15
    if source_pixels is not None:
        ll,lh=observation_windows(left,source_pixels[0],x,start,end)
        rl,rh=observation_windows(right,source_pixels[1],x,start,end)
        lower=np.minimum(ll,rl);upper=np.maximum(lh,rh)
    cl,vl=fit_windows(left,x,lower,upper,pixels=source_pixels[0] if source_pixels is not None else None)
    cr,vr=fit_windows(right,x,lower,upper,pixels=source_pixels[1] if source_pixels is not None else None)
    offsets=np.column_stack((first-x,np.zeros(len(x)),last-x))
    def evaluate(coef):return coef[:,0,None]+coef[:,1,None]*offsets+coef[:,2,None]*offsets**2
    yl,yr=evaluate(cl),evaluate(cr);y=(yl+yr)/2
    u=np.column_stack((x-first,y[:,1]-y[:,0]));v=np.column_stack((last-x,y[:,2]-y[:,1]))
    denominator=np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1)*np.linalg.norm(u+v,axis=1)
    usable=vl&vr&(x-first>=.25)&(denominator>1e-9)
    curvature=2*(u[:,0]*v[:,1]-u[:,1]*v[:,0])/np.maximum(denominator,1e-9)
    heading=np.arctan2(y[:,2]-y[:,0],last-first)
    widths=np.column_stack((x-first,last-x))
    hl=np.arctan2(np.diff(yl,axis=1),widths);hr=np.arctan2(np.diff(yr,axis=1),widths)
    tl,tr=hl[:,1]-hl[:,0],hr[:,1]-hr[:,0]
    agreement=~((tl*tr<0)&(np.minimum(abs(tl),abs(tr))>math.radians(2.)))
    samples=[dict(distance=float(x[i]),curvature=float(curvature[i]),heading=float(heading[i]),wall_agreement=bool(agreement[i])) for i in np.flatnonzero(usable)]
    if not samples:return empty
    # Preserve distance-local supported runs. A small noisy near patch must
    # not silently discard all independently measured far wall geometry.
    runs=[];run=[]
    for sample in samples:
        if run and sample['distance']-run[-1]['distance']>.2:
            runs.append(run);run=[]
        run.append(sample)
    if run:runs.append(run)
    runs=[run for run in runs if run[-1]['distance']-run[0]['distance']>=.8-1e-8]
    if not runs:return empty
    samples=[sample for run in runs for sample in run]
    empty['curvature_support_intervals_m']=[[run[0]['distance'],run[-1]['distance']] for run in runs]
    distances=np.array([s['distance'] for s in samples]);curv=np.array([s['curvature'] for s in samples]);heading=np.unwrap([s['heading'] for s in samples])
    significant=None
    for i in range(len(samples)-5):
        j=np.searchsorted(distances,distances[i]+.8)
        if j>=len(samples):break
        if np.max(np.diff(distances[i:j+1]))>.2:continue
        ks=curv[i:j+1];hs=heading[i:j+1]
        sustained=(np.count_nonzero(abs(ks)>=.04)>=6 and np.mean(np.sign(ks)==np.sign(np.median(ks)))>=.8)
        accumulated=abs(float(hs[-1]-hs[0]))>=math.radians(3.)
        if sustained and accumulated and all(s['wall_agreement'] for s in samples[i:j+1]):
            significant=float(distances[i]);break
    segments=[];changes=[];ratios=[]
    for lo in range(0,25,5):
        mask=(distances>=lo)&(distances<lo+5)
        values=abs(curv[mask]);h=heading[mask]
        ratio=min(1.,float(mask.sum())/50.)
        segments.append(dict(start_m=lo,end_m=lo+5,median=float(np.median(values)) if len(values) else None,p90=float(np.percentile(values,90)) if len(values) else None))
        changes.append(float(math.degrees(h[-1]-h[0])) if len(h)>1 else None);ratios.append(ratio)
    empty.update(curvature_supported_preview_m=float(distances[-1]),far_curvature_supported_length=float(distances[-1]),
        distance_to_first_significant_curvature=significant,segment_curvatures=segments,
        segment_heading_changes=changes,segment_support_ratios=ratios,curvature_samples=samples,
        camera_far_valid=source=='CAMERA_PREVIEW',lidar_far_valid=source=='LIDAR_PREVIEW',fused_far_valid=source=='FUSED_PREVIEW')
    return empty
