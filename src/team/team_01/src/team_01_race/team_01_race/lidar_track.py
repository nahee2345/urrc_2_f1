"""Observed wall prefixes with continuity association and short width memory."""
from dataclasses import dataclass
import math
import numpy as np
from .lidar_filter import groups,polyline_distance
from .navigation import valid_path,valid_boundary
from .corridor_geometry import basis,normal_hit,resample_x,same_x,progress,supported_prefix,drive_prefix,normal_distances,boundary_ok

@dataclass
class LidarTrackConfig:
    enabled: bool=True
    horizon_m: float=5.
    min_wall_points: int=12
    min_wall_span_m: float=1.0
    max_fit_error_m: float=.12
    min_forward_observed_m: float=1.0
    max_near_gap_m: float=2.5
    cluster_gap_m: float=.18
    single_wall_enabled: bool=True
    width_memory_sec: float=2.
    width_relative_tolerance: float=.3

class LidarTrack:
    def __init__(self,config=None,*,batch_smoothing=False):
        self.batch_smoothing=batch_smoothing
        self.c=config or LidarTrackConfig();self.width=None;self.width_time=-math.inf;self.previous=None;self.last_stamp=-math.inf
        if min(self.c.min_wall_points,self.c.min_wall_span_m,self.c.max_fit_error_m,self.c.min_forward_observed_m,self.c.width_memory_sec)<=0:raise ValueError('invalid LiDAR track config')
        self.debug={};self.wall_indices=[];self.parametric_accepted=0
    def _smooth(self,x,values,stations,count):
        x=np.asarray(x,dtype=float);values=np.asarray(values,dtype=float)
        if values.ndim==1:values=values[:,None]
        ids=np.argsort(abs(x[None,:]-stations[:,None]),axis=1)[:,:min(count,len(x))]
        if not self.batch_smoothing:
            return np.array([[np.polyval(np.polyfit(x[index]-value,values[index,j],2),0.)
                for j in range(values.shape[1])] for value,index in zip(stations,ids)])
        # Same nearest stations/order-2 fit as the scalar path, batched only
        # for the Depth preview. Normal LiDAR fitting keeps its original math.
        dx=x[ids]-stations[:,None]
        design=np.stack((dx*dx,dx,np.ones_like(dx)),axis=2)
        gram=np.einsum('nki,nkj->nij',design,design)
        rhs=np.einsum('nki,nkj->nij',design,values[ids])
        return np.linalg.solve(gram,rhs)[:,-1,:]

    def note_width(self,width,age,now):
        if math.isfinite(width) and .55<=width<=4 and 0<=age<=self.c.width_memory_sec:
            self.width=width;self.width_time=now-age
    def _parametric(self,q,minimum_span=None):
        minimum_span=self.c.min_wall_span_m if minimum_span is None else minimum_span
        # A wall crossing the field of view can be vertical in vehicle X.
        # Fit measured arc progress instead of forcing Y=f(X).
        q=q.copy()
        score=lambda p:float(p[0]+.1*np.linalg.norm(p))
        if score(q[-1])<score(q[0]):q=q[::-1]
        keep=[q[0]]
        for point in q[1:]:
            if np.linalg.norm(point-keep[-1])>=.03:keep.append(point)
        q=np.asarray(keep)
        if len(q)<6:return None,math.inf
        arc=progress(q)
        if arc[-1]<minimum_span:return None,math.inf
        sample=np.arange(0.,arc[-1]+1e-8,.1)
        measured=np.column_stack([np.interp(sample,arc,q[:,j]) for j in range(2)])
        wall=self._smooth(sample,measured,sample,7);error=float(np.percentile(np.linalg.norm(wall-measured,axis=1),90))
        wall=wall[(wall[:,0]>0)&(wall[:,0]<=self.c.horizon_m)]
        if boundary_ok(wall,self.c.horizon_m) and error<=self.c.max_fit_error_m:
            self.parametric_accepted+=1
            return wall,error
        return None,error

    def _boundary(self,q,minimum_span=None):
        minimum_span=self.c.min_wall_span_m if minimum_span is None else minimum_span
        # Bin only measured forward support, then fit local neighbourhoods.
        x=np.arange(math.ceil(q[:,0].min()*10)/10,min(self.c.horizon_m,q[:,0].max())+1e-8,.1)
        samples=[]
        for xi in x:
            ys=q[abs(q[:,0]-xi)<.075,1]
            if len(ys) and np.ptp(ys)>.35:return self._parametric(q,minimum_span)
            if len(ys):samples.append((xi,float(np.median(ys))))
        a=np.asarray(samples)
        if len(a)<6 or progress(a)[-1]+1e-6<minimum_span:return self._parametric(q,minimum_span)
        gaps=np.flatnonzero(np.diff(a[:,0])>.35)
        if len(gaps):a=a[:gaps[0]+1]
        if len(a)<6:return self._parametric(q,minimum_span)
        y=self._smooth(a[:,0],a[:,1],a[:,0],9)[:,0]
        p=np.column_stack((a[:,0],y));error=float(np.percentile(polyline_distance(q,p),90))
        return (p,error) if error<=self.c.max_fit_error_m and valid_boundary(p,self.c.horizon_m) else self._parametric(q,minimum_span)
    def _join_wall_prefixes(self,alternatives):
        # Both candidates come from measured walls plus the same recent LiDAR
        # width. Join only overlapping, agreeing support; never bridge a gap.
        if len(alternatives)<2:return None
        near=min(alternatives,key=lambda v:v[2][0][0,0])
        far=max(alternatives,key=lambda v:v[2][0][-1,0])
        a,b=near[2][0],far[2][0]
        lo=max(a[0,0],b[0,0]);hi=min(a[-1,0],b[-1,0])
        if near[1]==far[1] or hi-lo<self.c.min_wall_span_m:return None
        overlap=np.arange(lo,hi+1e-8,.1)
        if np.max(abs(np.interp(overlap,a[:,0],a[:,1])-np.interp(overlap,b[:,0],b[:,1])))>self.c.max_fit_error_m:return None
        x=np.arange(max(a[0,0],.1),b[-1,0]+1e-8,.1)
        u=np.clip((x-lo)/(hi-lo),0.,1.);weight=u*u*(3-2*u)
        center=np.column_stack((x,(1-weight)*np.interp(x,a[:,0],a[:,1])+weight*np.interp(x,b[:,0],b[:,1])))
        _,normal=basis(center)
        left=center+.5*self.width*normal;right=center-.5*self.width*normal
        left=left[(left[:,0]>0)&(left[:,0]<=self.c.horizon_m)]
        right=right[(right[:,0]>0)&(right[:,0]<=self.c.horizon_m)]
        if not boundary_ok(left,self.c.horizon_m) or not boundary_ok(right,self.c.horizon_m):return None
        center=drive_prefix(supported_prefix(center,left,right))
        if center is None:return None
        return float(progress(center)[-1]),near[1],[center,left,right]

    def _split_connected_contour(self,points,ids,reference):
        """Separate near walls joined by a far transverse scan surface.

        In a sag/valley the scan can hit road between the two walls, leaving
        one continuous U-shaped contour. Do not fit that whole contour as one
        wall. Keep the transverse returns unlabelled for obstacle checking.
        The same applies to an actual closed corridor: its barrier is retained.
        """
        q=points[ids]
        if len(q)<3*self.c.min_wall_points:return [ids]
        ends=q[[0,-1]]
        centre=np.interp(ends[:,0],reference[:,0],reference[:,1]) if valid_path(reference) else 0.
        offsets=ends[:,1]-centre
        if (np.max(ends[:,0])>self.c.max_near_gap_m or
            offsets[0]*offsets[1]>=0 or np.min(abs(offsets))<.25 or
            np.max(q[:,0])-np.max(ends[:,0])<self.c.min_wall_span_m):return [ids]
        def split(indices):
            cloud=points[indices];delta=cloud[-1]-cloud[0];length=np.linalg.norm(delta)
            if len(indices)<3 or length<1e-9:return [indices]
            v=cloud-cloud[0]
            residual=abs(v[:,0]*delta[1]-v[:,1]*delta[0])/length
            k=int(np.argmax(residual))
            if residual[k]<=self.c.max_fit_error_m or k==0 or k==len(indices)-1:return [indices]
            return split(indices[:k+1])+split(indices[k:])
        parts=split(ids)
        self.debug['connected_contour_splits']+=int(len(parts)>1)
        return parts

    def update(self,points,indices,stamp,now,reference=None):
        self.debug=dict(raw_points=len(points),left_wall_points=0,right_wall_points=0,left_wall_valid=False,right_wall_valid=False,track_width=self.width,forward_observed_length=0.,lidar_path_confidence=0.,reason='no_wall_support',connected_contour_splits=0,intervening_unfitted_wall_groups=0);self.wall_indices=[]
        if not self.c.enabled or stamp<=self.last_stamp:return None
        self.last_stamp=stamp;self.parametric_accepted=0;candidates={1:[],-1:[]}
        ref=reference if valid_path(reference) else self.previous
        wall_groups=[]
        for ids in groups(points,indices,self.c.cluster_gap_m):
            # Missing rays at mesh seams need not create a physical gap.
            # Measured metric continuity and the downstream fit remain required.
            if wall_groups and np.linalg.norm(points[ids[0]]-points[wall_groups[-1][-1]])<=self.c.cluster_gap_m:
                wall_groups[-1]=np.r_[wall_groups[-1],ids]
            else:
                # An isolated return that cannot form a wall must not hide a
                # metric-contiguous wall seam on its two sides. It remains in
                # the original scan for the independent obstacle classifier.
                while wall_groups and len(wall_groups[-1])<self.c.min_wall_points:
                    wall_groups.pop()
                    self.debug['intervening_unfitted_wall_groups']+=1
                if wall_groups and np.linalg.norm(points[ids[0]]-points[wall_groups[-1][-1]])<=self.c.cluster_gap_m:
                    wall_groups[-1]=np.r_[wall_groups[-1],ids]
                else:wall_groups.append(ids)
        wall_groups=[part for ids in wall_groups for part in self._split_connected_contour(points,ids,ref)]
        for ids in wall_groups:
            q=points[ids]
            if len(q)<self.c.min_wall_points:continue
            minimum_span=self.c.min_wall_span_m;trusted_side=None
            span=float(np.linalg.norm(np.ptp(q,axis=0)))
            if span<minimum_span:
                # Six 10cm fit stations can validate a previously associated
                # wall locally. This is not permission to bootstrap width from
                # an arbitrary compact object or renew width with one wall.
                if (span<.5 or self.width is None or now-self.width_time>self.c.width_memory_sec or not valid_path(ref)):continue
                _,normals=basis(ref)
                matches={side:float(np.percentile(polyline_distance(q,ref+side*.5*self.width*normals),90)) for side in (1,-1)}
                trusted_side=min(matches,key=matches.get)
                if matches[trusted_side]>self.c.max_fit_error_m:continue
                minimum_span=.5
            # Associate using the near part relative to the recent centre path;
            # a wall may cross vehicle Y=0 farther into a tight corner.
            near=q[q[:,0]<=np.percentile(q[:,0],25)+.2]
            centre=np.interp(near[:,0],ref[:,0],ref[:,1]) if valid_path(ref) else 0.
            offset=float(np.median(near[:,1]-centre));side=trusted_side if trusted_side is not None else 1 if offset>0 else -1
            if abs(offset)<.25:continue
            wall,error=self._boundary(q,minimum_span)
            if wall is None:continue
            candidates[side].append((wall,error,len(q),indices[ids]))
        selected={}
        for side,options in candidates.items():
            if options:
                # Prefer continuous near support associated with the expected
                # wall, rather than blindly taking all points of one Y sign.
                def score(v):
                    p=v[0];near=p[:min(8,len(p))];refy=np.interp(near[:,0],ref[:,0],ref[:,1]) if valid_path(ref) else 0.
                    expected=(self.width or 2.4)/2
                    return abs(float(np.median(abs(near[:,1]-refy)))-expected)+v[1]-.05*min(v[0][-1,0]-v[0][0,0],4.)
                selected[side]=min(options,key=score)
        self.debug['parametric_wall_candidates']=self.parametric_accepted
        observed=(1 in selected,-1 in selected)
        if not any(observed):self.previous=None;return None
        for side,key in [(1,'left_wall'),(-1,'right_wall')]:
            if side in selected:
                self.debug[key+'_points']=selected[side][2];self.debug[key+'_valid']=True
                self.wall_indices.extend(selected[side][3].tolist())
        residual=max(v[1] for v in selected.values())
        aligned=None;confidence=0.;both=False
        if all(observed):
            pair=same_x([resample_x(selected[1][0],horizon=self.c.horizon_m),resample_x(selected[-1][0],horizon=self.c.horizon_m)])
            if pair is not None:
                left,right=pair;initial=(left+right)/2;_,normals=basis(initial);centres=[];widths=[]
                for point,n in zip(initial,normals):
                    # Intersect the complete measured walls. Cropping both to
                    # equal X first removes the outer wall needed in a bend.
                    a=normal_hit(point,n,selected[1][0],1);b=normal_hit(point,n,selected[-1][0],-1)
                    if a is not None and b is not None:centres.append(point+.5*(a+b)*n);widths.append(a-b)
                short_associated=any(progress(v[0])[-1]<self.c.min_wall_span_m for v in selected.values())
                # Revalidate an existing width from locally associated wall
                # sections; bootstrapping still needs six full sections. The
                # short wall already passed point count and prior-band checks.
                sections=3 if short_associated and self.width is not None and now-self.width_time<=self.c.width_memory_sec else 6
                if len(centres)>=sections:
                    width=float(np.median(widths))
                    consistent=.55<=min(widths) and max(widths)<=4 and (self.width is None or now-self.width_time>self.c.width_memory_sec or abs(width-self.width)<=self.width*self.c.width_relative_tolerance)
                    if consistent:
                        self.width=width if self.width is None else .8*self.width+.2*width;self.width_time=now
                        aligned=same_x([resample_x(centres,horizon=self.c.horizon_m),left,right]);confidence=.9;both=True
        if all(observed) and self.width is not None and now-self.width_time<=self.c.width_memory_sec:
            # Same-X correspondence underestimates visible length in a bend.
            # Extend only where normal sections intersect BOTH measured walls.
            left_full,right_full=selected[1][0],selected[-1][0];extended=[]
            for side,wall in ((1,left_full),(-1,right_full)):
                _,normal=basis(wall)
                candidate=drive_prefix(supported_prefix(resample_x(wall-side*.5*self.width*normal,horizon=self.c.horizon_m),left_full,right_full))
                if candidate is None:continue
                widths=normal_distances(candidate,left_full,1)-normal_distances(candidate,right_full,-1)
                finite=widths[np.isfinite(widths)]
                if len(finite)>=6 and np.all(abs(finite-self.width)<=self.width*self.c.width_relative_tolerance):extended.append(candidate)
            if extended:
                best=max(extended,key=lambda p:progress(p)[-1])
                if aligned is None or progress(best)[-1]>progress(aligned[0])[-1]:
                    aligned=[best,left_full,right_full];both=True;confidence=.9
                    measured=normal_distances(best,left_full,1)-normal_distances(best,right_full,-1)
                    self.width=.8*self.width+.2*float(np.median(measured[np.isfinite(measured)]));self.width_time=now
        # The inside wall can disappear after less than one metre while the
        # outside wall remains long and continuous. Use the latter with recent
        # validated width rather than discarding all observed geometry.
        enough=aligned is not None and progress(aligned[0])[-1]>=self.c.min_forward_observed_m
        if self.c.single_wall_enabled and self.width is not None and now-self.width_time<=self.c.width_memory_sec:
            alternatives=[]
            for side,entry in selected.items():
                wall=entry[0];_,normals=basis(wall)
                center=drive_prefix(resample_x(wall-side*.5*self.width*normals,horizon=self.c.horizon_m))
                if center is None:continue
                # Intersections can only shorten this candidate. Avoid doing
                # them twice on normal full-prefix both-wall frames.
                if (enough and center[0,0]>=aligned[0][0,0] and
                    progress(center)[-1]<progress(aligned[0])[-1]+self.c.min_wall_span_m):continue
                opposite=wall-side*self.width*normals
                opposite=opposite[(opposite[:,0]>0)&(opposite[:,0]<=self.c.horizon_m)]
                if not boundary_ok(opposite,self.c.horizon_m):continue
                left=wall if side==1 else opposite;right=opposite if side==1 else wall
                center=drive_prefix(supported_prefix(center,left,right))
                candidate=[center,left,right] if center is not None else None
                if candidate is not None:alternatives.append((progress(candidate[0])[-1],side,candidate))
            joined=self._join_wall_prefixes(alternatives)
            if joined is not None:
                alternatives.append(joined)
            if alternatives:
                span,side,candidate=max(alternatives,key=lambda a:a[0])
                common_span=progress(aligned[0])[-1] if aligned is not None else 0.
                # An occluder can leave a valid but short common prefix while
                # the opposite wall remains measured all the way forward.
                # Reuse width memory on that wall; do not wait until the common
                # prefix is completely lost and the pass entry is behind us.
                longer=span>=common_span+self.c.min_wall_span_m
                if not enough or longer:
                    aligned=candidate;observed=(side==1,side==-1);both=False
                    self.debug['stitched_wall_prefix']=joined is not None and candidate is joined[2]
                    confidence=.68*max(.65,1-.35*(now-self.width_time)/self.c.width_memory_sec)
        if aligned is None:self.debug['reason']='no_aligned_observed_prefix';return None
        center,left,right=aligned
        length=float(progress(center)[-1])
        if not valid_path(center,self.c.horizon_m) or center[0,0]>self.c.max_near_gap_m or length<self.c.min_forward_observed_m:
            self.debug['reason']='insufficient_forward_prefix';return None
        confidence*=max(.8,1-residual/max(self.c.max_fit_error_m,1e-6)*.2)*min(1.,.7+.3*length/2.)
        self.previous=center.copy();self.debug.update(track_width=self.width,forward_observed_length=length,lidar_path_confidence=confidence,reason='both_walls' if both else 'single_wall_width_memory')
        return dict(path=center,left=left,right=right,confidence=confidence,observed=observed,light='UNKNOWN',reason=self.debug['reason'],path_source='LIDAR_PATH',track_width=self.width,track_width_age=now-self.width_time)
