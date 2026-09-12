"""Conservative observed-voxel landing checks. Unknown cells are never free.

This is a geometric gate, not a terrain semantics or dynamic-obstacle guarantee.
Geometry and sensor-frame configuration must be explicitly verified on the aircraft.
"""
import math
import threading
import time
import numpy as np

class LandingMap:
    resolution=.2
    def __init__(self):
        self.cells={}; self.heights={}; self.lock=threading.RLock(); self.last_stamp=None
    def key(self,p): return tuple(math.floor(float(v)/self.resolution) for v in p)
    def ingest(self,origin,points,stamp,now):
        if not all(math.isfinite(v) for v in origin): raise ValueError('invalid ray origin')
        if not 0<=now-stamp<=.5: raise ValueError('stale landing cloud')
        if self.last_stamp is not None and stamp<=self.last_stamp: return
        occupied=set(); free=set(); heights={}
        # A bounded subset can lose coverage, but may never invent it.
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[::max(1,math.ceil(len(points)/2000))]
        distances=np.linalg.norm(points-np.asarray(origin),axis=1)
        valid=np.isfinite(points).all(axis=1)&(distances>.3)&(distances<=8)
        points=points[valid]; distances=distances[valid]
        for p in points:
            k=self.key(p); occupied.add(k); heights.setdefault(k,[]).append(float(p[2]))
        if len(points):
            # Vectorized, bounded ray sampling keeps Python out of the per-ray-step loop.
            steps=np.arange(.2,8,.1)
            samples=np.asarray(origin)+(points-np.asarray(origin))[:,None,:]*(steps[None,:,None]/distances[:,None,None])
            samples=samples[steps[None,:]<(distances[:,None]-.35)]
            indices=np.unique(np.floor(samples/self.resolution).astype(np.int64),axis=0)
            free={tuple(int(v) for v in row) for row in indices}
        with self.lock:
            self.cells={k:v for k,v in self.cells.items() if now-v[1]<=3.}
            self.heights={k:v for k,v in self.heights.items() if k in self.cells}
            for k in free-occupied:
                # Never clear a recently observed obstacle using another sparse ray.
                if k not in self.cells or self.cells[k][0]==0: self.cells[k]=(0,stamp)
            for k in occupied:
                self.cells[k]=(1,stamp); self.heights[k]=(min(heights[k]),max(heights[k]))
            self.last_stamp=stamp
    def assess(self,center,hover_z,radius,reference,now):
        if not all(math.isfinite(v) for v in (*center,hover_z,radius,reference,now)) or radius<.5:
            raise ValueError('invalid landing geometry')
        with self.lock: cells=dict(self.cells); heights=dict(self.heights); stamp=self.last_stamp
        if stamp is None or not 0<=now-stamp<=.5: raise ValueError('landing observation unavailable/stale')
        r=self.resolution
        # Include the full square containing the footprint plus one voxel margin.
        x0,x1=math.floor((center[0]-radius)/r)-1,math.floor((center[0]+radius)/r)+1
        y0,y1=math.floor((center[1]-radius)/r)-1,math.floor((center[1]+radius)/r)+1
        ground=[]
        for x in range(x0,x1+1):
            for y in range(y0,y1+1):
                levels=[z for z in range(math.floor((reference-.5)/r),math.ceil((reference+.15)/r))
                        if cells.get((x,y,z),(None,0))[0]==1 and 0<=now-cells[(x,y,z)][1]<=1.]
                if len(levels)!=1: raise ValueError('ground coverage missing/ambiguous')
                z=levels[0]; low,high=heights.get((x,y,z),(float('nan'),float('nan')))
                if not math.isfinite(low+high) or high-low>.04: raise ValueError('rough ground or missing raw surface evidence')
                ground.append(((x+.5)*r,(y+.5)*r,(low+high)/2))
                # Require observed free space through the entire descent column and rotor headroom.
                for air_z in range(z+1,math.ceil((hover_z+.35)/r)+1):
                    state,when=cells.get((x,y,air_z),(None,0))
                    if state!=0 or not 0<=now-when<=1.:
                        raise ValueError('descent column obstructed or unobserved')
        p=np.array(ground); plane=np.linalg.lstsq(np.column_stack((p[:,:2],np.ones(len(p)))),p[:,2],rcond=None)[0]
        residual=np.max(np.abs(p[:,2]-np.column_stack((p[:,:2],np.ones(len(p)))).dot(plane)))
        if math.hypot(*plane[:2])>math.tan(math.radians(5)) or np.ptp(p[:,2])>.12 or residual>.06:
            raise ValueError('landing surface slope/roughness exceeds limit')
        return dict(center_world=[center[0],center[1],float(np.mean(p[:,2]))],radius_m=radius,
                    observed_stamp=stamp,ground_cells=len(ground),max_residual_m=float(residual))
