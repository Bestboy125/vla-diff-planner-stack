"""Pure, bounded first-batch target skill contract and geometry."""
import math
import re

def validate(spec):
    if not isinstance(spec, dict) or not isinstance(spec.get('task_id'),str) or not spec['task_id']:
        raise ValueError('missing task_id')
    if spec.get('action') not in ('APPROACH','RETREAT','TURN','PASS','ROTATE','LAND'):
        raise ValueError('unsupported action')
    if not isinstance(spec.get('target_label'),str) or (spec['action']!='ROTATE' and re.fullmatch(r'[A-Za-z][A-Za-z-]{0,31}',spec['target_label']) is None):
        raise ValueError('target label must be one English word')
    if spec.get('selector') not in ('highest_confidence','left','center','right'):
        raise ValueError('invalid selector')
    result = {key:spec[key] for key in ('task_id','action','selector','target_label')}
    result['target_label']=result['target_label'].lower()
    for key,low,high in (('stand_off_m',1,5),('object_radius_m',0,3),('distance_m',.2,3),('max_travel_m',1,15)):
        value=spec.get(key)
        if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or not low<=value<=high:
            raise ValueError('invalid '+key)
        result[key]=float(value)
    if result['action']=='RETREAT' and result['distance_m']>result['max_travel_m']:
        raise ValueError('retreat distance exceeds travel budget')
    if result['action'] in ('PASS','LAND'):
        if spec.get('side') not in ('left','right'): raise ValueError('side must be left/right')
        result['side']=spec['side']
        value=spec.get('exit_distance_m',1.)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not .5<=value<=3:
            raise ValueError('invalid exit_distance_m')
        result['exit_distance_m']=float(value)
    if result['action']=='ROTATE':
        if spec.get('rotation_direction') not in ('clockwise','counterclockwise'): raise ValueError('invalid rotation_direction')
        laps=spec.get('rotation_laps'); rate=spec.get('rotation_rate_deg_s')
        if isinstance(laps,bool) or not isinstance(laps,int) or not 1<=laps<=3: raise ValueError('rotation_laps must be integer 1..3')
        if isinstance(rate,bool) or not isinstance(rate,(int,float)) or not math.isfinite(rate) or not 10<=rate<=45: raise ValueError('invalid rotation rate')
        result.update(rotation_direction=spec['rotation_direction'],rotation_laps=laps,rotation_rate_deg_s=float(rate))
    return result

def bearing(position, target):
    dx,dy=target[0]-position[0],target[1]-position[1]
    if math.hypot(dx,dy)<.1: raise ValueError('target bearing undefined at current position')
    return math.atan2(dy,dx)

def yaw_error(target, current):
    return math.atan2(math.sin(target-current),math.cos(target-current))

def approach_goal(position,target,clearance):
    distance=math.hypot(position[0]-target[0],position[1]-target[1])
    if distance<.1: raise ValueError('target too close')
    if distance<=clearance+.15: return None
    amount=min(1.0,distance-clearance)
    return (position[0]+amount*(target[0]-position[0])/distance,
            position[1]+amount*(target[1]-position[1])/distance,position[2])

def retreat_goal(position,target,distance):
    yaw=bearing(position,target)
    return (position[0]-distance*math.cos(yaw),position[1]-distance*math.sin(yaw),position[2])

def bounded_goal(position, target):
    distance=math.dist(position,target)
    if distance<=.15: return None
    scale=min(1.0,1.0/distance)
    return tuple(position[i]+scale*(target[i]-position[i]) for i in range(3))

def target_axes(start,target,side):
    a=bearing(start,target); u=(math.cos(a),math.sin(a)); sign=1 if side=='left' else -1
    return u,(-u[1]*sign,u[0]*sign)

def pass_route(start,target,side,clearance,exit_distance):
    if math.dist(start[:2],target[:2])<clearance+1:
        raise ValueError('too close to target to enter specified-side pass safely')
    u,v=target_axes(start,target,side)
    return [tuple(target[i]+along*u[i]+clearance*v[i] for i in (0,1))+(start[2],)
            for along in (-clearance,0,clearance+exit_distance)]

def landing_candidates(start,target,side,clearance):
    u,v=target_axes(start,target,side)
    return [tuple(target[i]+along*u[i]+offset*v[i] for i in (0,1))+(start[2],)
            for offset in (clearance,clearance+.5,clearance+1.) for along in (0.,-.5,.5)]

def segment_distance(point,start,end):
    delta=tuple(end[i]-start[i] for i in range(3)); norm=sum(x*x for x in delta)
    t=0 if norm<1e-9 else max(0,min(1,sum((point[i]-start[i])*delta[i] for i in range(3))/norm))
    return math.dist(point,tuple(start[i]+t*delta[i] for i in range(3)))

class FullRotation:
    """Unwrapped, signed odometry progress; opposite-direction motion subtracts."""
    def __init__(self,yaw,direction,laps,rate):
        self.start=self.last=yaw; self.sign=1 if direction=='counterclockwise' else -1
        self.goal=2*math.pi*laps; self.rate=math.radians(rate); self.measured=0.; self.commanded=0.
    def step(self,yaw,dt):
        delta=yaw_error(yaw,self.last)
        if not 0<dt<=.5 or abs(delta)>.5: raise ValueError('rotation odometry gap/jump')
        self.measured+=self.sign*delta; self.last=yaw
        if self.measured<-.2 or self.measured>self.goal+.3: raise ValueError('rotation direction/overshoot violated')
        self.commanded=min(self.goal,self.commanded+self.rate*dt,max(0,self.measured)+math.radians(20))
        return yaw_error(self.start+self.sign*self.commanded,0)
    def reached(self): return abs(self.measured-self.goal)<.05 and self.commanded>=self.goal-1e-6
