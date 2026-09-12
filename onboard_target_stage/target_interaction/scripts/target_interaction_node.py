#!/usr/bin/env python3
"""APPROACH / RETREAT / TURN; no arming or takeoff services, timer-driven actions."""
import json
import math
import os
import sys
import threading
import time
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from atomic_skill_executor.msg import ExecuteAtomicSkillAction, ExecuteAtomicSkillGoal
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String, Empty, Bool
sys.path.insert(0,os.path.dirname(__file__))
from target_geometry import validate, bearing, yaw_error, approach_goal, retreat_goal, bounded_goal
from target_geometry import pass_route, landing_candidates, target_axes, segment_distance, FullRotation
from landing_observer import LandingObserver

class TargetInteraction:
    def __init__(self):
        self.lock=threading.RLock(); self.active=None; self.odom=None; self.fcu=None; self.px4=None
        self.phase=None; self.ready=False; self.candidate=None
        self.enabled=bool(rospy.get_param('~execution_enabled',False))
        self.reference=float(rospy.get_param('~altitude_reference_z_m',0.0))
        self.min_altitude=float(rospy.get_param('~min_operating_altitude',.3))
        self.max_altitude=float(rospy.get_param('~max_operating_altitude',2.0))
        self.client=actionlib.SimpleActionClient('/atomic_skill_executor/execute',ExecuteAtomicSkillAction)
        self.status_pub=rospy.Publisher('~status',String,queue_size=20,latch=True)
        self.class_pub=rospy.Publisher('/semantic_raw_stereo_node/target_class_command',String,queue_size=1)
        self.inference_pub=rospy.Publisher('/semantic_raw_stereo_node/inference_enabled',Bool,queue_size=1)
        self.hover_pub=rospy.Publisher('/vla_hover_stop_to_planner',Empty,queue_size=1)
        self.rotation_pub=rospy.Publisher('/drone_0_traj_server/operator_rotation',PositionCommand,queue_size=1)
        self.land_pub=rospy.Publisher('/target_interaction/land_request',String,queue_size=1)
        self.landing_observer=LandingObserver()
        rospy.Subscriber('/vla_bridge/status',String,self.on_bridge_status,queue_size=20)
        rospy.Subscriber('~request',String,self.request,queue_size=1)
        rospy.Subscriber('~cancel',String,self.cancel,queue_size=1)
        rospy.Subscriber('/ekf/ekf_odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('/mavros/state',State,self.on_fcu,queue_size=1)
        rospy.Subscriber('/px4ctrl/fsm_state',String,self.on_px4,queue_size=1)
        rospy.Subscriber('/semantic_raw_stereo_node/target_class_status',String,self.on_class,queue_size=1)
        rospy.Subscriber('/semantic_raw_stereo_node/stable_target_world',PointStamped,
                         lambda msg:self.on_target(msg,False),queue_size=1)
        rospy.Subscriber('/semantic_raw_stereo_node/stable_coarse_target_world',PointStamped,
                         lambda msg:self.on_target(msg,True),queue_size=1)
        rospy.Timer(rospy.Duration(.1),self.tick)

    def on_odom(self,msg):
        with self.lock: self.odom=(msg,time.monotonic())
    def on_fcu(self,msg):
        with self.lock: self.fcu=(msg,time.monotonic())
    def on_px4(self,msg):
        with self.lock: self.px4=(msg.data,time.monotonic())

    def flight(self):
        now=time.monotonic()
        if not self.odom or now-self.odom[1]>.3: raise ValueError('odometry unavailable/stale')
        msg=self.odom[0]; stamp_age=(rospy.Time.now()-msg.header.stamp).to_sec()
        if msg.header.frame_id!='world' or not -.05<=stamp_age<=.3: raise ValueError('invalid odometry frame/stamp')
        if not self.fcu or now-self.fcu[1]>1 or not self.fcu[0].connected or not self.fcu[0].armed:
            raise ValueError('fresh connected armed FCU required')
        if not self.px4 or now-self.px4[1]>.5 or self.px4[0] not in ('AUTO_HOVER','CMD_CTRL'):
            raise ValueError('autonomous control lost; no automatic retry')
        p=msg.pose.pose.position; q=msg.pose.pose.orientation; v=msg.twist.twist.linear
        values=(p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,msg.twist.twist.angular.z)
        if not all(math.isfinite(x) for x in values): raise ValueError('non-finite odometry')
        if not self.min_altitude<=p.z-self.reference<=self.max_altitude: raise ValueError('altitude outside approved operating range')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return (p.x,p.y,p.z),yaw,math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),abs(msg.twist.twist.angular.z)

    def status(self,state,detail,request=None):
        spec=request or self.active or {}
        payload=dict(state=state,detail=detail,task_id=spec.get('task_id'),
                     time_unix_ms=int(time.time()*1000),action=spec.get('action'))
        if self.active:
            payload.update(phase=self.phase,target_world=self.target,travel_m=self.travel,
                           goal_world=getattr(self,'goal_world',None))
            if self.active['action']=='ROTATE' and hasattr(self,'rotation'):
                payload['measured_rotation_deg']=math.degrees(self.rotation.measured)
            if getattr(self,'route',None) is not None:
                payload.update(route_index=self.route_index,route_world=self.route)
        self.status_pub.publish(String(data=json.dumps(payload)))

    def detect(self,selector):
        self.phase='DETECTING'; self.candidate=None; self.ready=False
        self.phase_started=time.monotonic(); self.observation_after=rospy.Time.now()
        self.class_pub.publish(String(data=json.dumps(dict(target_label=self.active['target_label'],selector=selector))))

    def request(self,message):
        spec=None; activated=False
        with self.lock:
            try:
                spec=validate(json.loads(message.data))
                if self.active: raise ValueError('another target skill is active')
                if not self.enabled: raise ValueError('target execution disabled')
                position,yaw,speed,rate=self.flight()
                if speed>.15 or rate>.1: raise ValueError('stabilize in hover before starting target skill')
                if spec['action']!='ROTATE' and not self.client.wait_for_server(rospy.Duration(.25)): raise ValueError('atomic executor unavailable')
                if spec['action']!='ROTATE' and self.class_pub.get_num_connections()<1: raise ValueError('stereo localizer unavailable')
                if spec['action']!='ROTATE' and spec['selector'] not in rospy.get_param('/semantic_raw_stereo_node/target_selector_modes',[]):
                    raise ValueError('stereo localizer must be upgraded for target selection')
                if spec['action'] in ('TURN','ROTATE') and self.rotation_pub.get_num_connections()<1:
                    raise ValueError('rotation hold interface unavailable')
                if spec['action']=='LAND':
                    self.landing_observer.ready()
                    if self.land_pub.get_num_connections()<1: raise ValueError('semantic landing handoff unavailable')
                self.active=spec; self.target=None; self.start=position; self.last_position=position
                activated=True
                self.start_yaw=yaw; self.travel=0.; self.started=time.monotonic(); self.last_status=0
                self.retreat_end=None; self.goal_world=None; self.settled=None
                self.route=None; self.route_index=0; self.move_origin=position
                if spec['action']=='ROTATE':
                    self.rotation=FullRotation(yaw,spec['rotation_direction'],spec['rotation_laps'],spec['rotation_rate_deg_s'])
                    self.rotation_time=time.monotonic(); self.phase='FULL_ROTATION'
                    self.status(self.phase,'counting signed, unwrapped measured yaw'); return
                self.inference_pub.publish(Bool(data=True))
                self.detect(spec['selector']); self.status('DETECTING','waiting for selected stable target')
            except Exception as exc:
                if activated: self.finish('FAILED','request initialization failed: '+str(exc))
                else: self.status('REJECTED',str(exc),spec)

    def on_class(self,msg):
        with self.lock:
            if self.active and self.phase=='DETECTING' and msg.data==self.active['target_label']:
                self.ready=True; self.observation_after=rospy.Time.now()

    def on_target(self,msg,coarse):
        with self.lock:
            if not self.active or self.phase!='DETECTING' or not self.ready: return
            age=(rospy.Time.now()-msg.header.stamp).to_sec()
            if msg.header.frame_id!='world' or msg.header.stamp<self.observation_after or not 0<=age<=.5: return
            target=(msg.point.x,msg.point.y,msg.point.z)
            if not all(math.isfinite(v) for v in target): return
            if self.target is not None and math.dist(target,self.target)>.75: return
            if coarse and self.active['action']!='APPROACH': return
            if self.candidate is None or not coarse: self.candidate=(target,coarse,msg.header.stamp)

    def send_move(self,point,yaw_mode):
        if self.travel+math.dist(self.last_position,point)>self.active['max_travel_m']+.15:
            raise ValueError('task travel budget insufficient for next stage')
        goal=ExecuteAtomicSkillGoal(); goal.skill='GOTO_WORLD'; goal.center_frame='world'
        goal.center.x,goal.center.y,goal.center.z=point; goal.yaw_mode=yaw_mode; goal.timeout=40
        self.move_origin=self.last_position
        self.goal_world=point; self.phase='MOVING'; self.phase_started=time.monotonic()
        self.client.send_goal(goal); self.status('MOVING','bounded collision-checked stage requested')

    def advance_target(self,position,yaw,speed,rate):
        spec=self.active
        if spec['action']=='APPROACH':
            candidate=self.candidate
            if candidate is None or (rospy.Time.now()-candidate[2]).to_sec()>.5: return
            target,coarse,_=candidate
            clearance=spec['stand_off_m']+spec['object_radius_m']
            distance=math.hypot(target[0]-position[0],target[1]-position[1])
            if coarse and distance<=clearance+2: return  # Never finish/enter close range on coarse depth.
            self.target=target
            point=approach_goal(position,target,clearance+(1.0 if coarse else 0.0))
            if point is None:
                if distance<clearance-.25: raise ValueError('inside requested clearance; use explicit retreat')
                if speed<=.15 and rate<=.1: self.finish('SUCCEEDED','precise target stand-off confirmed')
            else: self.send_move(point,'path_tangent')
        elif spec['action']=='RETREAT':
            if self.retreat_end is None:
                self.retreat_end=retreat_goal(position,self.target,spec['distance_m'])
                self.initial_range=math.dist(position[:2],self.target[:2])
            point=bounded_goal(position,self.retreat_end)
            if point is not None: self.send_move(point,'fixed')
            elif speed<=.15 and rate<=.1:
                if math.dist(position[:2],self.target[:2])<self.initial_range+spec['distance_m']-.2:
                    raise ValueError('retreat distance not confirmed')
                self.finish('SUCCEEDED','retreat endpoint and increased target distance confirmed')
        elif spec['action'] in ('PASS','LAND'):
            self.advance_route(position,yaw,speed,rate)
        elif spec['action']=='ROTATE':
            if math.dist(position,self.start)>.25: raise ValueError('position drift during full rotation')
            now=time.monotonic(); target_yaw=self.rotation.step(yaw,now-self.rotation_time); self.rotation_time=now
            if self.rotation.reached() and speed<=.15 and rate<=.1:
                if self.settled is None: self.settled=now
                if now-self.settled>=.5: self.finish('SUCCEEDED','full signed rotation confirmed by odometry'); return
            else: self.settled=None
            msg=PositionCommand(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='world'
            msg.position.x,msg.position.y,msg.position.z=self.start; msg.yaw=target_yaw
            self.rotation_pub.publish(msg)
        else:
            if math.dist(position,self.start)>.25: raise ValueError('position drift during target-facing turn')
            error=yaw_error(bearing(position,self.target),yaw)
            now=time.monotonic()
            if abs(error)<.07 and speed<=.15 and rate<=.1:
                if self.settled is None: self.settled=now
                if now-self.settled>=.5:
                    self.finish('SUCCEEDED','measured heading faces locked target'); return
            else:
                self.settled=None
            if self.phase!='FACING': self.phase='FACING'; self.phase_started=now
            if now-self.phase_started>30: raise ValueError('target-facing turn timed out')
            msg=PositionCommand(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='world'
            msg.position.x,msg.position.y,msg.position.z=self.start
            msg.yaw=yaw+max(-math.pi/4,min(math.pi/4,error))
            self.rotation_pub.publish(msg)

    def advance_route(self,position,yaw,speed,rate):
        spec=self.active; clearance=spec['stand_off_m']+spec['object_radius_m']
        if self.route is None:
            if spec['action']=='PASS':
                self.route=pass_route(self.start,self.target,spec['side'],clearance,spec['exit_distance_m'])
            else:
                accepted=[]
                for point in landing_candidates(self.start,self.target,spec['side'],clearance):
                    try:
                        evidence=self.landing_observer.assess(point,position[2],self.reference)
                        if math.dist(position,point)<=spec['max_travel_m']:
                            accepted.append((math.dist(position,point),point,evidence))
                    except ValueError: continue
                if not accepted: raise ValueError('no observed flat clear landing site on requested side; LAND not sent')
                _,point,self.landing_evidence=min(accepted,key=lambda item:item[0]); self.route=[point]
            total=sum(math.dist(a,b) for a,b in zip([position]+self.route,self.route))
            if total>spec['max_travel_m']: raise ValueError('route exceeds travel budget before departure')
            self.status('ROUTE_SELECTED','fixed world route selected; unknown landing space is rejected')
        endpoint=self.route[self.route_index]
        point=bounded_goal(position,endpoint)
        if point is not None: self.send_move(point,'path_tangent'); return
        if speed>.15 or rate>.1: return
        if self.route_index+1<len(self.route):
            self.route_index+=1; return
        if spec['action']=='PASS':
            self.finish('SUCCEEDED','specified-side pass exit and settled position confirmed'); return
        evidence=self.landing_observer.assess(endpoint,position[2],self.reference)
        if self.settled is None: self.settled=time.monotonic()
        if time.monotonic()-self.settled<.5: return
        self.phase='LAND_HANDOFF'; self.phase_started=time.monotonic()
        self.status('LAND_HANDOFF','site rechecked; handing off once to existing landing controller')
        self.land_pub.publish(String(data=json.dumps(dict(task_id=spec['task_id'],hover_world=list(position),
            evidence=evidence,sent_at_unix_ms=int(time.time()*1000)))))

    def check_route(self,position):
        if self.active['action']=='PASS':
            clearance=self.active['stand_off_m']+self.active['object_radius_m']
            if math.dist(position[:2],self.target[:2])<clearance-.15: raise ValueError('pass clearance violated')
            if self.phase=='MOVING' and segment_distance(position,self.move_origin,self.goal_world)>.35:
                raise ValueError('planner departed specified pass corridor; refusing opposite-side detour')
        elif self.active['action']=='LAND' and self.route is not None:
            self.landing_observer.assess(self.route[-1],position[2],self.reference)

    def on_bridge_status(self,message):
        with self.lock:
            try: status=json.loads(message.data)
            except (ValueError,TypeError): return
            if not self.active or self.phase!='LAND_HANDOFF' or status.get('task_id')!=self.active['task_id']: return
            state=status.get('status')
            if state in ('land_complete','land_failed','semantic_land_rejected'):
                self.status('SUCCEEDED' if state=='land_complete' else 'FAILED',status.get('detail',state))
                self.active=None; self.phase=None

    def tick(self,_event):
        with self.lock:
            if not self.active: return
            if self.phase=='STOPPING':
                self.confirm_stop(); return
            if self.phase=='LAND_HANDOFF':
                # The existing Bridge owns descent. Never publish hover/position goals here.
                if (self.fcu and time.monotonic()-self.fcu[1]<1 and self.fcu[0].connected
                        and not self.fcu[0].armed and time.monotonic()-self.phase_started>1):
                    self.status('SUCCEEDED','FCU disarmed after landing handoff')
                    self.active=None; self.phase=None; return
                if time.monotonic()-self.last_status>=1:
                    self.last_status=time.monotonic()
                    self.status('LAND_HANDOFF','waiting for landing controller result; RC required to interrupt descent')
                return
            try:
                position,yaw,speed,rate=self.flight(); now=time.monotonic()
                delta=math.dist(position,self.last_position)
                if delta>1: raise ValueError('localization jump')
                # Ignore sub-centimetre odometry jitter; retain accumulated displacement.
                if delta>=.01: self.travel+=delta; self.last_position=position
                if self.travel>self.active['max_travel_m']+.2 or now-self.started>180:
                    raise ValueError('travel/time budget exceeded')
                if now-self.last_status>=1:
                    self.last_status=now; self.status(self.phase,'target skill active')
                if self.active['action'] in ('PASS','LAND') and self.target is not None:
                    self.check_route(position)
                if self.phase=='MOVING':
                    if now-self.phase_started>42: raise ValueError('atomic stage timed out')
                    if self.client.simple_state!=2: return
                    result=self.client.get_result()
                    if self.client.get_state()!=GoalStatus.SUCCEEDED or not result or not result.success:
                        raise ValueError('atomic stage failed; critical target stages cannot be skipped')
                    if self.active['action']=='APPROACH': self.detect('highest_confidence'); return
                    self.phase='VERIFYING'
                if self.phase=='DETECTING':
                    if now-self.phase_started>20: raise ValueError('selected/locked target detection timed out')
                    if self.candidate is None or (rospy.Time.now()-self.candidate[2]).to_sec()>.5: return
                    if self.active['action']!='APPROACH': self.target=self.candidate[0]
                self.advance_target(position,yaw,speed,rate)
            except Exception as exc: self.finish('FAILED',str(exc))

    def finish(self,state,detail):
        if self.phase=='STOPPING': return
        self.stop_wait_action=self.phase=='MOVING'
        self.client.cancel_goal(); self.hover_pub.publish(Empty())
        if state!='SUCCEEDED':
            self.stop_result=(state,detail); self.phase='STOPPING'; self.settled=None
            self.last_status=0
            self.status('STOPPING',detail+'; waiting for action cancellation and settled vehicle')
            return
        self.status(state,detail); self.active=None; self.phase=None; self.candidate=None

    def confirm_stop(self):
        now=time.monotonic()
        disarmed=self.fcu and now-self.fcu[1]<1 and self.fcu[0].connected and not self.fcu[0].armed
        stopped=False
        try:
            _,_,speed,rate=self.flight()
            stopped=speed<=.15 and rate<=.1
        except ValueError: pass
        action_done=not self.stop_wait_action or self.client.simple_state==2
        if disarmed or (action_done and stopped):
            if self.settled is None: self.settled=now
            if now-self.settled>=.5:
                self.status(*self.stop_result); self.active=None; self.phase=None; self.candidate=None
                return
        else: self.settled=None
        if now-self.last_status>=1:
            self.last_status=now
            self.status('STOPPING','stop not yet confirmed; inspect vehicle/RC, new target requests remain blocked')

    def cancel(self,_message):
        with self.lock:
            if self.active and self.phase=='LAND_HANDOFF':
                self.status('LAND_HANDOFF','cancel received after landing handoff; cannot claim descent stopped, use RC')
                return
            if self.active: self.finish('CANCELLED','operator cancelled target skill')

if __name__=='__main__':
    rospy.init_node('target_interaction'); TargetInteraction(); rospy.spin()
