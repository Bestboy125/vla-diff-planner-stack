#!/usr/bin/env python3
"""Run ONLY against a private ROS master, with every control gate closed.
No MAVROS, planner, camera or PX4Ctrl is launched. Exits and reaps all children.
The temporary bridge listens on loopback:19191 for host integration tests.
"""
import json
import os
import subprocess
import time
import xmlrpc.client

env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:11411',ROS_HOSTNAME='127.0.0.1')
env.pop('ROS_IP',None)
os.environ.update(env)
processes=[]

def spawn(args):
    child=subprocess.Popen(args,env=env)
    processes.append(child)
    return child

try:
    try:
        xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/target_smoke')
    except OSError:
        pass
    else:
        raise RuntimeError('private test ROS master port is already occupied')
    spawn(['roscore','-p','11411'])
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from quadrotor_msgs.msg import PositionCommand,TakeoffLand
    from atomic_skill_executor.msg import ExecuteAtomicSkillActionGoal
    for _ in range(50):
        try:
            xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/target_smoke'); break
        except OSError: time.sleep(.2)
    else: raise RuntimeError('test master unavailable')
    rospy.init_node('isolated_target_smoke',disable_signals=True)
    motion=[]; statuses=[]
    subscribers=[]
    for topic,kind in [('/goal',PoseStamped),('/position_cmd',PositionCommand),
                       ('/drone_0_traj_server/operator_rotation',PositionCommand),
                       ('/px4ctrl/takeoff_land',TakeoffLand),
                       ('/atomic_skill_executor/execute/goal',ExecuteAtomicSkillActionGoal)]:
        subscribers.append(rospy.Subscriber(topic,kind,lambda msg:motion.append(msg)))
    subscribers.append(rospy.Subscriber('/target_interaction/status',String,lambda msg:statuses.append(json.loads(msg.data))))
    request=rospy.Publisher('/target_interaction/request',String,queue_size=1)
    spawn(['rosrun','target_interaction','target_interaction_node.py','__name:=target_interaction','_execution_enabled:=false'])
    spawn(['rosrun','vla_diff_bridge','vla_diff_bridge_node.py','__name:=isolated_bridge',
           '_bind_host:=127.0.0.1','_port:=19191','_auth_token:=isolated-test-token',
           '_live_publish_enabled:=false','_preview_only_mode:=true','_operator_task_enabled:=false'])
    for _ in range(50):
        if request.get_num_connections(): break
        time.sleep(.2)
    else: raise RuntimeError('disabled target executor unavailable')
    for action in ('APPROACH','RETREAT','TURN','PASS','ROTATE','LAND'):
        request.publish(String(data=json.dumps(dict(task_id='smoke-'+action,action=action,
            target_label='chair',selector='left',stand_off_m=1.5,object_radius_m=.5,distance_m=1,max_travel_m=10,
            side='right',exit_distance_m=1,rotation_direction='clockwise',rotation_laps=1,rotation_rate_deg_s=30))))
        for _ in range(50):
            if any(s.get('task_id')=='smoke-'+action for s in statuses): break
            time.sleep(.1)
        else: raise RuntimeError('missing rejection status')
        status=next(s for s in statuses if s.get('task_id')=='smoke-'+action)
        assert status['state']=='REJECTED' and 'disabled' in status['detail'],status
    print('ISOLATED_SMOKE_READY: six disabled-skill rejections confirmed; loopback bridge available for 120s',flush=True)
    for _ in range(120):
        assert not motion,'Unexpected motion command in disabled test'
        assert all(p.poll() is None for p in processes),'Test child exited unexpectedly'
        time.sleep(1)
    print('ISOLATED_SMOKE_PASS: zero flight/control commands',flush=True)
finally:
    for child in reversed(processes):
        if child.poll() is None: child.terminate()
    for child in reversed(processes):
        try: child.wait(timeout=10)
        except subprocess.TimeoutExpired: child.kill(); child.wait()
