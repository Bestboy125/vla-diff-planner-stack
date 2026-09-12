"""Optional calibrated LiDAR ray observer; no inferred frame aliases or fake free space."""
import threading
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_matrix
from landing_safety import LandingMap

class LandingObserver:
    def __init__(self):
        self.map=LandingMap(); self.processing=threading.Lock(); self.reason='no calibrated cloud received'
        self.sensor_frame=rospy.get_param('~landing_sensor_frame','')
        self.verified=bool(rospy.get_param('~landing_geometry_verified',False))
        self.radius=float(rospy.get_param('~landing_footprint_radius_m',.8))
        self.buffer=tf2_ros.Buffer(); self.listener=tf2_ros.TransformListener(self.buffer)
        self.sub=rospy.Subscriber(rospy.get_param('~landing_cloud_topic','/laserMapping/cloud_registered'),
                                  PointCloud2,self.on_cloud,queue_size=1)
    def ready(self):
        if not self.verified or not self.sensor_frame or self.radius<.8:
            raise ValueError('landing calibration not verified: sensor frame, 1m vehicle + clearance, and ground reference required')
    def transform(self,frame,stamp):
        if frame=='world': return np.eye(4)
        t=self.buffer.lookup_transform('world',frame,stamp,rospy.Duration(.05)).transform
        matrix=quaternion_matrix((t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
        matrix[:3,3]=(t.translation.x,t.translation.y,t.translation.z)
        return matrix
    def on_cloud(self,msg):
        if not self.verified or not self.sensor_frame or not self.processing.acquire(False): return
        try:
            self.ready(); now=rospy.Time.now().to_sec(); stamp=msg.header.stamp.to_sec()
            if not 0<=now-stamp<=.5: raise ValueError('stale cloud')
            if self.map.last_stamp and stamp-self.map.last_stamp<.4: return
            transform=self.transform(msg.header.frame_id,msg.header.stamp)
            origin=self.transform(self.sensor_frame,msg.header.stamp)[:3,3]
            # Honor a hard size bound before allocating; oversized observations are rejected.
            if msg.width*msg.height>200000: raise ValueError('landing cloud exceeds processing bound')
            points=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)))
            if not len(points): raise ValueError('empty cloud')
            points=points.dot(transform[:3,:3].T)+transform[:3,3]
            self.map.ingest(origin,points,stamp,now)
            self.reason='calibrated observed voxels available'
        except Exception as exc:
            self.reason=str(exc)
            rospy.logwarn_throttle(5,'landing observation rejected: %s',self.reason)
        finally: self.processing.release()
    def assess(self,center,hover_z,reference):
        self.ready()
        return self.map.assess(center,hover_z,self.radius,reference,rospy.Time.now().to_sec())
