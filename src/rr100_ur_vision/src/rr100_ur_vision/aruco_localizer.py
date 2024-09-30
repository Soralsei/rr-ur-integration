from rr100_ur_vision.srv import GetMarkerTransform, GetMarkerTransformRequest

from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import TransformStamped, Transform
import rospy
from threading import Lock
from ros_numpy import numpify, msgify
from tf2_ros import Buffer, TransformListener
import numpy as np

class ArucoLocalizer:
    
    def __init__(self) -> None:
        self.robot_marker_id = rospy.get_param("~robot_marker_id", 43)
        self.robot_marker_frame = rospy.get_param("~robot_marker_frame", "aruco")
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_footprint")
        markers_topic = rospy.get_param("~markers_topic", "/aruco_marker_publisher/markers")
        
        self.markers_sub = rospy.Subscriber(markers_topic, MarkerArray, self._markers_callback, queue_size=10)
        
        self.robot_marker : Marker = None
        self.marker_targets : dict = {}
        self.marker_transforms : dict = {}
        self.marker_lock = Lock()
        
        self.tf = Buffer(100.0)
        self.tf_listener = TransformListener(self.tf, queue_size=10)
        self.transform_timer = rospy.Timer(rospy.Duration(0.5), self._do_work)
        self.marker_transform_service = rospy.Service("/aruco_localizer/get_marker_transform", GetMarkerTransform, self.get_marker_transform)
    
    def _markers_callback(self, markers: MarkerArray) -> None:
        with self.marker_lock:
            for marker in markers.markers:
                marker : Marker = marker
                if marker.id == self.robot_marker_id:
                    self.robot_marker = marker
                # If no marker we know of has the same id, we store it
                else:
                    self.marker_targets[marker.id] = marker
                    
    def _do_work(self) -> None:
        with self.marker_lock:
            if self.robot_marker is None:
                rospy.loginfo(f'Robot marker not yet found')
                return
            
            if self.is_older_than(self.robot_marker, 5.0):
                rospy.logwarn("Robot marker has not been visible for at least 5 seconds")
                self.robot_marker = None
                return
            
            # Delete all markers older than 5.0 seconds
            self.marker_targets[:] = [marker for marker in self.marker_targets if not self.is_older_than(marker, 5.0)]
            if len(self.marker_targets) == 0:
                rospy.loginfo(f'No target markers detected')
                return
            
            base_frame_to_marker : TransformStamped = None
            try:
                base_frame_to_marker = self.tf.lookup_transform(self.robot_marker_frame, self.robot_base_frame, time=rospy.Time.from_sec(0))
            except Exception as e:
                rospy.logerr(f'Failed to lookup transform : {e.with_traceback(None)}')
                return
            
            self.marker_transforms.clear()
            base_frame_to_marker = numpify(base_frame_to_marker.transform)
            
            for marker in self.marker_targets.values():
                cam_to_target = numpify(marker.pose)
                robot_marker_to_cam = np.linalg.inv(numpify(self.robot_marker.pose))
                base_frame_to_cam = base_frame_to_marker @ robot_marker_to_cam
                base_frame_to_target = base_frame_to_cam @ cam_to_target

                base_to_target_transform = msgify(Transform, base_frame_to_target)
                self.marker_transforms[marker.id] = base_to_target_transform     

    def is_older_than(self, marker: Marker, duration: float) -> bool:
        return rospy.Time.now() - marker.header.stamp > rospy.Duration(duration)
    
    def get_marker_transform(self, req: GetMarkerTransformRequest) -> Transform:
        transform = None
        # Try to get 
        with self.marker_lock:
            transform = self.marker_transforms.get(id)
        return transform