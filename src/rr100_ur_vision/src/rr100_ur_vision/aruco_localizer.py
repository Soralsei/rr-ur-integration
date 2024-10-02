from rr100_ur_vision.srv import GetMarkerTransform, GetMarkerTransformRequest, GetMarkerTransformResponse

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
        self.robot_marker_frame = rospy.get_param("~robot_marker_frame", "aruco_marker_link")
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_footprint")
        markers_topic = rospy.get_param("~markers_topic", "/aruco_marker_publisher/markers")
        
        self.markers_sub = rospy.Subscriber(markers_topic, MarkerArray, self._markers_callback, queue_size=10)
        
        self.robot_marker : Marker = None
        self.marker_targets : dict = {}
        self.marker_transforms : dict = {}
        self.marker_lock = Lock()
        
        self.tf = Buffer(rospy.Duration(100.0))
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
                    
    def _do_work(self, _) -> None:
        with self.marker_lock:
            if self.robot_marker is None:
                rospy.loginfo(f'Robot marker not yet found')
                return
            
            if self.is_older_than(self.robot_marker, 5.0):
                rospy.logwarn("Robot marker has not been visible for at least 5 seconds")
                self.robot_marker = None
                return
            
            # Delete all markers older than 5.0 seconds
            to_delete = []
            for marker in self.marker_targets.values():
                if self.is_older_than(marker, 2.0):
                    to_delete.append(marker.id)
                    
            for id in to_delete:
                del self.marker_targets[id]
                    
            if len(self.marker_targets) == 0:
                rospy.loginfo(f'No target markers detected')
                return
            
            base_frame_to_marker : TransformStamped = None
            try:
                base_frame_to_marker = self.tf.lookup_transform(self.robot_base_frame, self.robot_marker_frame, time=rospy.Time.from_sec(0))
            except Exception as e:
                rospy.logerr(f'Failed to lookup transform : {e.with_traceback(None)}')
                return
            
            # print(base_frame_to_marker)
            
            self.marker_transforms.clear()
            base_frame_to_marker = numpify(base_frame_to_marker.transform)
            
            for marker in self.marker_targets.values():
                cam_to_target = numpify(marker.pose.pose)
                robot_marker_to_cam = np.linalg.inv(numpify(self.robot_marker.pose.pose))
                base_frame_to_cam = base_frame_to_marker @ robot_marker_to_cam
                base_frame_to_target = (base_frame_to_cam @ cam_to_target)

                base_to_target_transform = msgify(Transform, base_frame_to_target)
                self.marker_transforms[marker.id] = base_to_target_transform


    def is_older_than(self, marker: Marker, duration: float) -> bool:
        return rospy.Time.now() - marker.header.stamp > rospy.Duration(duration)
    
    def get_marker_transform(self, req: GetMarkerTransformRequest) -> GetMarkerTransformResponse:
        transform = None
        # Try to get 
        with self.marker_lock:
            transform = self.marker_transforms.get(req.id)
            
        if transform is None:
            return None
        
        res = GetMarkerTransformResponse()
        res.transform = transform
        
        return res