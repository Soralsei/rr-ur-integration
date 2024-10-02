#! /usr/bin/env python

import rospy
from rr100_ur_vision.srv import GetMarkerTransform, GetMarkerTransformRequest
from rr100_ur_worker.msg import Task
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_geometry_msgs as tf
import argparse

if __name__ == '__main__':
    rospy.init_node("aruco_localizer_test")
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-id', default=43, type=int, help="The robot's aruco marker id")
    parser.add_argument('--target-id', default=45, type=int, help="The robot's aruco marker id")
    args = parser.parse_args()
    
    task_pub = rospy.Publisher("/tasks", Task, queue_size=10)
    aruco_client = rospy.ServiceProxy("/aruco_localizer/get_marker_transform", GetMarkerTransform)
    aruco_client.wait_for_service(rospy.Duration(2.0))
    
    freq = rospy.Rate(rospy.Duration(1.0))
    
    done = False
    
    req = GetMarkerTransformRequest()
    req.id = args.target_id
    
    while not done:
        try :
            res = aruco_client.call(req)
        except:
            
            freq.sleep()
            continue
        answer = ""
        while answer != 'y' or answer != 'n':
            answer = input(f'Got transform {res.transform}, proceed [y/n] : ').lower()
            
        if answer == 'n':
            freq.sleep()
            continue
        
        task = Task()
        task.type = Task.TYPE_REACHING
        
        pose = PoseStamped()
        pose.header.frame_id = "base_footprint"
        pose.header.stamp = rospy.Time.now()
        
        pose.pose.position = res.transform.translation
        pose.pose.orientation = res.transform.orientation
        
        T = TransformStamped()
        T.transform.rotation.x = 1.0
        T.transform.rotation.y = 0.0
        T.transform.rotation.z = 0.0
        T.transform.rotation.w = 0.0
        pose = tf.do_transform_pose(pose, transform=T)
        
        task.target = pose
        
        rospy.loginfo(f'Publishing Task...')
        task_pub.publish(task)
        
        done = True
    
    rospy.loginfo(f'')
    aruco_client.close()
    