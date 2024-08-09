import rospy

import numpy as np
from itertools import product
from typing import Sequence, Union, SupportsIndex

from actionlib import SimpleActionClient
from ur5_kinematics.msg import URGoToAction, URGoToGoal, URGoToResult
from ur5_kinematics.srv import CheckCollision, CheckCollisionRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from placo_utils.tf import tf as ptf

import matplotlib.pyplot as plt

import sys, json, math

class WorkspaceSampler():
    def __init__(self, frame, collision_service: str, kinematics_topic: str, n_points: Union[SupportsIndex, Sequence[SupportsIndex]], xrange: list, yrange: list, zrange: list, angle_cone: float = np.pi / 4, prefix: str = ''):
        self.prefix = prefix
        self.seq = 0.0
        if len(n_points) == 1:
            n_points = n_points[0], n_points[0], n_points[0]
            
        assert len(n_points) == 3
        
        self.kinematics_client = SimpleActionClient(kinematics_topic, URGoToAction)
        self.collisions_checker = rospy.ServiceProxy(collision_service, CheckCollision)
        self.frame = frame
        self.angle = angle_cone
        
        self.kinematics_client.wait_for_server()
        self.collisions_checker.wait_for_service()
        rospy.loginfo(f'Connected to kinematics action server')
        
        self.xs = np.linspace(xrange[0], xrange[1], num=n_points[0])
        self.ys = np.linspace(yrange[0], yrange[1], num=n_points[1])
        self.zs = np.linspace(zrange[0], zrange[1], num=n_points[2])
        self.sample = list(product(self.xs, self.ys, self.zs))
        
        self.joint_states_pub = rospy.Publisher("/joint_states", JointState, queue_size=10, latch=True)
        self.joint_names = [
            f'{prefix}shoulder_pan_joint',
            f'{prefix}shoulder_lift_joint',
            f'{prefix}elbow_joint',
            f'{prefix}wrist_1_joint',
            f'{prefix}wrist_2_joint',
            f'{prefix}wrist_3_joint',
            f'{prefix}hande_left_finger_joint'
        ]
        
        state = JointState()
        state.header = Header()
        state.header.stamp = rospy.Time.now()
        state.name = self.joint_names
        state.position = [3.14, -1.57, 1.57, -1.57, -1.57, 1.57, 0.0]
        self.joint_states_pub.publish(state)
        
        self.targets = [ptf.translation_matrix(point) @ ptf.euler_matrix(np.pi, 0, 0) for point in self.sample]
        self.target_poses = [WorkspaceSampler.matrix_to_pose(target, self.frame) for target in self.targets]


    def sample_reachable_points(self) -> list:
        self.seq = 0
        reachable = []
        rejected = []
        
        for i, target in enumerate(self.target_poses):
            goal = URGoToGoal()
            goal.target_pose = target
            goal.timeout = 1.0
            goal.duration = rospy.Duration(secs=1.0)
            goal.target_pose.header.seq = self.seq
            
            self.kinematics_client.send_goal_and_wait(goal)
            result: URGoToResult = self.kinematics_client.get_result()
            
            if result.state == URGoToResult.SUCCEEDED:
                state = JointState()
                state.header = Header()
                state.header.stamp = rospy.Time.now()
                state.name = self.joint_names
                state.position = list(result.trajectory.points[-1].positions)
                state.position.append(0.0)
                
                req = CheckCollisionRequest()
                req.joints = state
                ret = self.collisions_checker.call(req)
                
                if not ret.isColliding:
                    reachable.append(self.targets[i])
                    self.joint_states_pub.publish(state)
                else:
                    rejected.append(self.targets[i])
            else:
                rejected.append(self.targets[i])
                
            self.seq += 1
            
        return reachable, rejected
    

    def sample_manipulable_points(self, points) -> list:
        manipulable = []
        rejected = []
        space = np.linspace(0.0, np.pi * 2, num=10)
        for point in points:
            n_reached = 0
            for t in space:
                x_theta = np.pi + self.angle * math.cos(t)
                y_theta = np.pi + self.angle * math.sin(t)
                rot = ptf.euler_matrix(x_theta, y_theta, 0.0)
                
                target = WorkspaceSampler.matrix_to_pose(point @ rot)
                goal = URGoToGoal()
                goal.target_pose = target
                goal.timeout = 1.0
                goal.duration = rospy.Duration(secs=1.0)
                goal.target_pose.header.seq = self.seq
                
                self.kinematics_client.send_goal_and_wait(goal)
                result: URGoToResult = self.kinematics_client.get_result()
                if result.state == URGoToResult.SUCCEEDED:
                    self.seq += 1
                    state = JointState()
                    state.header = Header()
                    state.header.stamp = rospy.Time.now()
                    state.name = self.joint_names
                    state.position = list(result.trajectory.points[-1].positions)
                    state.position.append(0.0)
                    
                    n_reached += 1
        
                    req = CheckCollisionRequest()
                    req.joints = state
                    ret = self.collisions_checker.call(req)
                    
                    if not ret.isColliding:
                        n_reached += 1
                        self.joint_states_pub.publish(state)
                        
            # If we reach an arbitrary amount of angles (here 80%)
            # at the target pose, keep track
            if n_reached / len(space) >= 0.8:
                manipulable.append(point)
            else:
                rejected.append(point)
            self.seq += 1

        return manipulable, rejected


    @staticmethod
    def matrix_to_pose(matrix: np.ndarray, frame: str = 'base_link') -> PoseStamped:
        #Transform point into arm base frame
        pos = ptf.translation_from_matrix(matrix)
        rot = ptf.quaternion_from_matrix(matrix)
        
        header = Header()
        header.frame_id = frame
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = rot[0]
        pose.pose.orientation.x = rot[1]
        pose.pose.orientation.y = rot[2]
        pose.pose.orientation.z = rot[3]
        
        return pose
    


if __name__=="__main__":
    import argparse
    
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-t', '--topic', type=str, default='/kinematics_server/goal_pose', help='Kinematics server action topic')
    parser.add_argument('-c', '--collision-service', type=str, default='/kinematics_server/check_collision', help='Kinematics server collision checking service')
    parser.add_argument('-f', '--frameid', type=str, default='base_footprint', help='Frame ID to work in')
    parser.add_argument('-o', '--output', type=str, help='Output file for sampled points')
    parser.add_argument('-n', '--npoints', type=int, default=200, help='Number of points to generate')
    parser.add_argument('-x', '--xrange', type=float, nargs=2, default=[0.0, 1.0], help='Range of values along the X axis for sampled points')
    parser.add_argument('-y', '--yrange', type=float, nargs=2, default=[0.0, 1.0], help='Range of values along the Y axis for sampled points')
    parser.add_argument('-z', '--zrange', type=float, nargs=2, default=[0.0, 1.0], help='Range of values along the Z axis for sampled points')
    
    parser.add_argument('-p', '--prefix', type=str, default='')
    args = parser.parse_args()
    
    print(args)
    
    rospy.init_node(name="workspace_estimator", argv=sys.argv, log_level=rospy.INFO)
    sampler = WorkspaceSampler(args.frameid, args.collision_service, args.topic, args.npoints, args.xrange, args.yrange, args.zrange, prefix=args.prefix)
    reached, rejected = sampler.sample_reachable_points()
    
    reached = np.array(reached)
    xs, ys, zs = reached[:, 0, 3], reached[:, 1, 3], reached[:, 2, 3]
    
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # xs, ys, zs = points[:][0, 3], points[:][1, 3], points[:][2, 3]
    # print(xs, ys, zs)
    ax.scatter(xs, ys, zs)
    