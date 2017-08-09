#!/usr/bin/env python

# import modules
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
import numpy as np

def message_from_transform(T):
    msg = Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    p = tf.transformations.translation_from_matrix(T)
    msg.translation.x = p[0]
    msg.translation.y = p[1]
    msg.translation.z = p[2]
    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

def homo_transformation(alpha, a, theta, d):

    rot_x   = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(alpha, 0.0, 0.0))
    trans_x = tf.transformations.translation_matrix([a, 0.0, 0.0])
    rot_z   = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, theta))
    trans_z = tf.transformations.translation_matrix([0.0, 0.0, d])
    
    return tf.transformations.concatenate_matrices(rot_x, trans_x, rot_z, trans_z)

class fk_checker(object):
    
    def __init__(self):
        # joint state sub.
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        # trans. publisher
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def FK_calculator(self,joint_values):
        
        s = {'alpha0':        0, 'a0':       0 , 'd1':   0.095, 'q1':           joint_values[0],
             'alpha1': -np.pi/2, 'a1':  -0.0105, 'd2':       0, 'q2': joint_values[1] - np.pi/2,
             'alpha2':        0, 'a2':    0.104, 'd3':       0, 'q3': joint_values[2] + np.pi/2,
             'alpha3':        0, 'a3':   0.0955, 'd4':       0, 'q4': joint_values[3] - np.pi/2,
             'alpha4': -np.pi/2, 'a4':   0.0275, 'd5':       0, 'q5':           joint_values[4],
             'alpha5':        0, 'a5':        0, 'd6':  0.0375, 'q6':                         0}
        
        # Define Modified DH Transformation matrix
        T01 = homo_transformation(s['alpha0'], s['a0'], s['q1'], s['d1'])
        T12 = homo_transformation(s['alpha1'], s['a1'], s['q2'], s['d2'])
        T23 = homo_transformation(s['alpha2'], s['a2'], s['q3'], s['d3'])
        T34 = homo_transformation(s['alpha3'], s['a3'], s['q4'], s['d4'])
        T45 = homo_transformation(s['alpha4'], s['a4'], s['q5'], s['d5'])
        T5G = homo_transformation(s['alpha5'], s['a5'], s['q6'], s['d6'])

        # Create individual transformation matrices
        T02 = np.dot(T01, T12)
        T03 = np.dot(T02, T23)
        T04 = np.dot(T03, T34)
        T05 = np.dot(T04, T45)
        T0G = np.dot(T05, T5G)

        # correcting transformations
        TC1 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, np.pi))
        TC2 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, -np.pi/2, 0.0))
        TC = np.dot(TC1,TC2)
        T0G = np.dot(T0G,TC)
        
        return T0G


    # joint state callback
    def joint_states_callback(self,joint_state):

        transform4_base = TransformStamped()
        transform4_base.header.stamp = rospy.Time.now()
        transform4_base.header.frame_id = "base_link"
        transform4_base.child_frame_id = "ee"
        
        transform4_base.transform = message_from_transform(self.FK_calculator(joint_state.position))
        self.tf_broadcaster.sendTransform(transform4_base)

        # ## DH parameters
        # s = {'alpha0':        0, 'a0':     0 , 'd1':    95, 'q1':           joint_values[0],
        #      'alpha1': -np.pi/2, 'a1':  -10.5, 'd2':     0, 'q2': joint_values[1] - np.pi/2,
        #      'alpha2':        0, 'a2':    104, 'd3':     0, 'q3': joint_values[2] + np.pi/2,
        #      'alpha3':        0, 'a3':   95.5, 'd4':     0, 'q4': joint_values[3] - np.pi/2,
        #      'alpha4': -np.pi/2, 'a4':   27.5, 'd5':     0, 'q5':           joint_values[4],
        #      'alpha5':        0, 'a5':      0, 'd6':  37.5, 'q6':                         0}

        # # transformations
        # _T01 = homo_transformation(s['alpha0'], s['a0'], s['q1'], s['d1'])
        # _T12 = homo_transformation(s['alpha1'], s['a1'], s['q2'], s['d2'])
        # _T23 = homo_transformation(s['alpha2'], s['a2'], s['q3'], s['d3'])
        # _T34 = homo_transformation(s['alpha3'], s['a3'], s['q4'], s['d4'])
        # _T45 = homo_transformation(s['alpha4'], s['a4'], s['q5'], s['d5'])
        # _T56 = homo_transformation(s['alpha5'], s['a5'], s['q6'], s['d6'])
        # _T6G = homo_transformation(s['alpha6'], s['a6'], s['q7'], s['d7'])

        # _T02 = np.dot(_T01, _T12)
        # _T03 = np.dot(_T02, _T23)
        # _T04 = np.dot(_T03, _T34)
        # _T05 = np.dot(_T04, _T45)
        # _T06 = np.dot(_T05, _T56)
        # _T0G = np.dot(_T06, _T6G)

        # # correcting transformations
        # TC1 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, 0.0, np.pi))
        # TC2 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.0, -np.pi/2, 0.0))
        # TC = np.dot(TC1,TC2)
        # ####### input of pick and place project ###############
        # T0G = np.dot(_T0G,TC)
        # ##################################################

        # T0G_dh = np.dot(T0G,np.linalg.inv(TC))
        # P = T0G_dh[:3,3]
        # R = T0G_dh[:3, :3]

        # ############# theta 1 #############
        # P05 = P - s['d7']*R[:,2]
        # theta1 = np.arctan2(P05[1],P05[0])

        
        # ############ theta 2 ##########
        # T01 = homo_transformation(s['alpha0'], s['a0'], theta1, s['d1'])
        # T12 = homo_transformation(s['alpha1'], s['a1'], - np.pi/2, s['d2'])
        
        # T02 = np.dot(T01, T12)
        # P02 = T02[:3, 3]
        # R02 = T02[:3,:3]
        
        # T20 = np.linalg.inv(T02)
        # R20 = np.linalg.inv(R02)
        
        # # base_link
        # P25 = P05 - P02
        # P25_ = np.dot(R20,P25)

        # beta1 = np.arctan2(P25_[0],P25_[1])

        # _d = np.sqrt(0.054**2 + (0.96 + 0.54)**2)
        # l = np.linalg.norm(P25_)
        # beta2 = np.arccos((l**2 + s['a2']**2 - _d**2)/(2*s['a2']*l))

        # theta2 = np.pi/2 - (beta1 + beta2)
        # ##### theta 3 ######

        # phi = np.arccos((s['a2']**2 + _d**2 - l**2)/(2*s['a2']*_d))
        # alpha = np.arctan2(0.054,1.5)
        # theta3 = np.pi/2 - (phi + alpha)

        # ####### theta 4 #########
        # T01 = homo_transformation(s['alpha0'], s['a0'], theta1, s['d1'])
        # T12 = homo_transformation(s['alpha1'], s['a1'], theta2 - np.pi/2, s['d2'])
        # T23 = homo_transformation(s['alpha2'], s['a2'], theta3, s['d3'])
        # T02 = np.dot(T01, T12)
        # T03 = np.dot(T02, T23)
        # R03 = T03[:3, :3]
        # R36 = np.dot(np.linalg.inv(R03),R)

        # theta4 = np.arctan2(R36[2,2],-R36[0,2]) 
        # ######## theta 5 ######
        # theta5 = np.arctan2(np.sqrt(R36[1,0]**2 + R36[1,1]**2),R36[1,2])
        # ######## theta 6 ########3
        # theta6 = np.arctan2(-R36[1,1],R36[1,0])
        
        # if np.sin(theta5) < 0 :
        #     theta4 = np.arctan2(-R36[2,2],R36[0,2]) 
        #     theta6 = np.arctan2(R36[1,1],-R36[1,0])

        # if np.allclose(theta5, 0):
        #     theta4 = 0
        #     theta6 = np.arctan2(-R36[0,1],-R36[2,1])

        # roll, pitch, yaw = tf.transformations.euler_from_matrix(R36,'ryzy')
        # pitch += np.pi/2
        # print([roll, pitch, yaw])
        # theta = [theta1, theta2, theta3, theta4, theta5, theta6, 0]
        # #theta = [theta1, theta2, theta3, roll, pitch, yaw, 0]
        # print(np.round(theta,2))

        # T01 = homo_transformation(s['alpha0'], s['a0'], theta1, s['d1'])
        # T12 = homo_transformation(s['alpha1'], s['a1'], - np.pi/2, s['d2'])
        
        # T02 = np.dot(T01, T12)
        #roll, pitch, yaw = tf.transformations.euler_from_matrix(R36,'ryxz')
        # pitch += np.pi/2
        # print([roll, pitch, yaw])
        
        
        # transform1.transform = message_from_transform(tf.transformations.translation_matrix(P05))
        # self.tf_broadcaster.sendTransform(transform1)
        # ############################

        # transform.transform = message_from_transform(tf.transformations.translation_matrix(P25_))
        # self.tf_broadcaster.sendTransform(transform)

if __name__ == "__main__":
    rospy.init_node('FK_checker')
    fk_checker = fk_checker()
    rospy.spin()
