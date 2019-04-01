#!/usr/bin/env python  
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
    # Base -> Object
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    
    q1 = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)
    T_Bas_Obj = numpy.dot(tf.transformations.quaternion_matrix(q1), tf.transformations.translation_matrix((0.0, 1.0, 1.0)))
    
    tr1 = tf.transformations.translation_from_matrix(T_Bas_Obj)
    object_transform.transform.translation.x = tr1[0]
    object_transform.transform.translation.y = tr1[1]
    object_transform.transform.translation.z = tr1[2]
    
    q2 = tf.transformations.quaternion_from_matrix(T_Bas_Obj)
    object_transform.transform.rotation.x = q2[0]
    object_transform.transform.rotation.y = q2[1]
    object_transform.transform.rotation.z = q2[2]
    object_transform.transform.rotation.w = q2[3]
    br.sendTransform(object_transform)


    # Base -> Robot
    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"

    q3 = tf.transformations.quaternion_about_axis(1.5, (0,0,1))
    T_Bas_Rob = numpy.dot(tf.transformations.quaternion_matrix(q3), tf.transformations.translation_matrix((0.0, -1.0, 0.0)))

    tr2 = tf.transformations.translation_from_matrix(T_Bas_Rob)
    robot_transform.transform.translation.x = tr2[0]
    robot_transform.transform.translation.y = tr2[1]
    robot_transform.transform.translation.z = tr2[2]

    q4 = tf.transformations.quaternion_from_matrix(T_Bas_Rob)
    robot_transform.transform.rotation.x = q4[0]
    robot_transform.transform.rotation.y = q4[1]
    robot_transform.transform.rotation.z = q4[2]
    robot_transform.transform.rotation.w = q4[3]

    br.sendTransform(robot_transform)
 
 
    # Robot -> Camera
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"

    # Get vector to move camera
    tr_Rob_Cam_mat = tf.transformations.translation_matrix((0.0, 0.1, 0.1))
    tr_Rob_Cam_vec = tf.transformations.translation_from_matrix(tr_Rob_Cam_mat)

    # Transform
    camera_transform.transform.translation.x = tr_Rob_Cam_vec[0]
    camera_transform.transform.translation.y = tr_Rob_Cam_vec[1]
    camera_transform.transform.translation.z = tr_Rob_Cam_vec[2]


    # Compute matrix from camera to object
    T_Rob_Cam = numpy.dot(tf.transformations.quaternion_matrix(tf.transformations.quaternion_about_axis(-1.0, (0,0,0))), tr_Rob_Cam_mat)
    T_Rob_Cam_inverse = tf.transformations.inverse_matrix(T_Rob_Cam)
    T_Bas_Rob_inverse = tf.transformations.inverse_matrix(T_Bas_Rob)

    T_Cam_Obj = numpy.dot(numpy.dot(T_Rob_Cam_inverse, T_Bas_Rob_inverse), T_Bas_Obj)
    
    # Vector from camera to object
    Vect_Cam_Obj = tf.transformations.translation_from_matrix(T_Cam_Obj)

    # Define a X-axis
    matX = tf.transformations.translation_matrix((1.0, 0.0, 0.0))
    vecX = tf.transformations.translation_from_matrix(matX)
    
    # Find the angle of rotation by the dot product, result into arccos (all vectors must be normalized)
    angle = numpy.arccos(numpy.dot(Vect_Cam_Obj, vecX) / (numpy.linalg.norm(Vect_Cam_Obj) * numpy.linalg.norm(vecX)))

    # Find the axis to rotate by the cross product (all vectors must be normalized)
    axis = numpy.cross(Vect_Cam_Obj, vecX)

    # This is the rotation angles
    rob_Cam_rotationAngles = tf.transformations.quaternion_about_axis(-angle, (axis[0], axis[1], axis[2]))

    rob_Cam_Matrix = numpy.dot(tf.transformations.quaternion_matrix(rob_Cam_rotationAngles), tf.transformations.translation_matrix((0.0, 0.0, 0.0)))
    rob_Cam_rotationMatrix = tf.transformations.quaternion_from_matrix(rob_Cam_Matrix)

    # Rotation
    camera_transform.transform.rotation.x = rob_Cam_rotationMatrix[0]
    camera_transform.transform.rotation.y = rob_Cam_rotationMatrix[1]
    camera_transform.transform.rotation.z = rob_Cam_rotationMatrix[2]
    camera_transform.transform.rotation.w = rob_Cam_rotationMatrix[3]


    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('camera_robot')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
