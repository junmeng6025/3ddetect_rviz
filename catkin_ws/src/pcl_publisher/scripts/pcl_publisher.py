#!/usr/bin/env python3
import os
import numpy as np
import open3d as o3d
import rospy
import tf as ros_tf
import sensor_msgs.point_cloud2 as pcl2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from pathlib import Path

FRAME_ID = "map"
QUEUE_SIZE = 10
FPS = 3
LIFE_TIME = 1.0/FPS
ROOT = str(Path(Path.cwd()).parent)
PCL_PATH = ROOT+"/cp_vis/pc"
GT_PATH = ROOT+"/catkin_ws/src/pcl_publisher/obj/gt"


def read_pcl(pcl_path, pcl_filname):
    # pcl_filname = "11_16_11_14_%d.npy" % frame
    flow_dict = np.load(os.path.join(pcl_path, pcl_filname), allow_pickle=True)

    # with open (path, 'rb') as f:
    #     flow_dict = pickle.load(f)

    data_org = flow_dict['points_c'].squeeze()  # .permute(1, 0)
    data_org = data_org[:, :3]

    # points = np.array(
    #     [[225.0, -71.0, 819.8], [237.0, -24.0, 816.0], [254.0, -82.0, 772.3]])
    return data_org.numpy()


# def read_gt(gt_path, gt_filname):
#     gt = o3d.io.read_triangle_mesh(os.path.join(gt_path, gt_filname))
#     return gt


def publish_pcl(pcl_pub, points):
    msg = PointCloud2()
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id = FRAME_ID

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = False
    msg.data = np.asarray(points, np.float32).tostring()

    pcl_pub.publish(msg)


def publish_bbox(bbox_pub, obj_resource, rgb, alpha):
    mesh_marker = Marker()
    mesh_marker.header.frame_id = FRAME_ID
    mesh_marker.header.stamp = rospy.Time.now()
    mesh_marker.id = 1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.mesh_resource = obj_resource

    mesh_marker.pose.position.x = 0.0
    mesh_marker.pose.position.y = 0.0
    mesh_marker.pose.position.z = 0.0

    q = ros_tf.transformations.quaternion_from_euler(
        0, 0, -np.pi/2)  # configure the .dae model's orientation
    mesh_marker.pose.orientation.x = q[0]
    mesh_marker.pose.orientation.y = q[1]
    mesh_marker.pose.orientation.z = q[2]
    mesh_marker.pose.orientation.w = q[3]

    mesh_marker.color.r = rgb[0]
    mesh_marker.color.g = rgb[1]
    mesh_marker.color.b = rgb[2]
    mesh_marker.color.a = alpha

    mesh_marker.scale.x = 1.0
    mesh_marker.scale.y = 1.0
    mesh_marker.scale.z = 1.0

    bbox_pub.publish(mesh_marker)


def talker():
    # declare publishers
    pcl_pub = rospy.Publisher('/pcl', PointCloud2, queue_size=QUEUE_SIZE)
    gtbbox_pub = rospy.Publisher('/gt_bbox', Marker, queue_size=QUEUE_SIZE)
    blbbox_pub = rospy.Publisher('/bl_bbox', Marker, queue_size=QUEUE_SIZE)
    oursbbox_pub = rospy.Publisher('/ours_bbox', Marker, queue_size=QUEUE_SIZE)

    # reday to publish
    rospy.init_node('pointcloud_publisher_node', anonymous=True)
    rate = rospy.Rate(FPS)

    i = 0
    rospy.loginfo("starting... root path: %s" % ROOT)

    while not rospy.is_shutdown():
        # get filename for pc, gt, obj
        frame = 15 + i
        if frame <= 59:
            pcl_filename = "11_17_7_33_%d.npy" % frame
            obj_filename = "11_17_7_33_%d.obj" % frame
        else:
            frame -= 60
            pcl_filename = "11_17_7_34_%d.npy" % frame
            obj_filename = "11_17_7_34_%d.obj" % frame

        # read pc
        points = read_pcl(PCL_PATH, pcl_filename)

        # publish data
        publish_pcl(pcl_pub, points)
        publish_bbox(
            gtbbox_pub,
            "package://pcl_publisher/obj/gt/"+obj_filename,
            [0.0, 1.0, 0.2],
            0.5
        )
        publish_bbox(
            blbbox_pub,
            "package://pcl_publisher/obj/baseline_pred/"+obj_filename,
            [1.0, 69.0/255.0, 0.0],
            0.4
        )
        publish_bbox(
            oursbbox_pub,
            "package://pcl_publisher/obj/ours_pred/"+obj_filename,
            [1.0, 40.0/255.0, 141/255],
            0.4
        )

        rospy.loginfo("publishing [%r] ..." % pcl_filename)

        # node spin
        rate.sleep()
        i += 1
        if i >= 63:
            i = 0


if __name__ == '__main__':
    talker()
