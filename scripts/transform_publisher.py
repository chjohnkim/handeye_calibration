from geometry_msgs.msg import Transform
import rospy
import os
import csv
import argparse

class TransformPublisher():
    def __init__(self, data_dir):
        self.data_dir = data_dir

        rospy.init_node('transform_publisher', anonymous=True)
        self.world_effector_pub = rospy.Publisher('world_effector', Transform, queue_size=10)
        self.camera_object_pub = rospy.Publisher('camera_object', Transform, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        self.world_effector_ls = []
        self.camera_object_ls = []
            
        with open(os.path.join(self.data_dir, 'effector2world_transform.txt')) as file:
            csvreader = csv.reader(file)
            for row in csvreader:
                self.world_effector_ls.append(row)
        with open(os.path.join(self.data_dir, 'object2cam_transform.txt')) as file:
            csvreader = csv.reader(file)
            for row in csvreader:
                self.camera_object_ls.append(row)
        self.publish()
        print('Done')

    def publish(self):
        for i, (w, c) in enumerate(zip(self.world_effector_ls, self.camera_object_ls)):
            world_effector_tf = Transform()
            world_effector_tf.translation.x = float(w[0])
            world_effector_tf.translation.y = float(w[1])
            world_effector_tf.translation.z = float(w[2])
            world_effector_tf.rotation.x = float(w[3])
            world_effector_tf.rotation.y = float(w[4])
            world_effector_tf.rotation.z = float(w[5])
            world_effector_tf.rotation.w = float(w[6])

            camera_object_tf = Transform()
            camera_object_tf.translation.x = float(c[0])
            camera_object_tf.translation.y = float(c[1])
            camera_object_tf.translation.z = float(c[2])
            camera_object_tf.rotation.x = float(c[3])
            camera_object_tf.rotation.y = float(c[4])
            camera_object_tf.rotation.z = float(c[5])
            camera_object_tf.rotation.w = float(c[6])

            self.world_effector_pub.publish(world_effector_tf)
            self.camera_object_pub.publish(camera_object_tf)
            print('*'*i, i)
            self.rate.sleep()


def parse_args():
    return args


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract data from bag file')
    parser.add_argument('-d', '--data_dir', required=True, type=str, help='path of data directory')
    args = parser.parse_args()
    be = TransformPublisher(args.data_dir)