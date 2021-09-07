#!/usr/bin/env python
import bagpy, yaml, math
from bagpy import bagreader
import pandas as pd
import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
from acoustic_msgs.msg import SonarImage
from gazebo_msgs.msg import ModelState
from tf2_msgs.msg import TFMessage
from gazebo_msgs.srv import SetModelState, GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

# Target bagfile
filename = '/home/woensug/Downloads/2021-08-20-Data/poses_raven_2021-08-05-19-20-14.bag'

# Target image
target_sequence = 260

# <!-- plate model (designed using SolidWorks) -->
# <!-- Plate size : 600 mm x 450 mm, origin at left bottom corner-->
# <!-- Assume that the plate is at the same z position as the sonar -->
# <!-- sonar is at 12" below water surface where the water is ~48" deep -->

# <!-- Oculus m1200d -->
# <!-- Sonar placed at 12" below water surface, 20" from the sonar tank wall -->
# <!-- zero roll and pitch, the water is ~48" deep -->

class Node():
  def __init__(self):

    # Init variables
    self.tf_update_counter = 1
    self.image_update_counter = 0
    self.rotate = quaternion_from_euler(0, -1.5708, 0)

    # Init messages
    # self.image = Image()
    self.image_raw = SonarImage()
    self.image_raw_exp = SonarImage()
    self.exp_tf = TFMessage()

    # ---- Read Rosbag data ---- #
    b = bagreader(filename)
    # Read tranforms
    self.exp_tf = b.message_by_topic('/tf')
    self.df_exp_tf = pd.read_csv(self.exp_tf)

    # Read sonar data
    self.exp_image_raw = b.message_by_topic('/raven/oculus/sonar_image')
    self.df_exp_image_raw = pd.read_csv(self.exp_image_raw)

    # Set sonar model state
    yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[0][1:-1], Loader=yaml.FullLoader)
    set_state_msg_sonar = ModelState()
    set_state_msg_sonar.model_name = 'oculus_m1200d'
    set_state_msg_sonar.pose.position.x = yaml_exp_tf['transform']['translation']['x']
    set_state_msg_sonar.pose.position.y = yaml_exp_tf['transform']['translation']['z'] - 0.6604
    set_state_msg_sonar.pose.position.z = yaml_exp_tf['transform']['translation']['y'] + 0.9144
    orientation = quaternion_from_euler(0.0, 0.0, math.pi/2.0)
    set_state_msg_sonar.pose.orientation.x = orientation[0]
    set_state_msg_sonar.pose.orientation.y = orientation[1]
    set_state_msg_sonar.pose.orientation.z = orientation[2]
    set_state_msg_sonar.pose.orientation.w = orientation[3]
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(set_state_msg_sonar)
    except rospy.ROSInterruptException:
      print("Set model state service failed.")

    # Pubs and subs
    self.sub_image_raw = rospy.Subscriber("/oculus_m1200d/sonar_image_raw", SonarImage, self.callback_image_raw)
    # self.pub_orientation = rospy.Publisher("/gazebo/set_model_state", NavSatFix, queue_size=1)

    # Get initial position
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
      self.init_state = model_coordinates(model_name="plate_model")
    except rospy.ROSInterruptException:
      print("Obtaining initial model state failed.")
    # print(self.init_state.pose)

  def callback_image_raw(self, data):
    if self.image_raw.intensities != data.intensities:
      self.image_raw = data
      # image_update_counter increment
      # : 1 for orientation update 2 for reflectivity
      self.image_update_counter += 1

  def update(self):

    self.tf_update_counter += 2

    # Search with timestamp sec
    for i in range(len(self.df_exp_tf)):
      yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[i][1:-1], Loader=yaml.FullLoader)
      if yaml_exp_tf['header']['stamp']['secs'] >= self.df_exp_image_raw['header.stamp.secs'][target_sequence-1]:
        self.tf_update_counter = math.floor(i/2.0)*2 + 1 # Only odd number sequence values are charuco board. (even: oculus)
        for j in range(self.tf_update_counter, len(self.df_exp_tf)):
          yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[j][1:-1], Loader=yaml.FullLoader)
          if yaml_exp_tf['header']['stamp']['nsecs'] >= self.df_exp_image_raw['header.stamp.nsecs'][target_sequence-1]:
            self.tf_update_counter = math.floor(j/2.0)*2 + 1
            # yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[self.tf_update_counter-2][1:-1], Loader=yaml.FullLoader)
            # print('target (sec,nsec) : (' + repr(self.df_exp_image_raw['header.stamp.secs'][target_sequence-1]) \
            #       + ', ' + repr(self.df_exp_image_raw['header.stamp.nsecs'][target_sequence-1]) + ')')
            # print('found  (sec,nsec) : (' + repr(yaml_exp_tf['header']['stamp']['secs']) \
            #       + ', ' + repr(yaml_exp_tf['header']['stamp']['nsecs']) + ')')
            break
        break


    print(self.tf_update_counter)
    # self.tf_update_counter = 1119
    yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[self.tf_update_counter][1:-1], Loader=yaml.FullLoader)

    # Model state - plate
    set_state_msg = ModelState()
    set_state_msg.model_name = 'plate_model'
    set_state_msg.pose.position.x = yaml_exp_tf['transform']['translation']['x'] + 0.2475
    set_state_msg.pose.position.y = yaml_exp_tf['transform']['translation']['z'] - 0.6604 - 0.14625
    set_state_msg.pose.position.z = yaml_exp_tf['transform']['translation']['y'] + 0.9144
    orientation = [yaml_exp_tf['transform']['rotation']['x'], yaml_exp_tf['transform']['rotation']['y'], \
                   yaml_exp_tf['transform']['rotation']['z'], yaml_exp_tf['transform']['rotation']['w']]
    orientation = quaternion_multiply(quaternion_multiply(orientation, self.rotate), self.rotate)
    orientation_euler = euler_from_quaternion(orientation)
    orientation = quaternion_from_euler(orientation_euler[0], orientation_euler[2], orientation_euler[1])
    orientation = quaternion_multiply(quaternion_multiply(orientation, self.rotate), self.rotate)
    set_state_msg.pose.orientation.x = orientation[0]
    set_state_msg.pose.orientation.y = orientation[1]
    set_state_msg.pose.orientation.z = orientation[2]
    set_state_msg.pose.orientation.w = orientation[3]

    rospy.wait_for_service("/gazebo/set_model_state")
    try:
      set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
      resp = set_state(set_state_msg)
      # set_state_msg.model_name = 'bolt_model'
      # resp = set_state(set_state_msg)
    except rospy.ROSInterruptException:
      print("Set model state service failed.")

    return

if __name__ == '__main__':
  # Start node
  rospy.init_node('compare')

  # Initiate node object
  node = Node()
  print("Maneuver initiated!")

  # Spin
  r = rospy.Rate(30) # 10hz
  try:
    while not rospy.is_shutdown():
      node.update()
      r.sleep()
  except rospy.ROSInterruptException:
    pass