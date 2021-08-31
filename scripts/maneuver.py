#!/usr/bin/env python
import bagpy, yaml
from bagpy import bagreader
import pandas as pd
import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
from acoustic_msgs.msg import SonarImage
from gazebo_msgs.msg import ModelState
from tf2_msgs.msg import TFMessage
from gazebo_msgs.srv import SetModelState, GetModelState
from tf.transformations import quaternion_from_euler, quaternion_multiply

# Target time sequence number
targets = [50, 150, 280, 400]

# Target bagfile
filename = '/home/woensug/Downloads/2021-08-20-Data/poses_raven_2021-08-05-19-20-14.bag'


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

    print(self.df_exp_tf.transforms[0][1:-1])
    print(self.df_exp_tf.transforms[1][1:-1])

    # Set sonar model state
    yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[0][1:-1], Loader=yaml.FullLoader)
    set_state_msg_sonar = ModelState()
    set_state_msg_sonar.model_name = 'oculus_m1200d'
    set_state_msg_sonar.pose.position.x = yaml_exp_tf['transform']['translation']['x']
    set_state_msg_sonar.pose.position.y = yaml_exp_tf['transform']['translation']['z'] - 0.6604
    set_state_msg_sonar.pose.position.z = yaml_exp_tf['transform']['translation']['y'] + 0.9144
    orientation = [yaml_exp_tf['transform']['rotation']['x'], yaml_exp_tf['transform']['rotation']['y'], \
                   yaml_exp_tf['transform']['rotation']['z'], yaml_exp_tf['transform']['rotation']['w']]
    orientation = quaternion_multiply(orientation, self.rotate)
    set_state_msg_sonar.pose.orientation.x = orientation[0]
    set_state_msg_sonar.pose.orientation.y = orientation[1]
    set_state_msg_sonar.pose.orientation.z = orientation[2]
    set_state_msg_sonar.pose.orientation.w = orientation[3]
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(set_state_msg_sonar)
    except rospy.ROSInterruptException:
      print("Set model state service failed.")

    # yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[0][1:-1], Loader=yaml.FullLoader)
    # print(yaml_exp_tf['transform'])
    # yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[1][1:-1], Loader=yaml.FullLoader)
    # print(yaml_exp_tf['transform'])
    # yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[2][1:-1], Loader=yaml.FullLoader)
    # print(yaml_exp_tf['transform'])
    # yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[3][1:-1], Loader=yaml.FullLoader)
    # print(yaml_exp_tf['transform'])

    # for i in range(len(self.df_image_raw_exp)):
    #   yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[i][1:-1], Loader=yaml.FullLoader)
    #   print(yaml_exp_tf['header']['stamp']['secs'])
    #   print(self.df_image_raw_exp['header.stamp.secs'][i])
    #   print("  ")

    # self.exp_image_raw = b.message_by_topic('/raven/oculus/sonar_image')
    # self.df_exp_image_raw = pd.read_csv(self.exp_image_raw)
    # # self.exp_azimuth_beamwidth =
    # self.exp_intensities = [ord(c) for c in self.df_exp_image_raw.intensities[0]]
    # # print(len(self.exp_intensities))
    # # print(self.df_exp_image_raw.dtypes)
    # print(self.df_exp_image_raw.columns.values.tolist())
    # print(" ")
    # print(self.df_exp_image_raw.iloc[:,9][0])
    # print(" ")
    # print(self.df_exp_image_raw.iloc[:,8][0])

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

    # Search with timestamp sec
    yaml_exp_tf = yaml.load(self.df_exp_tf.transforms[self.tf_update_counter][1:-1], Loader=yaml.FullLoader)

    # Model state - plate
    set_state_msg = ModelState()
    set_state_msg.model_name = 'plate_model'
    set_state_msg.pose.position.x = yaml_exp_tf['transform']['translation']['x'] + 0.2475
    set_state_msg.pose.position.y = yaml_exp_tf['transform']['translation']['z'] - 0.6604 + 0.18
    set_state_msg.pose.position.z = yaml_exp_tf['transform']['translation']['y'] + 0.9144
    orientation = [yaml_exp_tf['transform']['rotation']['x'], yaml_exp_tf['transform']['rotation']['y'], \
                   yaml_exp_tf['transform']['rotation']['z'], yaml_exp_tf['transform']['rotation']['w']]
    orientation = quaternion_multiply(quaternion_multiply(orientation, self.rotate), self.rotate)
    set_state_msg.pose.orientation.x = orientation[0]
    set_state_msg.pose.orientation.y = orientation[1]
    set_state_msg.pose.orientation.z = orientation[2]
    set_state_msg.pose.orientation.w = orientation[3]

    rospy.wait_for_service("/gazebo/set_model_state")
    try:
      set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
      resp = set_state(set_state_msg)
      set_state_msg.model_name = 'bolt_model'
      resp = set_state(set_state_msg)
    except rospy.ROSInterruptException:
      print("Set model state service failed.")

    # Update tf counter
    self.tf_update_counter += 2

    return

if __name__ == '__main__':
  # Start node
  rospy.init_node('compare')

  # Initiate node object
  node = Node()

  # Spin
  r = rospy.Rate(30) # 10hz
  try:
    while not rospy.is_shutdown():
      node.update()
      r.sleep()
  except rospy.ROSInterruptException:
    pass