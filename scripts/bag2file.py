##################################################
###  Translate and dissect data from bag file  ###
##################################################
# Read bag file and obtain sonar_image and save
#------------------------------------------------#
import rosbag, os, shutil

# Define target msg type
targetMsgType = 'acoustic_msgs/SonarImage'

# Obtain bag file lists
bagList = filter(lambda x: x[-4:] == '.bag', os.listdir('./'))

# Loop through all bag files in the currentdirectory
for fileI in range(len(bagList)):
  # Obtain filename without extension
  filename = os.path.splitext(bagList[fileI])[0]
  print('Translating ' + filename + '... (' + repr(fileI+1) + '/' + repr(len(bagList)) + ')')

  # Read bag file
  bag = rosbag.Bag('./' + filename + '.bag')
  topic = '/raven/oculus/sonar_image'
  # Obtain topic list
  topics = bag.get_type_and_topic_info()[1].keys()
  # Search topic with target msg type
  for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
    if bag.get_type_and_topic_info()[1].values()[i][0] == targetMsgType:
      topic = bag.get_type_and_topic_info()[1].keys()[i]

  # Make a folder with the filename
  if not os.path.exists(filename):
    os.makedirs(filename)
  else:
    shutil.rmtree(filename)

  # Read bag data and save
  counter = 0
  for topic, msg, t in bag.read_messages(topics=topic):
    # make folder for each sequence starting from 1
    counter = counter + 1
    if not os.path.exists(filename + '/' + repr(counter)):
      os.makedirs(filename + '/' + repr(counter))
    # write sequence number
    f = open(filename + '/' + repr(counter) + '/sequence', 'a')
    f.write(repr(msg.header.seq)); f.close()
    # write time
    f = open(filename + '/' + repr(counter) + '/time', 'a')
    f.write(repr(msg.header.stamp.secs)); f.close()
    # write frequency
    f = open(filename + '/' + repr(counter) + '/frequency', 'a')
    f.write(repr(msg.frequency)); f.close()
    # write sound_speed
    f = open(filename + '/' + repr(counter) + '/sound_speed', 'a')
    f.write(filename + '/' + repr(msg.sound_speed)); f.close()
    # write azimuth_beamwidth
    f = open(filename + '/' + repr(counter) + '/azimuth_beamwidth', 'a')
    f.write(repr(msg.azimuth_beamwidth)); f.close()
    # write elevation_beamwidth
    f = open(filename + '/' + repr(counter) + '/elevation_beamwidth', 'a')
    f.write(repr(msg.elevation_beamwidth)); f.close()
    # write azimuth_angles
    f = open(filename + '/' + repr(counter) + '/azimuth_angles', 'a')
    for data in msg.azimuth_angles:
      f.write(repr(data) + '\n')
    f.close()
    # write elevation_angles
    f = open(filename + '/' + repr(counter) + '/elevation_angles', 'a')
    for data in msg.elevation_angles:
      f.write(repr(data) + '\n')
    f.close()
    # write ranges
    f = open(filename + '/' + repr(counter) + '/ranges', 'a')
    for data in msg.ranges:
      f.write(repr(data) + '\n')
    f.close()
    # write is_bigendian
    f = open(filename + '/' + repr(counter) + '/is_bigendian', 'a')
    f.write(repr(msg.is_bigendian)); f.close()
    # write data_size
    f = open(filename + '/' + repr(counter) + '/data_size', 'a')
    f.write(repr(msg.data_size)); f.close()
    # write intensities
    f = open(filename + '/' + repr(counter) + '/intensities', 'a')
    for data in msg.intensities:
      f.write(repr(ord(data)) + '\n')
    f.close()