import numpy as np
import rospkg
import rosbag

# Load bag file
bag = rosbag.Bag("cw3q5/bag/cw3q5.bag")
rospack = rospkg.RosPack()
path = rospack.get_path('cw3q5') + '/bag/cw3q5.bag'
bag = rosbag.Bag(path)

# Topic, number of messages & message types
info = bag.get_type_and_topic_info()

topic_traj = next(iter(info.topics.keys()))
msg_type = info.topics[topic_traj].msg_type  # JointTrajectory()
msg_count = info.topics[topic_traj].message_count

print(f"Topic name: {topic_traj}")
print(f"Message type: {msg_type}")
print(f"Number of messages: {msg_count}")

# Message content
# includes header, joint_names, & points = JointTrajectoryPoint()
for _, msg, _ in bag.read_messages(topics=[topic_traj]):
    
    print(f"Number of joints: {len(msg.joint_names)}")
    print(f"Number of waypoints: {len(msg.points)}")
    
    for i, point in enumerate(msg.points):  # JointTrajectoryPoint() 
    # includes positions, velocities, accelerations, effort, time_from_start

        print(f"Waypoint_{i+1}: {point}")