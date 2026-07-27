
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import argparse

def read_bag_and_extract_trajectory(bag_file, odom_topic_name):
    # read bag file
    bag = rosbag.Bag(bag_file)
    
    x_vals = []
    y_vals = []
    z_vals = []
    time_stamps = []

    # read nav_msgs/Odometry message
    for topic, msg, t in bag.read_messages(topics=[odom_topic_name]):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        x_vals.append(x)
        y_vals.append(y)
        z_vals.append(z)
        time_stamps.append(t.to_sec())

    bag.close()
    
    return np.array(x_vals), np.array(y_vals), np.array(z_vals), np.array(time_stamps)

def plot_trajectory_and_deltas(x_vals, y_vals, z_vals, time_stamps, odom_topic):
    # compute position delta
    delta_x0 = np.diff(x_vals[0])
    delta_y0 = np.diff(y_vals[0])
    delta_z0 = np.diff(z_vals[0])
    
    delta_x1 = np.diff(x_vals[1])
    delta_y1 = np.diff(y_vals[1])
    delta_z1 = np.diff(z_vals[1])
    
    ylim = (-0.07, 0.07)
    alpha_val = 1
    fig, axs = plt.subplots(3, 1, figsize=(20, 15))
    # plot x axis differ
    axs[0].plot(time_stamps[0][1:], delta_x0, label=odom_topic[0] + '/X', alpha=alpha_val)
    axs[0].plot(time_stamps[1][1:], delta_x1, label=odom_topic[1] + '/X', alpha=alpha_val)
    axs[0].set_ylim(ylim)
    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Delta X')
    axs[0].legend(loc="upper right")
    axs[0].grid(True)
    
    # plot y axis differ
    axs[1].plot(time_stamps[0][1:], delta_y0, label=odom_topic[0] + '/Y', alpha=alpha_val)
    axs[1].plot(time_stamps[1][1:], delta_y1, label=odom_topic[1] + '/Y', alpha=alpha_val)
    axs[1].set_ylim(ylim)
    axs[1].set_xlabel('Time [s]')
    axs[1].set_ylabel('Delta Y')
    axs[1].legend(loc="upper right")
    axs[1].grid(True)
    
    # plot z axis differ
    axs[2].plot(time_stamps[0][1:], delta_z0, label=odom_topic[0] + '/Z', alpha=alpha_val)
    axs[2].plot(time_stamps[1][1:], delta_z1, label=odom_topic[1] + '/Z', alpha=alpha_val)
    axs[2].set_ylim(ylim)
    axs[2].set_xlabel('Time [s]')
    axs[2].set_ylabel('Delta Z')
    axs[2].legend(loc="upper right")
    axs[2].grid(True)
    # plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='''Analyze trajectories''')
    parser.add_argument('--bag_path', help="Bag Path", default=None)
    parser.add_argument('--odom_topic0', help='Imu Rate Odometry 0', default="/far_vins_node/far_imu_propagation")
    parser.add_argument('--odom_topic1', help='Imu Rate Odometry 1', default="/vins_node/imu_propagate")
    args = parser.parse_args()
    if (args.bag_path == None):
        print("Please Check Your Bag FILE Path")
        exit()
    print("Odometry0 Topic is: ", args.odom_topic0)
    print("Odometry1 Topic is: ", args.odom_topic1)
    bag_file = args.bag_path
    odom0_topic = args.odom_topic0
    odom1_topic = args.odom_topic1
    
    # read bag and extractor odometry message
    x_vals0, y_vals0, z_vals0, time_stamps0 = read_bag_and_extract_trajectory(bag_file, odom0_topic)
    x_vals1, y_vals1, z_vals1, time_stamps1 = read_bag_and_extract_trajectory(bag_file, odom1_topic)
    
    # plot result
    plot_trajectory_and_deltas([x_vals0,x_vals1], [y_vals0,y_vals1], 
                               [z_vals0,z_vals1], [time_stamps0,time_stamps1],
                               [odom0_topic, odom1_topic])
