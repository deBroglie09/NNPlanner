from numpy import loadtxt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Normalization
from tensorflow import keras
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import  Bool
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import math
import time

current_position = np.zeros(3)
current_velocity = np.zeros(3)
last_velocity = np.zeros(3)
current_acc = np.zeros(3)
current_acc[2] = 9.81
current_rate = np.zeros(3)
is_ready = 0

ros_freq = 30.0

def odometry_cb(data):
    global current_position
    current_position = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
    global current_velocity
    current_velocity = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z])
    global current_rate
    current_rate = np.array([data.twist.twist.angular.x,data.twist.twist.angular.y,data.twist.twist.angular.z])
    global current_acc
    global last_velocity
    current_acc = (current_velocity - last_velocity) * ros_freq
    current_acc[2] += 9.81
    last_velocity = current_velocity


def status_cb(data):
    global is_ready
    is_ready = data.data

    
if __name__ == "__main__":
    

    waypoint0 = np.array([2.0, 0.5, 1.5])
    waypoint1 = np.array([4.0, 0.0, 1.0])
    current_lambda = [1, 1]
    if is_ready == False:
        time_start = time.time()
    k_waypoint0 = np.array([0,0.5,0])
    time_sum= 0

    rospy.init_node("planner")

    odom_sub = rospy.Subscriber('/uav0/mavros/local_position/odom',Odometry,odometry_cb)
    status_sub = rospy.Subscriber('/offb_node/status',Bool,status_cb)
    
    position_planner_pub = rospy.Publisher('planner/position', Vector3Stamped, queue_size=1)
    position_setpoint = Vector3Stamped()
    velocity_planner_pub = rospy.Publisher('planner/velocity', Vector3Stamped, queue_size=1)
    velocity_setpoint = Vector3Stamped()
    attitude_planner_pub = rospy.Publisher('planner/attitude', Vector3Stamped, queue_size=1)
    attitude_setpoint = Vector3Stamped()
    rate_planner_pub = rospy.Publisher('planner/rate', Vector3Stamped, queue_size=1)
    rate_setpoint = Vector3Stamped()
    command_planner_pub = rospy.Publisher('planner/command', Vector3Stamped, queue_size=1)
    command_setpoint = Vector3Stamped()

    rate = rospy.Rate(ros_freq)

    model = keras.models.load_model('/home/leo/BP-training/model/quad5_m5.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    val = keras.models.load_model('/home/leo/BP-training/model/quad5_val.h5')
    # dataset = loadtxt('/home/zhoujin/trajectory-generation/trajectory/quad2.txt', delimiter=',')
    # split into input (X) and output (y) variables
    # model = keras.models.load_model('/home/zhoujin/learning/model/model1')
    # input_test = np.ones(15)
    # print(model(input_test.reshape(-1,15)))

    # X = dataset[:,0:15]
    # y = dataset[:,18:36]


    # min_max_scaler = MinMaxScaler()
    # min_max_scaler.fit(X)
    # scalerX = min_max_scaler
    # # X = min_max_scaler.transform(X)

    # min_max_scalery = MinMaxScaler()
    # min_max_scalery.fit(y)
    # scalery = min_max_scalery
    # y = min_max_scaler.transform(y)

    while not rospy.is_shutdown():
        # print(current_position)
        if is_ready == False:
            time_end = time.time()    
            time_sum=(time_end - time_start)+time_sum 
            time_start = time.time() 
            waypoint0 = np.array([2.0 ,  0.5 * k_waypoint0[1] * math.cos(time_sum), 1.5 ])
        print(waypoint0)
        input = np.zeros(15)
        input_val = np.zeros(6)
        error0 = waypoint0 - current_position
        error1 = waypoint1 - current_position
        for i in range(3):
            input[i] = error0[i]
            input[i+3] = error1[i]
            input_val[i] = error0[i]
            input_val[i+3] = error1[i]
            input[i+6] = current_velocity[i]
            input[i+9] = current_acc[i]
            input[i+12] = current_rate[i]
        # input[6] = current_lambda[0]
        # input[7] = current_lambda[1]
        # print(input[0])
        output = model(input.reshape(-1,15))
        output_val = val(input_val.reshape(-1,6))
        # output = model.predict(scalerX.transform(input.reshape(-1,15)))
        # output = scalery.inverse_transform(output)
        # print(output[0,1])

        # if output_val[0, 0] > 0.2 :
        #     position_setpoint.vector.x = ((waypoint0[0] - output[0, 0]) + (waypoint1[0] - output[0, 3])) / 2
        #     position_setpoint.vector.y = ((waypoint0[1] - output[0, 1]) + (waypoint1[1] - output[0, 4])) / 2
        #     position_setpoint.vector.z = ((waypoint0[2] - output[0, 2]) + (waypoint1[2] - output[0, 5])) / 2
        #     position_setpoint.header.stamp = rospy.Time.now()
        #     position_planner_pub.publish(position_setpoint)
        #     print(position_setpoint.vector.z)
        # else:
        #     position_setpoint.vector.x = waypoint1[0] - output[0, 3]
        #     position_setpoint.vector.y = waypoint1[1] - output[0, 4]
        #     position_setpoint.vector.z = waypoint1[2] - output[0, 5]
        #     position_setpoint.header.stamp = rospy.Time.now()
        #     position_planner_pub.publish(position_setpoint)
        #     print(position_setpoint.vector.z)

        position_setpoint.vector.x = ((waypoint0[0] - output[0, 0]) + (waypoint1[0] - output[0, 3])) / 2
        position_setpoint.vector.y = ((waypoint0[1] - output[0, 1]) + (waypoint1[1] - output[0, 4])) / 2
        position_setpoint.vector.z = ((waypoint0[2] - output[0, 2]) + (waypoint1[2] - output[0, 5])) / 2
        position_setpoint.header.stamp = rospy.Time.now()
        position_planner_pub.publish(position_setpoint)
        # print(position_setpoint.vector.z)

        velocity_setpoint.vector.x = output[0, 6]
        velocity_setpoint.vector.y = output[0, 7]
        velocity_setpoint.vector.z = output[0, 8]
        velocity_setpoint.header.stamp = rospy.Time.now()
        velocity_planner_pub.publish(velocity_setpoint)

        attitude_setpoint.vector.x = output[0, 9]
        attitude_setpoint.vector.y = output[0, 10]
        attitude_setpoint.vector.z = output[0, 11]
        attitude_setpoint.header.stamp = rospy.Time.now()
        attitude_planner_pub.publish(attitude_setpoint)
        # print(current_acc[2])
        # print(attitude_setpoint.vector.z)

        rate_setpoint.vector.x = output[0, 12]
        rate_setpoint.vector.y = output[0, 13]
        rate_setpoint.vector.z = output[0, 14]
        rate_setpoint.header.stamp = rospy.Time.now()
        rate_planner_pub.publish(rate_setpoint)
        
        command_setpoint.vector.x = 0
        command_setpoint.vector.y = output_val[0, 0]
        command_setpoint.vector.z = output_val[0, 1]
        command_setpoint.header.stamp = rospy.Time.now()
        command_planner_pub.publish(command_setpoint)
        # if is_ready:
        #     current_lambda = [command_setpoint.vector.y, command_setpoint.vector.z]
        # else:
        #     current_lambda = [1, 1]
        # print(is_ready)

        # rospy.spin()
        rate.sleep()
