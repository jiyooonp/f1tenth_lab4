#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import math

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        self.scan_callback = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.range_max = 30
        
        # TODO: Publish to drive
        self.pub = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10)
        
        # TODO: set PID gains
        self.kp = 0.5
        self.kd = 0.
        self.ki = 0.4

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.prev_time = self.get_clock().now().to_msg().nanosec
        self.count = 0
        
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = ranges
        window_size = 3

        ranges = np.array(ranges)

        # mean_values = np.convolve(ranges, np.ones(window_size)/window_size, mode='same')
        
        # ranges = mean_values
        # proc_ranges = mean_values.tolist()

        proc_ranges = np.where(
            ranges < self.closest_point, 0, ranges)
        proc_ranges = np.where(
            ranges >= self.range_max, 0, ranges)
        

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        result = []
        start_index = 0
        prev = 0

        for i, num in enumerate(free_space_ranges):

            if num == 0 or abs(prev-num)> 0.2:
                result.append((start_index, i - 1))
                start_index = i
            prev = num

        if start_index ==0:
            result.append((start_index, len(free_space_ranges) - 1))

        print("gaps: " , len(result))

        return result

    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """

        max_distance = -1
        best_point = 0

        window_size = 5

        ranges = np.array(ranges)

        mean_values = np.convolve(ranges, np.ones(window_size)/window_size, mode='same')
        ranges = mean_values

        for i in range(start_i, end_i):
            if ranges[i] > max_distance and ranges[i] < self.range_max:
                max_distance = ranges[i]
                best_point = i
        

        confidence = np.sum(np.where(ranges[start_i:end_i] > max_distance * 0.7, 1, 0))
        return best_point, confidence, max_distance
    
    def pid_controller(self, error):
        """ Implement the PID algorithm. Expert implementation includes anti-windup
        """
        current_time = self.get_clock().now().to_msg().nanosec
        dt = current_time - self.prev_time

        self.integral = (self.integral * self.count + error)/ (self.count + 1)
        self.count += 1

        derivative = (error - self.prev_error) / (dt * 1e-9)

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # print("error: ", round(error, 3), "integral: ", round(self.integral, 3), "derivative: ", round(derivative, 3), "output: ", round(output, 3))

        self.prev_error = error
        self.prev_time = current_time

        return output

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment = data.angle_increment
        self.range_max = data.range_max * 0.8

        # TODO:
        #Find closest point to LiDAR
        self.closest_point = min(ranges)
        # print("closest point: ", self.closest_point)

        sensor_len = len(ranges)
        # get the right and left distance
        right_distance = np.mean(ranges[sensor_len//7: sensor_len//6])
        left_distance = np.mean(ranges[-sensor_len//6: -sensor_len//7])

        #Eliminate all points inside 'bubble' (set them to zero) 
        proc_ranges = self.preprocess_lidar(ranges)

        #Find max length gap 
        gaps = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_confidence = 0
        best_distance = 0
        for start_i, end_i in gaps:
            best_point, confidence, max_distance = self.find_best_point(
                start_i, end_i, proc_ranges)
            if max_distance > best_distance:
                best_distance = max_distance
                best_point = best_point
            # if confidence > best_confidence:
            #     best_confidence = confidence
            #     best_point = best_point

        # print("best point: ", best_point, "len:gap: ", len(gaps), "left: ", left_distance, "right: ", right_distance, "confidence: ", best_confidence)

        # best_point = self.find_best_point(start_i, end_i, proc_ranges)
        print("best point: ", best_point, "max_distance:", max_distance)


        # Set your steering angle based on the best_point_index
        steering_angle = self.angle_min + best_point * self.angle_increment

        self.prev_steering_angle = steering_angle

        # decide steering angle and speed

        # full_hall_length = right_distance + left_distance
        # if right_distance > full_hall_length * 0.65:
        #     print("GOING RIGHT")
        #     angle_offset = -250 * (right_distance - full_hall_length*0.5)/(full_hall_length*0.5) * self.angle_increment
        #     # speed = 0.5
        # elif left_distance > full_hall_length * 0.65:
        #     print("GOING LEFT")
        #     angle_offset = 250 * (left_distance - full_hall_length*0.5) / \
        #         (full_hall_length*0.5) * self.angle_increment
        #     # speed = 0.5
        # else:
        #     angle_offset = 0
        speed = 0.3
        # speed = 2.0



        # steering_angle = steering_angle + angle_offset
        self.prev_steering_angle = steering_angle

        pid_steering_angle = self.pid_controller(steering_angle)
        # print("angle: ",  round(steering_angle, 3), "offset: ",
        #       angle_offset, "total: ", pid_steering_angle)


        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header = data.header

        drive_msg.drive.steering_angle = steering_angle

        drive_msg.drive.speed = speed  # Set your desired speed
        self.pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()