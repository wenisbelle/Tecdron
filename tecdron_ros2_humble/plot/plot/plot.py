#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import numpy as np

class Plot(Node):
    def __init__(self):
        super().__init__('plot')
        
        self.declare_parameter('real_life', False)
        self.declare_parameter('odom_topics', ['odom'])
        self.declare_parameter('true_odom', 'true_odom')
        
        self.real_life = self.get_parameter('real_life').value
        self.odom_topics = self.get_parameter('odom_topics').value
        self.true_odom = self.get_parameter('true_odom').value
        
        self.odom_data = {topic: {'x': [], 'y': [], 'start_x': None, 'start_y': None} for topic in self.odom_topics}
        self.ground_truth = {'x': [], 'y': [], 'vx': [], 'vy': [], 'start_x': None, 'start_y': None}
        self.errors = {topic: [] for topic in self.odom_topics}
        
        for topic in self.odom_topics:
            self.create_subscription(Odometry, topic, self.odom_callback(topic), 10)
        
        if not self.real_life:
            self.create_subscription(Odometry, self.true_odom, self.ground_truth_callback(topic), 10)
        
        # Initialize the plot
        plt.ion()
        if not self.real_life:
            self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))
            self.ax2.set_xlabel('Time')
            self.ax2.set_ylabel('Error')
            self.ax2.set_title('Odometry Discrepancies')
        else:
            self.fig, self.ax1 = plt.subplots(1, 1, figsize=(6, 5))
        
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Odometry Investigation')
        
        plt.show()

        self.create_timer(0.1, self.update_plot)
        
        if not self.real_life:
            self.create_timer(0.1, self.calculate_errors)

    def odom_callback(self, topic):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if self.odom_data[topic]['start_x'] is None:
                self.odom_data[topic]['start_x'] = x
                self.odom_data[topic]['start_y'] = y
            self.odom_data[topic]['x'].append(x - self.odom_data[topic]['start_x'])
            self.odom_data[topic]['y'].append(y - self.odom_data[topic]['start_y'])
        return callback
    
    def ground_truth_callback(self, topic):
        def callback(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            if self.ground_truth['start_x'] is None:
                self.ground_truth['start_x'] = x
                self.ground_truth['start_y'] = y
            self.ground_truth['x'].append(x - self.ground_truth['start_x'])
            self.ground_truth['y'].append(y - self.ground_truth['start_y'])
            self.ground_truth['vx'].append(vx)
            self.ground_truth['vy'].append(vy)
        
        #Py plot a test message
        print("Ground truth callback")


        return callback
        
        

    def calculate_errors(self):
        if self.ground_truth['x'] and self.ground_truth['vx'] and self.ground_truth['vy']:
            velocity = np.sqrt(self.ground_truth['vx'][-1]**2 + self.ground_truth['vy'][-1]**2)
            if velocity < 0.01:
                for topic in self.odom_topics:
                    if self.odom_data[topic]['x']:
                        error = np.sqrt((self.ground_truth['x'][-1] - self.odom_data[topic]['x'][-1])**2 +
                                        (self.ground_truth['y'][-1] - self.odom_data[topic]['y'][-1])**2)
                        self.errors[topic].append(error)
    
    def update_plot(self):
        self.ax1.clear()
        
        self.ax1.set_xlabel('X')
        self.ax1.set_ylabel('Y')
        self.ax1.set_title('Odometry Investigation')
        
        for topic in self.odom_topics:
            if self.odom_data[topic]['x']:
                self.ax1.plot(self.odom_data[topic]['x'], self.odom_data[topic]['y'], label=topic)
        
        if not self.real_life and self.ground_truth['x']:
            self.ax1.plot(self.ground_truth['x'], self.ground_truth['y'], label='Ground Truth')
            
            self.ax2.clear()
            self.ax2.set_xlabel('Time')
            self.ax2.set_ylabel('Error')
            self.ax2.set_title('Odometry Discrepancies')
            for topic in self.odom_topics:
                if self.errors[topic]:
                    self.ax2.plot(range(len(self.errors[topic])), self.errors[topic], label=f'{topic} Error')
            self.ax2.legend()
        
        self.ax1.legend()
        
        self.ax1.set_aspect('equal', 'box')

        # Set axis limits to ensure a minimum axis length of 2.0
        x_min, x_max = self.ax1.get_xlim()
        y_min, y_max = self.ax1.get_ylim()
        
        x_range = x_max - x_min
        y_range = y_max - y_min
        
        if x_range < 2.0:
            x_center = (x_min + x_max) / 2
            x_min = x_center - 1.0
            x_max = x_center + 1.0
        
        if y_range < 2.0:
            y_center = (y_min + y_max) / 2
            y_min = y_center - 1.0
            y_max = y_center + 1.0
        
        self.ax1.set_xlim(x_min, x_max)
        self.ax1.set_ylim(y_min, y_max)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    plot = Plot()
    try:
        rclpy.spin(plot)
    except KeyboardInterrupt:
        pass
    finally:
        plot.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()