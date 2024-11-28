#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist 
from gazebo_msgs.msg import ModelStates
from rclpy.executors import MultiThreadedExecutor
import matplotlib.pyplot as plt
import numpy as np

class Plot(Node):
    def __init__(self):
        super().__init__('plot')
        
        self.declare_parameter('real_life', False)
        self.declare_parameter('command', 'cmd_vel')
        self.declare_parameter('true_odom', 'true_odom')
        
        self.real_life = self.get_parameter('real_life').value
        self.command_topic = self.get_parameter('command').value
        self.true_odom = self.get_parameter('true_odom').value
        
        self.command_data = {'vx': [], 'vy': [], 'vtta': []}
        self.ground_truth = {'vx': [], 'vy': [], 'vtta': []}
        self.errors = {'error_vx': [], 'error_vy': [], 'error_vtta': []}
        
        self.create_subscription(Twist, self.command_topic, self.command_callback, 10)
        
        if not self.real_life:
            self.create_subscription(Odometry, self.true_odom, self.ground_truth_callback, 10)

        
        
        # Initialize the plot
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 3, figsize=(12, 5))
                
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Error Vx')
        self.ax1.set_title('Vx Investigation')

        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Error Vy')
        self.ax2.set_title('Vy Investigation')

        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('Error Vtta')
        self.ax3.set_title('Vtta Investigation')
        
        plt.show()

        self.create_timer(0.05, self.update_plot)
        
        self.create_timer(0.005, self.calculate_errors)

    def command_callback(self, msg):
        cmd_vx = msg.linear.x
        cmd_vy = msg.linear.y
        cmd_vtta = msg.angular.z
        self.command_data['vx'].append(cmd_vx)
        self.command_data['vy'].append(cmd_vy)
        self.command_data['vtta'].append(cmd_vtta)


    
    def ground_truth_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vtta = msg.twist.twist.angular.z
        self.ground_truth['vx'].append(vx)
        self.ground_truth['vy'].append(vy)
        self.ground_truth['vtta'].append(vtta)
        
        #Py plot a test message
        #print("Ground truth callback")
  
        

    def calculate_errors(self):
        error_vx = self.command_data['vx'][-1] - self.ground_truth['vx'][-1]
        error_vy = self.command_data['vy'][-1] - self.ground_truth['vy'][-1]
        error_vtta = self.command_data['vtta'][-1] - self.ground_truth['vtta'][-1]

        self.errors['error_vx'].append(error_vx)
        self.errors['error_vy'].append(error_vy)    
        self.errors['error_vtta'].append(error_vtta)
    
    def update_plot(self):
        self.ax1.clear()
        
        self.ax1.set_xlabel('Time')
        self.ax1.set_ylabel('Error Vx')
        self.ax1.set_title('Vx Investigation')
        
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