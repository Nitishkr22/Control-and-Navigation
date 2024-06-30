#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import threading

class UserValuePublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('user_value_publisher', anonymous=True)

        # Create a publisher with topic 'user_value' and message type Float32
        self.pub = rospy.Publisher('user_value', Float32, queue_size=10)

        # Rate at which to publish (in Hz)
        self.rate = rospy.Rate(10)  # 1 Hz

        # Initialize user_value with 0.0
        self.user_value = 0.0

        # Flag to indicate if a new value has been received
        self.new_value_received = False

        # Create a lock for thread safety
        self.lock = threading.Lock()

    def input_callback(self, user_input):
        # Callback function to receive new input value from the user
        try:
            new_value = float(user_input.data)
            with self.lock:
                self.user_value = new_value
                self.new_value_received = True
            rospy.loginfo("New value received: %.2f" % self.user_value)
        except ValueError:
            rospy.logerr("Invalid input. Please enter a valid floating point number.")

    def user_input_thread(self):
        # Thread for receiving user input continuously
        while not rospy.is_shutdown():
            user_input = input("Enter a new value to publish: ")
            with self.lock:
                self.user_value = float(user_input)
                self.new_value_received = True

    def run(self):
        # Subscribe to the topic where user input will be published
        rospy.Subscriber('user_input', Float32, self.input_callback)

        # Start a thread for continuously receiving user input
        input_thread = threading.Thread(target=self.user_input_thread)
        input_thread.daemon = True  # Exit when the main program exits
        input_thread.start()

        while not rospy.is_shutdown():
            with self.lock:
                if self.new_value_received:
                    # Publish the new user-provided value continuously until a new value is received
                    while self.new_value_received and not rospy.is_shutdown():
                        rospy.loginfo("Publishing: %.2f" % self.user_value)
                        self.pub.publish(self.user_value)
                        self.rate.sleep()

                    # Reset flag once value has been published continuously
                    self.new_value_received = False

            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = UserValuePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
