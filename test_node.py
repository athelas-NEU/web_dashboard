#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import Float32MultiArray
import argparse
import math
import time
import numpy as np

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text


def float_or_str(text):
    """Helper function for argument parsing."""
    try:
        return float(text)
    except ValueError:
        return text

def talker(args):

	rospy.init_node('test_node', anonymous=True)
	rate = rospy.Rate(args.rate) # 40hz (loops forty times per second)

	print("Starting test topics.")

	test_pub = rospy.Publisher(args.topic, Float32MultiArray, queue_size=10)
	test_msg = Float32MultiArray()

	try:
		while not rospy.is_shutdown():
			arr = [random.uniform(args.min, args.max)]
			test_msg.data = arr
			test_pub.publish(test_msg)
			rate.sleep()

	except rospy.ROSInterruptException:
		return


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Run a test ROS publisher for float arrays.')
	parser.add_argument(
		'-t', '--topic', type=str, default="/biosensors/stethoscope",
		help='the topic to publish to')
	parser.add_argument(
		'-min', '--min', type=float_or_str, default=90,
		help='minimum value for random values')
	parser.add_argument(
		'-max', '--max', type=float_or_str, default=95,
		help='maxmimum value for random values')
	parser.add_argument(
		'-r', '--rate', type=int_or_str, default=40,
		help='the rate in Hz for this publisher')
	args, remaining = parser.parse_known_args()

	print("Config: ", args)

	talker(args)

	print("Exiting.")
