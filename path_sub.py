import paho.mqtt.client as mqtt
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path




           
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("cic_pose")
    




	
def path_feedback(data):
    print ("Plan")
    res = ""
    print ("Before: ", len(data.poses))
    step = 20
    for i in range(0,len(data.poses),step):
        point = "{:0.2f},{:0.2f}_".format(data.poses[i].pose.position.x, data.poses[i].pose.position.y)
        res += point
    print ("After: ", len(res.split('_')))
    client.publish("cic/plan", res)


def main(args=None):
	global client, publisher_goal_values, x
	# Initialize the rclpy library
	client = mqtt.Client()
	client.on_connect = on_connect
	# Create the node
	rclpy.init(args=args)
	x = Node('values')
	publisher_goal_values = x.create_publisher(PoseStamped, 'goal_pose', 10) 
	global_plan_sub = x.create_subscription(Path, "plan" , path_feedback, 10) 
	client.connect("broker.hivemq.com", 1883, 60)
	rclpy.spin(x)
	#client.loop_forever()
	
    
    
if __name__ == '__main__':
	main()
