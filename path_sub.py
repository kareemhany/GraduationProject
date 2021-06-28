import paho.mqtt.client as mqtt
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import time

def on_disconnect(client, userdata, rc):
    print("disconnecting reason  "  +str(rc))
    client.connect("broker.hivemq.com", 1883, 60)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))


def path_feedback(data):
    #print ("Plan")
    res = ""
    #print ("Before: ", len(data.poses))
    step = 20
    for i in range(0,len(data.poses),step):
        point = "{:0.2f},{:0.2f}_".format(data.poses[i].pose.position.x, data.poses[i].pose.position.y)
        res += point
    print ("After: ", len(res.split('_')))
    client.publish("cic/plan", res)
    print (time.time())

client = mqtt.Client()

def main(args=None):
	global client, x
	# Initialize the rclpy library
	#client = mqtt.Client()
	client.on_connect = on_connect
	client.on_disconnect = on_disconnect
	# Create the node
	rclpy.init(args=args)
	x = Node('values')
	global_plan_sub = x.create_subscription(Path, "plan" , path_feedback, 10) 
	client.connect("broker.hivemq.com", 1883, 60)
	rclpy.spin(x)
	#client.loop_forever()
	
    
    
if __name__ == '__main__':
	main()
