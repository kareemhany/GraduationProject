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


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global publisher_goal_values
    
    x,y = msg.payload.decode('utf-8').split(',')
    goalMsg = PoseStamped()
    
    
    inMinX = 0
    inMaxX = 945
    inMinY = 0
    inMaxY = 681
    outMinX = -8.36
    outMaxX = 18
    outMinY = 8.18
    outMaxY = -11.1
    outx = ((float(x)- inMinX) / (inMaxX - inMinX)) * (outMaxX - outMinX) + outMinX
    outy = ((float(y) - inMinY) / (inMaxY - inMinY)) * (outMaxY - outMinY) + outMinY
    goalMsg.pose.position.x = outx
    goalMsg.pose.position.y = outy
    
    goalMsg.header.frame_id    = "map"
    print (goalMsg)
     
    # Publish the x coordinates to the topic
    
    publisher_goal_values.publish(goalMsg)

def path_feedback(data):
    print ("Plan")
    res = ""
    print ("Before: ", len(data.poses))
    step = 20
    for i in range(0,len(data.poses),step):
        point = "{:0.2f},{:0.2f}_".format(data.poses[i].pose.position.x, data.poses[i].pose.position.y)
        res += point
    print ("After: ", len(res.split('_')))
    client.publish("tekomoro/carkyo/plan", res)

def main(args=None):
	global publisher_goal_values
	# Initialize the rclpy library
	rclpy.init(args=args)
	  
	# Create the node
	x = Node('values')
	publisher_goal_values = x.create_publisher(PoseStamped, '/goal_pose', 10) 
	global_plan_sub = x.create_subscription(Path, "/plan" , path_feedback, 1)               
	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message


	client.connect("broker.hivemq.com", 1883, 60)
	client.loop_forever()

if __name__ == '__main__':
	main()
