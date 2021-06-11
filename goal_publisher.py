import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from geometry_msgs.msg import PoseStamped






class GoalPublisher(Node):
  """
  Create a GoalPublisher class, which is a subclass of the Node class.
  The class publishes the goal positions (x,y) for a mobile robot in a Gazebo maze world.
  """
   
  def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('goal_publisher')
      
    # Create publisher(s)      
    self.publisher_goal_x_values = self.create_publisher(PoseStamped, '/goal_pose', 10)
      
    # Call the callback functions every 0.5 seconds
    timer_period = 0.5
    self.timer1 = self.create_timer(timer_period, self.get_x_values)
     
    # Initialize a counter variable
    self.i = 0 
    self.j = 0
      
    # List the goal destinations
    # We create a list of the (x,y) coordinate goals
    self.goal_x = 4.0
    self.goal_y = 0.999
    self.goal_z = 0.0025
                                                                                                    
    
  def get_x_values(self):
    """
    Callback function.
    """
    goalMsg = PoseStamped()
    goalMsg.pose.position.x = self.goal_x
    goalMsg.pose.position.y = self.goal_y
    goalMsg.pose.position.z = self.goal_z
    goalMsg.header.frame_id    = "map"
    
     
    # Publish the x coordinates to the topic
    self.publisher_goal_x_values.publish(goalMsg)
      
    # Increment counter variable
    self.i += 1
      
  
      
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  goal_publisher = GoalPublisher()
  
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  rclpy.spin(goal_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  goal_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

