import rclpy, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Image
rclpy.init(); n=Node('t'); c={'clock':0,'scan':0,'rgb':0,'depth':0}; enc={}
def inc(k): 
    def f(m):
        c[k]+=1
        if hasattr(m,'encoding'): enc[k]=(m.encoding,m.width,m.height)
    return f
n.create_subscription(Clock,'/clock',inc('clock'),10)
n.create_subscription(LaserScan,'/scan',inc('scan'),qos_profile_sensor_data)
n.create_subscription(Image,'/rgbd/image',inc('rgb'),qos_profile_sensor_data)
n.create_subscription(Image,'/rgbd/depth_image',inc('depth'),qos_profile_sensor_data)
t=time.time()
while time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.1)
print("COUNTS 10s:",c,enc)
