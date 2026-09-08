"""Real Nav2 + kinematic synthetic robot; not SLAM/hardware navigation validation."""
import os,pathlib,subprocess,signal,time,math,json,yaml
os.environ.update(ROS_DOMAIN_ID='91',ROS_LOCALHOST_ONLY='1',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp')
os.environ['CYCLONEDDS_URI']='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist,TransformStamped
from tf2_ros import TransformBroadcaster
root=pathlib.Path(__file__).resolve().parents[2];out=root/'data/validation/nav2';out.mkdir(parents=True,exist_ok=True);os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
# Static room with a box between robot and goal. This test deliberately excludes live sensing.
import numpy as np
img=np.full((160,160),255,dtype=np.uint8);img[[0,-1],:]=0;img[:,[0,-1]]=0;img[75:85,90:96]=0
(out/'room.pgm').write_bytes(b'P5\n160 160\n255\n'+img.tobytes());(out/'room.yaml').write_text('image: room.pgm\nresolution: 0.1\norigin: [-8, -8, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
c=yaml.safe_load((root/'src/m20_navigation/config/nav2_light_slam.yaml').read_text())
for k in ['local_costmap','global_costmap']:c[k][k]['ros__parameters']['plugins']=['static_layer','inflation_layer']
(out/'params.yaml').write_text(yaml.safe_dump(c))
rclpy.init();n=rclpy.create_node('kinematic_fixture');pub=n.create_publisher(Odometry,'/odom',10);tf=TransformBroadcaster(n)
x=y=yaw=0.;cmd=[0.,0.,0.];last_cmd=0.;last=time.monotonic();trajectory=[];commands=[]
def command(m):
 global cmd,last_cmd
 cmd=[m.linear.x,m.linear.y,m.angular.z];last_cmd=time.monotonic();commands.append(cmd)
n.create_subscription(Twist,'/cmd_vel',command,10)
def tick():
 global x,y,yaw,last
 now=time.monotonic();dt=min(now-last,.1);last=now;vx,vy,w=cmd if now-last_cmd<.4 else [0.,0.,0.]
 x+=(math.cos(yaw)*vx-math.sin(yaw)*vy)*dt;y+=(math.sin(yaw)*vx+math.cos(yaw)*vy)*dt;yaw+=w*dt
 stamp=n.get_clock().now().to_msg();t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom';t.child_frame_id='base_link';t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
 m=TransformStamped();m.header.stamp=stamp;m.header.frame_id='map';m.child_frame_id='odom';m.transform.rotation.w=1.;tf.sendTransform([m,t])
 o=Odometry();o.header=t.header;o.child_frame_id='base_link';o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation=t.transform.rotation;o.twist.twist.linear.x=vx;o.twist.twist.linear.y=vy;o.twist.twist.angular.z=w;pub.publish(o);trajectory.append([now,x,y,yaw])
n.create_timer(.02,tick)
log=(out/'nav2.log').open('w');p=subprocess.Popen(['/opt/ros/humble/bin/ros2','launch','m20_navigation','light_slam_nav2.launch.py','map:='+str(out/'room.yaml'),'params_file:='+str(out/'params.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def spin_until(f,timeout):
 end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end and p.poll() is None:rclpy.spin_once(n,timeout_sec=.02)
 if not f.done():raise TimeoutError('Nav2 action timeout; launch exit='+str(p.poll()))
 return f.result()
result={'scope':'Nav2 closed loop with synthetic kinematics, synthetic TF/odometry and static obstacle map; excludes SLAM and robot hardware'}
try:
 a=ActionClient(n,NavigateToPose,'/navigate_to_pose');end=time.monotonic()+35
 while not a.server_is_ready() and time.monotonic()<end and p.poll() is None:rclpy.spin_once(n,timeout_sec=.02)
 if not a.server_is_ready():raise RuntimeError('Nav2 action unavailable')
 for name in ['bt_navigator','controller_server','velocity_smoother']:
  client=n.create_client(GetState,'/'+name+'/get_state');deadline=time.monotonic()+30
  while time.monotonic()<deadline:
   if client.service_is_ready() and spin_until(client.call_async(GetState.Request()),5).current_state.id==3:break
   until=time.monotonic()+.2
   while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
  else:raise TimeoutError(name+' not active')
 g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=n.get_clock().now().to_msg();g.pose.pose.position.x=3.;g.pose.pose.position.y=1.;g.pose.pose.orientation.w=1.
 handle=spin_until(a.send_goal_async(g),5)
 if not handle.accepted:raise RuntimeError('goal rejected')
 response=spin_until(handle.get_result_async(),65)
 result.update(action_status=response.status,final_pose=[x,y,yaw],goal_distance_m=math.hypot(x-3,y-1),command_messages=len(commands),max_abs_vx=max(abs(c[0]) for c in commands),max_abs_wz=max(abs(c[2]) for c in commands))
except Exception as e:result['error']=repr(e)
finally:
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 result.update(final_pose=[x,y,yaw],goal_distance_m=math.hypot(x-3,y-1),command_messages=len(commands),max_abs_vx=max([abs(c[0]) for c in commands] or [0]),max_abs_wz=max([abs(c[2]) for c in commands] or [0]));log.close();n.destroy_node();rclpy.shutdown();np.savetxt(out/'trajectory.csv',trajectory,delimiter=',',header='monotonic,x,y,yaw',comments='');(out/'result.json').write_text(json.dumps(result,indent=2));print(result)

if "error" in result or result.get("action_status") != 4:
 raise SystemExit(1)
