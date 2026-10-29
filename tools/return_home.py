"""Return to configured home once; never command the gripper."""
import fcntl,json,os,signal,sys,time,socket,struct,subprocess
from pathlib import Path
from datetime import datetime
from dataclasses import replace
from threading import Event,Thread
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from grasp_executor.config import load_config
from grasp_executor.grasp import build_return_home_pose
from grasp_executor.cartesian import pose_error
from grasp_executor.moveit_ros import MoveItArm,JOINTS

root=Path(__file__).resolve().parents[1]
config_path=root/'src/grasp_executor/config/place_zones.json'
old_bytes=config_path.read_bytes();zones=json.loads(old_bytes)
if zones.get('schema_version') != 2 or zones.get('place_ready_frame') != 'base_link' or zones.get('place_ready_tool_frame') != 'grasp_center':
 raise ValueError('invalid_home_config_frames_or_schema')
suffix=datetime.now().strftime('%Y%m%d_%H%M%S');output=root/'debug_output'/'return_home'/f'{suffix}_{os.getpid()}.json'
output.parent.mkdir(parents=True,exist_ok=True)
config=replace(load_config(root/'src/grasp_executor/config/place_config.json'),plan_only=False,
 approach_speed=.0675,angular_speed=.45,velocity_scaling=.225,acceleration_scaling=.05,
 max_joint_velocity=.675,max_joint_acceleration=.10,motion_timeout=180.,planning_timeout=120.,
 return_home_joint_positions=zones['return_home']['joint_positions'],
 return_home_position=zones['return_home']['position'],return_home_orientation=zones['return_home']['orientation'])
fd=os.open(f'/tmp/robot-grasp-{os.getuid()}-domain-{int(os.environ.get("ROS_DOMAIN_ID","0"))}.place-task.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
# Prepare controllers while holding the same task lock as the grasp pipeline.
subprocess.run([str(root/'run_grasp_position_control.sh')],check=True,timeout=90)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('return_to_saved_home');cancel=Event()
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:cancel.set())
arm=MoveItArm(node,config,cancel);executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
thread=Thread(target=executor.spin,daemon=True);thread.start()
record={'started_at':datetime.now().astimezone().isoformat(),'mode':'execute','motion_sent':False,'gripper_commands_sent':False}
try:
 arm.check_ready(20.)
 deadline=time.monotonic()+120.
 current=arm._state(stationary=True);arm._valid(current,deadline)
 # Read-only runtime check: program-running Bool can remain true while paused.
 rtde_deadline=time.monotonic()+10.
 with socket.create_connection((os.environ.get('ROBOT_IP','192.168.10.102'),30004),timeout=3) as sock:
  def read_exact(n):
   b=b''
   while len(b)<n:
    if cancel.is_set():raise RuntimeError('cancelled')
    if time.monotonic()>rtde_deadline:raise TimeoutError('rtde_state_timeout')
    chunk=sock.recv(n-len(b))
    if not chunk:raise RuntimeError('rtde_closed')
    b+=chunk
   return b
  def response(wanted):
   while True:
    if cancel.is_set():raise RuntimeError('cancelled')
    if time.monotonic()>rtde_deadline:raise TimeoutError('rtde_state_timeout')
    size,kind=struct.unpack('!HB',read_exact(3));payload=read_exact(size-3)
    if kind==wanted:return payload
  def send(kind,payload=b''):sock.sendall(struct.pack('!HB',len(payload)+3,kind)+payload)
  send(86,struct.pack('!H',2));assert response(86)==b'\x01'
  send(79,struct.pack('!d',5.)+b'runtime_state,speed_scaling,target_speed_fraction,safety_mode');recipe=response(79)
  assert recipe[1:]==b'UINT32,DOUBLE,DOUBLE,INT32'
  send(83);assert response(83)==b'\x01'
  payload=response(85);runtime,scaling,fraction,safety=struct.unpack('!Iddi',payload[1:])
  record['runtime_before_execution']={'runtime_state':runtime,'speed_scaling':scaling,'target_speed_fraction':fraction,'safety_mode':safety}
  if runtime!=2 or safety!=1 or scaling<=0 or fraction<=0:raise RuntimeError('robot_program_paused_or_speed_zero_or_safety_not_normal')
 goal=build_return_home_pose(config)
 segment=arm._return_home_plan(current,deadline)
 if config_path.read_bytes()!=old_bytes:raise RuntimeError('home_config_changed_during_planning')
 arm.plans=[segment];arm.next_segment=0
 joint=segment.trajectory.joint_trajectory
 record['planned_duration_s']=joint.points[-1].time_from_start.sec+joint.points[-1].time_from_start.nanosec/1e9
 record['target_home']=zones['return_home']
 print(json.dumps({'stage':'executing_home','planned_duration_s':record['planned_duration_s']},ensure_ascii=False),flush=True)
 if config_path.read_bytes()!=old_bytes:raise RuntimeError('home_config_changed_before_execution')
 record['motion_sent']=True
 arm.move_to_pose(goal,config.motion_timeout)
 stop=time.monotonic()+config.motion_timeout
 while not arm.is_pose_reached(.1):
  if time.monotonic()>stop:raise TimeoutError('home_motion_timeout')
  if cancel.wait(.05):raise RuntimeError('cancelled')
 arm.finish()
 state=arm._state(stationary=True);measured=arm.read_pose()
 record.update(measured_joint_positions=dict(zip(state.joint_state.name,state.joint_state.position)),max_joint_error_rad=arm._joint_error(state,JOINTS,[config.return_home_joint_positions[j] for j in JOINTS]),tool_position_error_m=pose_error(measured,goal)[0],tool_orientation_error_rad=pose_error(measured,goal)[1])
 record['passed']=True
except Exception as exc:
 record['passed']=False;record['error']=str(exc)
 if arm.commanded:
  try:arm.stop_motion(config.stop_timeout)
  except Exception as stop_exc:record['stop_error']=str(stop_exc)
finally:
 record['finished_at']=datetime.now().astimezone().isoformat()
 with output.open('x') as f:json.dump(record,f,ensure_ascii=False,indent=2);f.write('\n')
 print(json.dumps({'record':str(output),**record},ensure_ascii=False,indent=2),flush=True)
 arm.close();executor.shutdown();thread.join(timeout=3.);node.destroy_node();rclpy.shutdown();os.close(fd)
sys.exit(0 if record['passed'] else 1)
