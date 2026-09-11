#!/usr/bin/env python3
"""Stationary, disarmed hardware TF test. Owns only processes it starts."""
import json, math, os, signal, socket, subprocess, threading, time
from pathlib import Path
os.environ['ROS_MASTER_URI']='http://127.0.0.1:11311'
ROOT=Path('/home/nrc/catkin_ws')
LOG=ROOT/'logs/tf-stage-refactor'/time.strftime('live-%Y%m%d-%H%M%S')
LOG.mkdir(parents=True)
children=[]; handles=[]; checks=[]; report={'checks':checks,'log':str(LOG)}
def launch(name,args):
 h=(LOG/(name+'.log')).open('w');handles.append(h)
 p=subprocess.Popen(args,stdout=h,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
 children.append(p);return p

def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(10)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM);p.wait(5)

def check(name,passed,detail=None):
 checks.append(dict(name=name,passed=bool(passed),detail=detail))
 print(json.dumps(checks[-1]),flush=True)
 if not passed:raise RuntimeError(name)

def wait(predicate,seconds=20):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  if predicate():return True
  time.sleep(.05)
 return False

sock=socket.socket();busy=sock.connect_ex(('127.0.0.1',11311))==0;sock.close()
check('dedicated real master is unused',not busy)
try:
 import rospy,tf2_ros
 from tf2_msgs.msg import TFMessage
 from std_msgs.msg import Bool
 from nav_msgs.msg import Odometry
 from mavros_msgs.msg import State
 from fast_lio_sam.srv import save_map
 base=launch('base',['roslaunch','ducted_bringup','base_system.launch'])
 def master_up():
  try:return rospy.get_master().getPid()[0]==1
  except Exception:return False
 check('master starts',wait(master_up))
 rospy.init_node('tf_stage_probe',disable_signals=True)
 latest={};hist={};lock=threading.RLock();edges=[];private=[]
 def receive(topic):
  def callback(msg):
   with lock:
    latest[topic]=msg
    if hasattr(msg,'header'):
     hist.setdefault(topic,{})[msg.header.stamp.to_nsec()]=msg
     while len(hist[topic])>200:del hist[topic][min(hist[topic])]
  return callback
 for topic,kind in [('/mavros/state',State),('/ducted/system/ready',Bool),('/ducted/localization/odom',Odometry),('/ducted/localization/body_odom',Odometry),('/ducted/relocalization/global_odom',Odometry),('/ducted/localization/map_ready',Bool)]:
  rospy.Subscriber(topic,kind,receive(topic),queue_size=100)
 def tf_callback(message):
  with lock:
   for t in message.transforms:edges.append((t.header.frame_id,t.child_frame_id,message._connection_header.get('callerid',''),t.header.stamp.to_nsec()))
 rospy.Subscriber('/tf',TFMessage,tf_callback,queue_size=100)
 rospy.Subscriber('/tf_static',TFMessage,tf_callback,queue_size=10)
 rospy.Subscriber('/mavros/internal_tf_static',TFMessage,lambda m:private.extend((t.header.frame_id,t.child_frame_id) for t in m.transforms),queue_size=10)
 def disarmed():
  s=latest.get('/mavros/state');return s is not None and s.connected and not s.armed
 check('base ready and disarmed',wait(lambda:disarmed() and getattr(latest.get('/ducted/system/ready'),'data',False),30))
 time.sleep(2)
 check('base public TF has no localization roots',not edges,list(edges))
 check('MAVROS internal conversion TF retained',('odom','odom_ned') in private and ('base_link','base_link_frd') in private,private)

 def inspect_chain(expected,label):
  pairs={(a,b) for a,b,_,_ in edges}
  check(label+' exact public edges',pairs==set(expected),sorted(pairs))
  owners={}
  for a,b,owner,stamp in edges:owners.setdefault((a,b),set()).add(owner)
  check(label+' one publisher per edge',all(len(v)==1 for v in owners.values()),{a+' -> '+b:sorted(v) for (a,b),v in owners.items()})
  check(label+' remains disarmed',disarmed())

 mapping=launch('mapping',['roslaunch','ducted_bringup','mapping.launch'])
 check('mapping local odometry arrives',wait(lambda:'/ducted/localization/body_odom' in latest,30))
 buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
 time.sleep(10)
 inspect_chain([('odom','camera_init'),('camera_init','body'),('body','base_link')],'mapping')
 errors=[]
 with lock:samples=list(hist['/ducted/localization/body_odom'].values())
 for msg in samples:
  try:
   t=buffer.lookup_transform('odom','base_link',msg.header.stamp,rospy.Duration(0))
   p,q=msg.pose.pose.position,msg.pose.pose.orientation
   tr,qr=t.transform.translation,t.transform.rotation
   pe=math.sqrt((p.x-tr.x)**2+(p.y-tr.y)**2+(p.z-tr.z)**2)
   qe=min(math.sqrt(sum((a-b)**2 for a,b in zip((q.x,q.y,q.z,q.w),(qr.x,qr.y,qr.z,qr.w)))),math.sqrt(sum((a+b)**2 for a,b in zip((q.x,q.y,q.z,q.w),(qr.x,qr.y,qr.z,qr.w)))))
   errors.append((pe,qe))
  except Exception:pass
 check('mapping TF numerically matches body odometry',len(errors)>20 and max(x[0] for x in errors)<1e-5 and max(x[1] for x in errors)<1e-5,dict(pairs=len(errors),maximum=max(errors) if errors else None))
 result={};done=threading.Event()
 def save():
  try:result['success']=rospy.ServiceProxy('/ducted/mapping/save_map',save_map)(.2,str(LOG/'map')).success
  except Exception as e:result['error']=str(e)
  finally:done.set()
 threading.Thread(target=save,daemon=True).start()
 check('current-scene map saved',done.wait(60) and result.get('success'),result)
 stop(mapping);listener.unregister()
 with lock:edges.clear();hist.clear();latest.pop('/ducted/localization/body_odom',None)
 relocal=launch('relocalization',['roslaunch','ducted_bringup','relocalization.launch','map_file:='+str(LOG/'map/GlobalMap.pcd')])
 check('relocalization correction becomes ready',wait(lambda:getattr(latest.get('/ducted/localization/map_ready'),'data',False),45))
 buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
 time.sleep(8)
 inspect_chain([('map','odom'),('odom','camera_init'),('camera_init','body'),('body','base_link')],'relocalization')
 from ducted_bringup.fastlio_odometry import adapt_odometry_values,quaternion_from_rpy
 errors=[]
 with lock:
  samples=list(hist['/ducted/relocalization/global_odom'].values())
  corrected_stamps={stamp for a,b,_,stamp in edges if (a,b)==('map','odom')}
 report['global_source_samples']=len(samples)
 report['correction_stamps']=len(corrected_stamps)
 report['exact_global_samples']=sum(m.header.stamp.to_nsec() in corrected_stamps for m in samples)
 for msg in samples:
  if msg.header.stamp.to_nsec() not in corrected_stamps:continue
  try:
   t=buffer.lookup_transform('map','base_link',msg.header.stamp,rospy.Duration(0))
   p,q=msg.pose.pose.position,msg.pose.pose.orientation
   target=adapt_odometry_values((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),(0,0,0),(0,0,0),(.13,0,0),quaternion_from_rpy(.03,.4567,0))
   tr,qr=t.transform.translation,t.transform.rotation
   pe=math.sqrt(sum((a-b)**2 for a,b in zip(target.position,(tr.x,tr.y,tr.z))))
   qe=min(math.sqrt(sum((a-b)**2 for a,b in zip(target.child_orientation,(qr.x,qr.y,qr.z,qr.w)))),math.sqrt(sum((a+b)**2 for a,b in zip(target.child_orientation,(qr.x,qr.y,qr.z,qr.w)))))
   errors.append((pe,qe))
  except Exception:pass
 check('global correction TF composes to global body pose',len(errors)>20 and max(x[0] for x in errors)<1e-5 and max(x[1] for x in errors)<1e-5,dict(pairs=len(errors),maximum=max(errors) if errors else None))
 pubs=dict(rospy.get_master().getSystemState()[2][0])
 command_topics={'/mavros/setpoint_raw/local','/mavros/setpoint_raw/global','/mavros/setpoint_raw/attitude','/mavros/setpoint_position/local','/mavros/setpoint_velocity/cmd_vel','/mavros/setpoint_velocity/cmd_vel_unstamped','/mavros/setpoint_attitude/attitude','/mavros/setpoint_attitude/thrust','/mavros/setpoint_trajectory/local'}
 forbidden={k:v for k,v in pubs.items() if k in command_topics and v}
 check('no flight setpoint publishers',not forbidden,forbidden)
 report['passed']=True
except Exception as e:
 report.update(passed=False,error=str(e))
finally:
 for p in reversed(children):
  try:stop(p)
  except Exception as e:report.setdefault('cleanup_errors',[]).append(str(e))
 for h in handles:h.close()
 (LOG/'result.json').write_text(json.dumps(report,indent=2))
 print(json.dumps(report),flush=True)
raise SystemExit(0 if report.get('passed') else 1)
