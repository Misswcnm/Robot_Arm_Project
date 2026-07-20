import math, os, threading, time, traceback
import numpy as np
from scipy.spatial.transform import Rotation
from .store import atomic_json, digest, load_json, now

MOTION_ACTIONS=set(['vision_icp_record_a','vision_icp_record_b','vision_icp_align','vision_icp_align_and_move_b','apriltag_validate','apriltag_pick'])
TERMINAL=set(['succeeded','failed','timeout','cancelled'])

class VisionExecutor(object):
 """Thread-safe task state machine. This module never imports rospy."""
 def __init__(self, node, cfg, robot_factory=None, servo_factory=None, tag_factory=None):
  self.node,self.cfg=node,cfg; self.lock=threading.RLock(); self.motion_lock=threading.Lock(); self.tasks={}; self.cancelled=set(); self.owner=None; self.execution_enabled=False
  self.root=os.path.expanduser(cfg['data_dir']); self.robot_factory=robot_factory; self.servo_factory=servo_factory; self.tag_factory=tag_factory; self.robot=None; self.servo=None; self.tag=None
  if not os.path.isdir(self.root): os.makedirs(self.root)
 def response(self, rid, action, status='accepted', message='', code='', **kw):
  out={'request_id':rid,'action':action,'backend':'vision','status':status,'progress':0.0,'message':message,'error_code':code,'metrics':{},'artifacts':[],'started_at':now(),'finished_at':''}; out.update(kw); return out
 def submit(self, req):
  rid,action=req['request_id'],req['action']
  with self.lock:
   if rid in self.tasks:return self.tasks[rid].copy()
   if action in ('health','arm_status','task_status','task_cancel','execution_enable','execution_disable'):
    result=self._immediate(req); self.tasks[rid]=result; return result.copy()
   if action in MOTION_ACTIONS and self.owner:
    return self.response(rid,action,'failed','another mechanical-arm backend owns the resource','busy',metrics={'owner':self.owner})
   record=self.response(rid,action,'accepted','queued'); self.tasks[rid]=record
   if action in MOTION_ACTIONS:self.owner=rid
   threading.Thread(target=self._run,args=(req,),daemon=True).start(); return record.copy()
 def _immediate(self, req):
  a,rid=req['action'],req['request_id']
  if a=='health': return self.response(rid,a,'succeeded','vision RPC healthy',metrics={'execution_enabled':self.execution_enabled,'owner':self.owner})
  if a=='task_status':
   target=req['params'].get('request_id'); return self.tasks.get(target,self.response(rid,a,'failed','unknown request_id','not_found')).copy()
  if a=='task_cancel':
   target=req['params'].get('request_id'); task=self.tasks.get(target)
   if not task:return self.response(rid,a,'failed','unknown request_id','not_found')
   self.cancelled.add(target); return self.response(rid,a,'succeeded','cancel requested at next safety checkpoint',metrics={'target_request_id':target})
  if a=='execution_enable': self.execution_enabled=True; return self.response(rid,a,'succeeded','real motion explicitly enabled')
  if a=='execution_disable': self.execution_enabled=False; return self.response(rid,a,'succeeded','real motion disabled')
  return self._status(rid,a)
 def _status(self,rid,a):
  metrics={'execution_enabled':self.execution_enabled,'owner':self.owner,'lock_held':bool(self.owner)}
  try:
   robot=self._robot(); metrics.update({'robot_mode':robot.get_mode(),'pose':robot.get_tool(warn=False)})
  except Exception as e: metrics['robot_error']=str(e)
  return self.response(rid,a,'succeeded','status read',metrics=metrics)
 def _run(self,req):
  rid,a=req['request_id'],req['action']; started=time.time()
  with self.lock:self.tasks[rid].update(status='running',progress=.05,message='running',started_at=now())
  try:
   result=self._dispatch(req)
   if time.time()-started>float(req['timeout_sec']): raise RuntimeError('task timeout')
   if rid in self.cancelled: result=self.response(rid,a,'cancelled','cancelled at safety checkpoint')
   result['finished_at']=now(); result['started_at']=self.tasks[rid]['started_at']
  except Exception as e:
   result=self.response(rid,a,'failed',str(e),'execution_error',metrics={'traceback':traceback.format_exc()[-2000:]}); result['finished_at']=now(); result['started_at']=self.tasks[rid]['started_at']
  finally:
   with self.lock:
    self.tasks[rid]=result
    if self.owner==rid:self.owner=None
 def _robot(self):
  if self.robot is None:
   if self.robot_factory:self.robot=self.robot_factory(self.node,int(self.cfg['robot_speed']))
   else:
    from icp_servoing.robot import CR5Robot; self.robot=CR5Robot(self.node,speed=int(self.cfg['robot_speed']))
   if not self.robot.init(): raise RuntimeError('CR5 initialization failed')
  return self.robot
 def _servo(self):
  if self.servo is None:
   from icp_servoing.handeye import load_X
   from icp_servoing.servoing import VisualServo
   self.servo=(self.servo_factory or VisualServo)(self._robot(),load_X(os.path.expanduser(self.cfg['handeye_path'])))
   self.servo._pc_node=self.node
  return self.servo
 def _tag(self):
  if self.tag is None:
   if self.tag_factory:self.tag=self.tag_factory(self.node)
   else:
    from apriltag_pick.pick_node import AprilTagPickNode; self.tag=AprilTagPickNode()
  return self.tag
 def _safety(self, req, target=None):
  if req.get('dry_run'):return
  if not self.execution_enabled:raise RuntimeError('execution_disabled')
  robot=self._robot(); mode=robot.get_mode()
  if getattr(robot,'dragging',False) or mode==getattr(robot,'MODE_BACKDRIVE',6):raise RuntimeError('robot_in_drag_mode')
  if mode not in (getattr(robot,'MODE_ENABLED',5),5):raise RuntimeError('robot_not_enabled')
  if target is not None:
   cur=robot.get_tool(warn=False)
   if cur is None:raise RuntimeError('GetPose failed')
   xyz=np.asarray(target[:3,3]); lo=np.asarray(self.cfg['workspace_min_xyz_mm']); hi=np.asarray(self.cfg['workspace_max_xyz_mm'])
   if np.any(xyz<lo) or np.any(xyz>hi):raise RuntimeError('target_outside_workspace')
   dp=np.linalg.norm(xyz-np.asarray(cur[:3])); dr=np.degrees(np.linalg.norm(Rotation.from_matrix(target[:3,:3]).as_rotvec()))
   if dp>float(self.cfg['max_move_translation_mm']) or dr>float(self.cfg['max_move_rotation_deg']):raise RuntimeError('target_delta_exceeds_limit')
 def _record(self, name, payload):
  payload.update(schema_version=1,record_id=name+'-'+str(int(time.time()*1000)),created_at=now(),robot_model='CR5',user_id=0,tool_id=0)
  path=os.path.join(self.root,name+'.json'); atomic_json(path,payload); return path
 def _dispatch(self,req):
  a,rid=req['action'],req['request_id']; dry=req.get('dry_run',False)
  if a=='vision_icp_record_a':
   self._safety(req); servo=self._servo(); ok=True if dry else servo.record_template(int(req['params'].get('frames',self.cfg['icp_frames'])))
   if not ok:raise RuntimeError('ICP template recording failed')
   T=np.eye(4) if dry else servo._T_base_tool_ref
   path=self._record('icp_a',{'request_id':rid,'dry_run':dry,'T_base_flange_A':T.tolist(),'template_frames':int(req['params'].get('frames',self.cfg['icp_frames'])),'handeye_path':self.cfg['handeye_path']})
   return self.response(rid,a,'succeeded','ICP reference A recorded',artifacts=[path])
  if a=='vision_icp_record_b':
   a_data=load_json(os.path.join(self.root,'icp_a.json'))
   if not a_data:raise RuntimeError('ICP reference A missing')
   robot=self._robot(); self._safety(req)
   # Caller explicitly confirms manual positioning; no RPC thread waits for an operator.
   if not req['params'].get('manual_positioned',False):raise RuntimeError('manual_positioned=true required after drag positioning and safe exit')
   pose=[0,0,0,0,0,0] if dry else robot.get_tool(warn=False)
   if pose is None:raise RuntimeError('GetPose failed')
   T=np.eye(4); T[:3,3]=pose[:3]; T[:3,:3]=Rotation.from_euler('xyz',pose[3:],degrees=True).as_matrix(); delta=np.linalg.inv(np.asarray(a_data['T_base_flange_A']))@T
   path=self._record('icp_b',{'request_id':rid,'dry_run':dry,'T_base_flange_B':T.tolist(),'T_A_to_B':delta.tolist(),'a_record_id':a_data['record_id']})
   return self.response(rid,a,'succeeded','ICP relative B recorded',artifacts=[path])
  if a in ('vision_icp_align','vision_icp_align_and_move_b'):
   servo=self._servo(); self._safety(req); ok=True if dry else servo.align(int(req['params'].get('max_iters',self.cfg['icp_max_iters'])))
   if not ok:raise RuntimeError('ICP did not converge; B motion prohibited')
   metrics={'converged':True}
   if a.endswith('move_b'):
    b=load_json(os.path.join(self.root,'icp_b.json'))
    if not b:raise RuntimeError('ICP B missing')
    cur=servo._get_tool_matrix(); target=cur@np.asarray(b['T_A_to_B']); self._safety(req,target)
    if not dry and not self._robot().movj_pose(target,'ICP A-to-B'):raise RuntimeError('relative B move failed')
    metrics['T_target']=target.tolist()
   return self.response(rid,a,'succeeded','ICP aligned',metrics=metrics)
  if a=='apriltag_locate':
   target=self._tag().locate(); tag=self._tag(); path=self._record('apriltag_cache',{'request_id':rid,'dry_run':dry,'cached_at':now(),'target':target.tolist(),'localization':self._jsonify(tag.last_localization),'ttl_sec':float(self.cfg['apriltag_cache_ttl_sec']),'handeye_path':self.cfg['handeye_path'],'handeye_sha256':digest(self.cfg['handeye_path'])})
   return self.response(rid,a,'succeeded','AprilTag target cached',artifacts=[path])
  if a=='apriltag_validate':
   self._safety(req); value=self._tag().record_manual_tag_center(); return self.response(rid,a,'succeeded','AprilTag manual validation recorded',metrics={'error_mm':self._jsonify(value)})
  if a=='apriltag_pick':
   cache=load_json(os.path.join(self.root,'apriltag_cache.json'))
   if not cache:raise RuntimeError('AprilTag cache missing')
   if time.time()-os.path.getmtime(os.path.join(self.root,'apriltag_cache.json'))>float(self.cfg['apriltag_cache_ttl_sec']):raise RuntimeError('AprilTag cache expired')
   if cache.get('handeye_sha256')!=digest(self.cfg['handeye_path']):raise RuntimeError('calibration changed since locate')
   self._safety(req,np.asarray(cache['target']));
   if not dry:self._tag().pick()
   return self.response(rid,a,'succeeded','AprilTag cached pick complete')
  raise RuntimeError('unsupported action')
 def _jsonify(self,v):
  if isinstance(v,np.ndarray):return v.tolist()
  if isinstance(v,dict):return dict((k,self._jsonify(x)) for k,x in v.items())
  if isinstance(v,(list,tuple)):return [self._jsonify(x) for x in v]
  return v
