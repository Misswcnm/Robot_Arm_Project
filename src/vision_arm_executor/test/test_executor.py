import os
import json
import tempfile
import unittest

import numpy as np
from apriltag_pick.pick_node import TagDetectionTimeout

from vision_arm_executor.backends import (
    AprilTagIdentity, AprilTagLocation, IcpAlignment, IcpReference,
    RobotState)
from vision_arm_executor.executor import VisionExecutor

class FakeRobot(object):
 MODE_ENABLED=5; MODE_BACKDRIVE=6
 def __init__(self,*a):
  self.dragging=False; self.init_calls=0; self.calls=[]; self.mode=5
 def init(self): self.init_calls+=1; return True
 def get_mode(self): return self.mode
 def get_tool(self,**kw): return [0,0,100,0,0,0]
 def reset_robot(self): self.calls.append('reset'); self.mode=4; return True
 def enable_robot(self): self.calls.append('enable'); self.mode=5; return True
 def disable_robot(self): self.calls.append('disable'); self.mode=4; return True
 def clear_error(self): self.calls.append('clear_error'); return True
 def start_drag(self):
  self.calls.append('start_drag'); self.dragging=True; self.mode=6; return True
 def stop_drag(self):
  self.calls.append('stop_drag'); self.dragging=False; self.mode=4; return True


class FakeTeachingBackend(object):
 def __init__(self, unused_node, unused_cfg, **unused):
  self.mode=5; self.dragging=False; self.align_calls=[]; self.move_calls=[]
  self.events=[]; self.execution_events=[]; self.stable_calls=[]
 def robot_state(self):
  return RobotState(self.mode,[0,0,100,0,0,0],self.dragging,5,6)
 def start_drag(self):
  self.events.append('start_drag')
  self.mode=6; self.dragging=True; return self.robot_state()
 def stop_drag_and_enable(self):
  self.events.append('stop_drag_and_enable')
  self.mode=5; self.dragging=False; return self.robot_state()
 def record_reference_teaching(self,frames,path):
  self.events.append('record_a')
  os.makedirs(os.path.dirname(path),exist_ok=True)
  with open(path,'wb') as stream: stream.write(b'icp-template')
  return IcpReference(np.eye(4),path,frames)
 def flange_transform_teaching(self):
  self.events.append('read_b')
  value=np.eye(4); value[0,3]=25.0; return value
 def flange_pose_teaching(self):
  self.events.append('read_observation')
  return [120.0,220.0,320.0,10.0,20.0,30.0]
 def flange_joints_teaching(self):
  self.events.append('read_joints')
  return [1,2,3,4,5,6]
 def wait_teaching_stable(self,min_duration_sec=1.0,timeout_sec=8.0):
  self.events.append('wait_stable')
  self.stable_calls.append((min_duration_sec,timeout_sec)); return True
 def flange_transform(self):
  value=np.eye(4); value[0,3]=10.0; return value
 def initialize_robot(self):
  return True
 def align(self,path,max_iters,should_stop,progress_callback,motion_guard):
  self.execution_events.append('align')
  self.align_calls.append(path)
  progress_callback(1,max_iters,{'reason':'converged'})
  return IcpAlignment(True,{'reason':'converged'})
 def move_to_reference(self,transform,label,motion_guard,joints=None):
  target=np.asarray(transform,dtype=float)
  motion_guard(target)
  self.execution_events.append('recover_a')
  return target
 def move_relative(self,delta,label,motion_guard,**kwargs):
  base=np.asarray(kwargs.get('base_transform',np.eye(4)),dtype=float)
  target=base @ np.asarray(delta,dtype=float)
  motion_guard(target)
  self.execution_events.append('move_b')
  self.move_calls.append(label)
  return target
 def move_joints(self,joints,label,motion_guard=None,target_transform=None):
  if motion_guard is not None and target_transform is not None:
   motion_guard(target_transform)
  self.execution_events.append('move_joints')
  return list(joints)
 def reset_robot(self):
  self.execution_events.append('reset')
  return True


class FakeFailedAlignmentBackend(FakeTeachingBackend):
 def align(self,path,max_iters,should_stop,progress_callback,motion_guard):
  self.execution_events.append('align')
  self.align_calls.append(path)
  raise RuntimeError('simulated ICP failure')


class FakeAprilTagBackend(object):
 def __init__(self, unused_node, **unused):
  self.locate_calls=0; self.pick_calls=0; self.motion_disabled=True
  self.observation_calls=[]; self.last_offset=None; self.tag_ids=[]
  self.detection_timeouts=[]; self.move_labels=[]; self.move_targets=[]
  self.target=np.eye(4); self.target[0,3]=100.0
  self.base_flange=np.eye(4); self.base_flange[0,3]=25.0
 def identity(self,tag_id=None):
  selected=0 if tag_id is None else int(tag_id)
  return AprilTagIdentity(
      selected,'tag36h11:%d' % selected,'/dev/null','/dev/null')
 def robot_state(self):
  return RobotState(5,[100,0,0,0,0,0],False,5,6)
 def disable_motion(self):
  self.motion_disabled=True
 def locate(self,tag_id=None,tag_offset_xyz_mm=None,
            wait_for_new_detection=True,detection_timeout_sec=5.0):
  self.locate_calls+=1
  self.wait_for_new_detection=wait_for_new_detection
  self.detection_timeout_sec=detection_timeout_sec
  self.detection_timeouts.append(float(detection_timeout_sec))
  self.tag_ids.append(0 if tag_id is None else int(tag_id))
  self.last_offset=list(tag_offset_xyz_mm or [0,0,0])
  self.target[:3,3]=np.asarray(self.last_offset,dtype=float)
  return AprilTagLocation(
      self.target.copy(),
      {'frames':10,'spread_mm':0.2,
       'flange_xyzrpy':[100,0,0,0,0,0],
       'base_flange_at_detection':self.base_flange.copy(),
       'base_tag':np.eye(4)},
      self.identity(tag_id))
 def move_to_observation(self,pose,safety_check,joints=None):
  target=np.eye(4); target[:3,3]=np.asarray(pose[:3],dtype=float)
  safety_check('before_observation',target)
  self.observation_calls.append(list(pose))
  return target
 def move_flange_target(self,target,safety_check,label='work'):
  safety_check('before_work',target)
  self.move_labels.append(label)
  self.move_targets.append(np.asarray(target,dtype=float).copy())
  self.pick_calls+=1
  return np.asarray(target,dtype=float)
 def lateral_observation_target(self,target,offset_mm):
  shifted=np.asarray(target,dtype=float).copy()
  shifted[0,3]+=float(offset_mm)
  return shifted
 def prepare_pick(self,unused_localization):
  self.motion_disabled=False
 def localization_drift(self,unused_localization):
  return 0.0,0.0
 def pick(self,safety_check):
  safety_check('pick',self.target)
  self.pick_calls+=1
  self.motion_disabled=True


class FakeSearchAprilTagBackend(FakeAprilTagBackend):
 failures_before_detection=2
 def locate(self,*args,**kwargs):
  result=super().locate(*args,**kwargs)
  if self.locate_calls<=self.failures_before_detection:
   raise TagDetectionTimeout('no tag in this search position')
  return result


class FakeMissingAprilTagBackend(FakeSearchAprilTagBackend):
 failures_before_detection=3


class ExecutorTests(unittest.TestCase):
 def make(self, robot=None, icp_backend_factory=None,
          apriltag_backend_factory=None, cfg_overrides=None):
  robot=robot or FakeRobot()
  cfg={'data_dir':tempfile.mkdtemp(),'robot_speed':15,'workspace_min_xyz_mm':[-1e3]*3,'workspace_max_xyz_mm':[1e3]*3,'max_move_translation_mm':300,'max_move_rotation_deg':45,'icp_frames':2,'icp_max_iters':1,'apriltag_cache_ttl_sec':1,'apriltag_detection_timeout_sec':5.0,'apriltag_search_detection_timeout_sec':2.0,'apriltag_post_observation_settle_sec':0.0,'apriltag_search_lateral_mm':20.0,'apriltag_max_robot_drift_mm':10,'apriltag_max_robot_drift_deg':3,'handeye_path':'/dev/null','tcp_calibration_path':'/dev/null'}
  cfg.update(cfg_overrides or {})
  executor=VisionExecutor(None,cfg,robot_factory=lambda *a:robot,
                          icp_backend_factory=icp_backend_factory,
                          apriltag_backend_factory=apriltag_backend_factory)
  executor.fake_robot=robot
  return executor
 def save_tag_group(self,e,task_command='1',tag_id=0,observe_x=100.0):
  reference={
      'record_id':'tag-ref-'+task_command,'tag_id':tag_id,
      'observe_flange_pose':[observe_x,0,100,0,0,0],
      'observe_joints':[1,2,3,4,5,6],
      'T_base_tag_ref':np.eye(4).tolist()}
  target=np.eye(4); target[0,3]=25.0
  work={
      'record_id':'tag-work-'+task_command,'work_id':'W1','index':1,
      'command':2,'T_tag_to_flange_work':target.tolist()}
  e.station_store.save_group(
      'map-1','station-1',task_command,0,reference,[work])
 def wait(self,e,request_id):
  import time
  for unused in range(100):
   if e.tasks[request_id]['status'] not in ('accepted','running'):
    return e.tasks[request_id]
   time.sleep(.01)
  self.fail('task did not finish: '+request_id)
 def test_enable_does_not_survive_new_executor(self):
  e=self.make(); self.assertFalse(e.execution_enabled); self.assertEqual('succeeded',e.submit({'request_id':'x','action':'execution_enable','params':{},'timeout_sec':1,'dry_run':False})['status']); self.assertTrue(e.execution_enabled); self.assertFalse(self.make().execution_enabled)
 def test_request_id_idempotent(self):
  e=self.make(); req={'request_id':'same','action':'health','params':{},'timeout_sec':1,'dry_run':False}; self.assertEqual(e.submit(req),e.submit(req))
 def test_arm_status_does_not_initialize_or_enable_robot(self):
  e=self.make(); req={'request_id':'status','action':'arm_status','params':{},'timeout_sec':1,'dry_run':False}
  result=e.submit(req)
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(0,e.fake_robot.init_calls)
 def test_arm_status_fails_when_feedback_is_unavailable(self):
  robot=FakeRobot(); robot.get_mode=lambda:None; robot.get_tool=lambda **kw:None; e=self.make(robot)
  req={'request_id':'missing-status','action':'arm_status','params':{},'timeout_sec':1,'dry_run':False}
  result=e.submit(req)
  self.assertEqual('failed',result['status'])
  self.assertEqual('robot_unavailable',result['error_code'])
 def test_cancel_unknown(self):
  e=self.make(); r=e.submit({'request_id':'c','action':'task_cancel','params':{'request_id':'none'},'timeout_sec':1,'dry_run':False}); self.assertEqual('failed',r['status'])
 def test_station_delete_without_command_removes_unique_pose(self):
  e=self.make()
  self.save_tag_group(e,'1')
  self.save_tag_group(e,'2')
  result=e.submit({
      'request_id':'delete-station','action':'vision_station_delete',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':1,'dry_run':False})
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['1','2'],result['metrics']['deleted_commands'])
  points,unused_path=e.station_store.load('map-1','station-1')
  self.assertEqual([],points)
 def test_station_delete_without_command_cancels_matching_active_teaching(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  for request_id,command,extra in [
      ('delete-start','start',{
          'mapid':'map-1','poseid':'station-1','point_type':1}),
      ('delete-ref',1,{})]:
   params={'command':command}; params.update(extra)
   e.submit({'request_id':request_id,'action':'vision_point_teach',
             'params':params,'timeout_sec':2,'dry_run':False})
   self.assertEqual('succeeded',self.wait(e,request_id)['status'])
  directory=e.station_store.record_dir('map-1','station-1','1',1)
  self.assertTrue(os.path.isdir(directory))
  result=e.submit({
      'request_id':'delete-active','action':'vision_station_delete',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':1,'dry_run':False})
  self.assertEqual('succeeded',result['status'])
  self.assertTrue(result['metrics']['teaching_cancelled'])
  self.assertFalse(e.teaching.active)
  self.assertFalse(os.path.exists(directory))
  self.assertIn('stop_drag_and_enable',e._icp().events)
 def test_motion_is_rejected_until_explicit_enable(self):
  e=self.make(); r=e.submit({'request_id':'b','action':'vision_icp_record_a','params':{},'timeout_sec':1,'dry_run':False});
  # The asynchronous worker must fail closed before it can operate the CR5.
  import time; time.sleep(.02); self.assertEqual('failed',e.tasks['b']['status'])
 def test_busy_request_is_idempotently_recorded(self):
  e=self.make(); e.owner='running'
  req={'request_id':'busy','action':'apriltag_locate','params':{},'timeout_sec':1,'dry_run':False}
  first=e.submit(req); second=e.submit(req)
  self.assertEqual('busy',first['error_code']); self.assertEqual(first,second)
 def test_removed_unscoped_record_action_is_rejected(self):
  e=self.make(); req={'request_id':'dry','action':'vision_icp_record_a','params':{},'timeout_sec':1,'dry_run':True}
  e.submit(req)
  import time; time.sleep(.02)
  self.assertEqual('failed',e.tasks['dry']['status'])
  self.assertIn('unsupported action',e.tasks['dry']['message'])
  self.assertFalse(__import__('os').path.exists(__import__('os').path.join(e.root,'icp_a.json')))
 def test_robot_enable_short_circuits_when_already_enabled(self):
  e=self.make(); req={'request_id':'robot-enable','action':'robot_enable','params':{},'timeout_sec':1,'dry_run':False}
  self.assertEqual('accepted',e.submit(req)['status'])
  import time
  for unused in range(20):
   if e.tasks['robot-enable']['status'] != 'accepted' and e.tasks['robot-enable']['status'] != 'running': break
   time.sleep(.01)
  self.assertEqual('succeeded',e.tasks['robot-enable']['status'])
  self.assertTrue(e._icp().initialized)
  self.assertEqual([],e.fake_robot.calls)
  self.assertTrue(e.tasks['robot-enable']['metrics']['already_enabled'])
 def test_robot_enable_calls_controller_from_disabled_mode(self):
  robot=FakeRobot(); robot.mode=4; e=self.make(robot)
  req={'request_id':'robot-enable-disabled','action':'robot_enable',
       'params':{},'timeout_sec':1,'dry_run':False}
  e.submit(req)
  result=self.wait(e,'robot-enable-disabled')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['enable'],robot.calls)
  self.assertFalse(result['metrics']['already_enabled'])
 def test_robot_disable_revokes_motion_permission(self):
  e=self.make(); e.execution_enabled=True
  req={'request_id':'robot-disable','action':'robot_disable','params':{},'timeout_sec':1,'dry_run':False}
  e.submit(req)
  import time
  for unused in range(20):
   if e.tasks['robot-disable']['status'] not in ('accepted','running'): break
   time.sleep(.01)
  self.assertEqual('succeeded',e.tasks['robot-disable']['status'])
  self.assertFalse(e.execution_enabled)
  self.assertFalse(e._icp().initialized)
 def test_manual_drag_commands_enter_and_leave_backdrive(self):
  e=self.make()
  start={'request_id':'drag-start','action':'robot_start_drag',
         'params':{},'timeout_sec':2,'dry_run':False}
  self.assertEqual('accepted',e.submit(start)['status'])
  started=self.wait(e,'drag-start')
  self.assertEqual('succeeded',started['status'])
  self.assertEqual(6,started['metrics']['robot_mode'])
  self.assertTrue(started['metrics']['dragging'])
  stop={'request_id':'drag-stop','action':'robot_stop_drag',
        'params':{},'timeout_sec':2,'dry_run':False}
  self.assertEqual('accepted',e.submit(stop)['status'])
  stopped=self.wait(e,'drag-stop')
  self.assertEqual('succeeded',stopped['status'])
  self.assertEqual(5,stopped['metrics']['robot_mode'])
  self.assertFalse(stopped['metrics']['dragging'])
  self.assertFalse(e.execution_enabled)
 def test_drag_teaching_records_a_b_and_finishes_disabled(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  backend=e._icp()
  e.execution_enabled=True
  for request_id,command in [
      ('teach-start','start'),('teach-ref',1),('teach-w1',2),
      ('teach-w2',3),('teach-w3',4),('teach-finish',0)]:
   params={'command':command}
   if command == 'start':
    params.update(
        mapid='map-1',poseid='station-1',point_type=1)
   req={'request_id':request_id,'action':'vision_point_teach',
        'params':params,'timeout_sec':2,'dry_run':False}
   self.assertEqual('accepted',e.submit(req)['status'])
   result=self.wait(e,request_id)
   self.assertEqual('succeeded',result['status'])
  self.assertFalse(e.icp_teaching['active'])
  self.assertEqual('idle',e.icp_teaching['phase'])
  self.assertFalse(e.execution_enabled)
  self.assertEqual(5,backend.mode)
  self.assertEqual(5,result['metrics']['robot_mode'])
  self.assertTrue(result['metrics']['reset_completed'])
  self.assertEqual(1,backend.execution_events.count('reset'))
  self.assertFalse(result['metrics']['lock_held'])
  self.assertIsNone(result['metrics']['owner'])
  point_dir=os.path.join(
      e.root,'records','map-1_station-1_1_1')
  with open(os.path.join(point_dir,'icp_a.json')) as stream:
   a=json.load(stream)
  with open(os.path.join(point_dir,'work_3.json')) as stream:
   w3=json.load(stream)
  self.assertTrue(os.path.isfile(
      os.path.join(point_dir,'icp_a.npz')))
  self.assertEqual(a['record_id'],w3['a_record_id'])
  self.assertEqual(point_dir,a['record_dir'])
  self.assertEqual(point_dir,w3['record_dir'])
  self.assertAlmostEqual(25.0,w3['T_A_to_B'][0][3])
  self.assertTrue(a['teaching_drag'])
  self.assertTrue(w3['teaching_drag'])
  manifest=os.path.join(
      e.root,'tasks','map-1','station-1','map-1_station-1.json')
  with open(manifest) as stream:
   points=json.load(stream)
  self.assertEqual(['1'],[point['command'] for point in points])
  self.assertEqual(a['record_id'],points[0]['reference']['record_id'])
  self.assertEqual(['W1','W2','W3'],[
      work['work_id'] for work in points[0]['work_points']])
  self.assertEqual(1,points[0]['point_type'])
  self.assertEqual(
      os.path.join(point_dir,'icp_a.npz'),
      points[0]['reference']['template_path'])
 def test_new_type1_discards_incomplete_old_session_and_switches(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  for request_id,params in [
      ('old-tag',{
          'command':'start','mapid':'map-1','poseid':'old-tag',
          'point_type':0}),
      ('new-icp',{
          'command':'start','mapid':'map-1','poseid':'new-icp',
          'point_type':1})]:
   e.submit({'request_id':request_id,'action':'vision_point_teach',
             'params':params,'timeout_sec':2,'dry_run':False})
   result=self.wait(e,request_id)
   self.assertEqual('succeeded',result['status'])
  self.assertEqual(
      ('map-1','new-icp','1',1),e.teaching.identity())
  self.assertTrue(result['metrics']['previous_teaching_finalized'])
  self.assertTrue(result['metrics']['previous_incomplete_discarded'])
  self.assertFalse(result['metrics']['previous_saved'])
  self.assertEqual(
      ['start_drag','stop_drag_and_enable','start_drag'],
      e._icp().events)
 def test_new_type1_preserves_completed_old_group_and_switches(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  for request_id,command,extra in [
      ('old-start','start',{
          'mapid':'map-1','poseid':'old-icp','point_type':1}),
      ('old-ref',1,{}),
      ('old-work',2,{})]:
   params={'command':command}; params.update(extra)
   e.submit({'request_id':request_id,'action':'vision_point_teach',
             'params':params,'timeout_sec':2,'dry_run':False})
   self.assertEqual('succeeded',self.wait(e,request_id)['status'])
  e.submit({'request_id':'new-tag','action':'vision_point_teach',
            'params':{'command':'start','mapid':'map-1',
                      'poseid':'new-tag','point_type':0},
            'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'new-tag')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(('map-1','new-tag','1',0),e.teaching.identity())
  self.assertTrue(result['metrics']['previous_teaching_finalized'])
  self.assertTrue(result['metrics']['previous_saved'])
  self.assertFalse(result['metrics']['previous_incomplete_discarded'])
  points,unused_path=e.station_store.load('map-1','old-icp')
  self.assertEqual(1,len(points))
  self.assertEqual(1,len(points[0]['work_points']))
  self.assertNotIn('reset',e._icp().execution_events)
 def test_teaching_b_requires_a_in_same_session(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  for request_id,command in [('start','start'),('b',2)]:
   params={'command':command}
   if command == 'start':
    params.update(mapid='map-1',poseid='station-1',point_type=1)
   e.submit({'request_id':request_id,'action':'vision_point_teach',
             'params':params,'timeout_sec':2,'dry_run':False})
   result=self.wait(e,request_id)
  self.assertEqual('failed',result['status'])
  self.assertIn('before Ref',result['message'])
 def test_repeated_a_is_rejected_without_replacing_first_record(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  commands=[
      ('start','start',{'mapid':'map-1','poseid':'station-1'}),
      ('first-a',1,{}),
      ('second-a',1,{})]
  for request_id,command,extra in commands:
   params={'command':command}; params.update(extra)
   if command == 'start':
    params['point_type']=1
   e.submit({'request_id':request_id,'action':'vision_point_teach',
             'params':params,'timeout_sec':2,'dry_run':False})
   result=self.wait(e,request_id)
  self.assertEqual('failed',result['status'])
  self.assertIn('already recorded',result['message'])
  self.assertEqual('awaiting_work',e.icp_teaching['phase'])
  self.assertIsNotNone(e.icp_teaching['a_record_id'])
 def test_finish_zero_is_idempotent_when_idle(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  request={'request_id':'idle-finish','action':'vision_point_teach',
           'params':{'command':0,'point_type':1},
           'timeout_sec':2,'dry_run':False}
  e.submit(request)
  result=self.wait(e,'idle-finish')
  self.assertEqual('succeeded',result['status'])
  self.assertIn('already idle',result['message'])
  self.assertFalse(e.icp_teaching['active'])
 def test_apriltag_mode_locates_and_picks_in_one_task(self):
  e=self.make(apriltag_backend_factory=FakeAprilTagBackend)
  e.execution_enabled=True
  request={'request_id':'tag-one-shot',
           'action':'apriltag_locate_and_pick',
           'params':{},'timeout_sec':2,'dry_run':False}
  self.assertEqual('accepted',e.submit(request)['status'])
  result=self.wait(e,'tag-one-shot')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(1,e._apriltag().locate_calls)
  self.assertEqual(1,e._apriltag().pick_calls)
  self.assertTrue(os.path.isfile(
      os.path.join(e.root,'apriltag_cache.json')))
 def test_apriltag_locate_and_touch_is_the_production_alias(self):
  e=self.make(apriltag_backend_factory=FakeAprilTagBackend)
  e.execution_enabled=True
  request={'request_id':'tag-touch',
           'action':'apriltag_locate_and_touch',
           'params':{},'timeout_sec':2,'dry_run':False}
  e.submit(request)
  result=self.wait(e,'tag-touch')
  self.assertEqual('succeeded',result['status'])
  self.assertIn('touch point completed',result['message'])
  self.assertEqual(1,e._apriltag().locate_calls)
  self.assertEqual(1,e._apriltag().pick_calls)
 def test_apriltag_teaching_saves_one_observation_pose_and_offset(self):
  e=self.make(
      icp_backend_factory=FakeTeachingBackend,
      apriltag_backend_factory=FakeAprilTagBackend)
  start={
      'request_id':'tag-teach-start','action':'vision_point_teach',
      'params':{
          'command':'start','mapid':'map-1','poseid':'station-1',
          'point_type':0,'tag_id':0,
          'tag_offset_xyz_mm':[5.0,-10.0,0.0]},
      'timeout_sec':2,'dry_run':False}
  e.submit(start)
  started=self.wait(e,'tag-teach-start')
  self.assertEqual('succeeded',started['status'])
  self.assertEqual(0,started['metrics']['teaching_point_type'])
  record={
      'request_id':'tag-teach-record','action':'vision_point_teach',
      'params':{'command':1},'timeout_sec':2,'dry_run':False}
  e.submit(record)
  result=self.wait(e,'tag-teach-record')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual('awaiting_work',result['metrics']['teaching_phase'])
  self.assertTrue(e._apriltag().wait_for_new_detection)
  self.assertEqual([(2.0,8.0)],e._icp().stable_calls)
  work={
      'request_id':'tag-teach-work','action':'vision_point_teach',
      'params':{'command':2},'timeout_sec':2,'dry_run':False}
  e.submit(work)
  result=self.wait(e,'tag-teach-work')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual('W1',result['metrics']['work_id'])
  self.assertTrue(result['metrics']['teaching_active'])
  manifest=e._station_manifest_path('map-1','station-1')
  with open(manifest) as stream:
   points=json.load(stream)
  self.assertEqual(1,len(points))
  self.assertEqual(0,points[0]['point_type'])
  self.assertEqual('1',points[0]['command'])
  self.assertEqual(
      [25.0,0.0,0.0,0.0,0.0,0.0],
      points[0]['reference']['observe_flange_pose'])
  self.assertEqual(1,len(points[0]['work_points']))
  self.assertEqual('W1',points[0]['work_points'][0]['work_id'])
  self.assertFalse(os.path.isdir(
      os.path.join(e.root,'records','apriltag_point')))
 def test_apriltag_teaching_defaults_tag_id_to_zero(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  e.submit({
      'request_id':'tag-id-default','action':'vision_point_teach',
      'params':{
          'command':'start','mapid':'map-1','poseid':'station-1',
          'point_type':0},
      'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'tag-id-default')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(0,result['metrics']['apriltag_tag_id'])
 def test_apriltag_station_execution_uses_saved_ref_and_work(self):
  e=self.make(apriltag_backend_factory=FakeAprilTagBackend)
  self.save_tag_group(e,'1',0,100.0)
  e.execution_enabled=True
  request={
      'request_id':'tag-station-run','action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':False}
  e.submit(request)
  result=self.wait(e,'tag-station-run')
  self.assertEqual('succeeded',result['status'])
  backend=e._apriltag()
  self.assertEqual(
      [[100.0,0.0,100.0,0.0,0.0,0.0]],
      backend.observation_calls)
  self.assertEqual([0,0,0],backend.last_offset)
  self.assertEqual(1,backend.locate_calls)
  self.assertEqual([2.0],backend.detection_timeouts)
  self.assertEqual(1,backend.pick_calls)
  self.assertEqual(0,result['metrics']['points'][0]['point_type'])
 def test_apriltag_station_searches_left_and_right_after_timeouts(self):
  e=self.make(apriltag_backend_factory=FakeSearchAprilTagBackend)
  self.save_tag_group(e,'1',0,100.0)
  e.execution_enabled=True
  e.submit({
      'request_id':'tag-search','action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'tag-search')
  self.assertEqual('succeeded',result['status'])
  backend=e._apriltag()
  self.assertEqual(3,backend.locate_calls)
  self.assertEqual([2.0,2.0,2.0],backend.detection_timeouts)
  self.assertEqual(
      ['AprilTag search +20.0mm','AprilTag search -20.0mm'],
      backend.move_labels[:2])
  self.assertEqual(-20.0,
      result['metrics']['points'][0]['search_lateral_offset_mm'])
 def test_apriltag_missing_after_search_returns_to_center_and_fails(self):
  e=self.make(apriltag_backend_factory=FakeMissingAprilTagBackend)
  self.save_tag_group(e,'1',0,100.0)
  e.execution_enabled=True
  e.submit({
      'request_id':'tag-search-missing','action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'tag-search-missing')
  self.assertEqual('failed',result['status'])
  self.assertIn('未检测到Tag ID 0',result['message'])
  backend=e._apriltag()
  self.assertEqual(3,backend.locate_calls)
  self.assertEqual('AprilTag search return',backend.move_labels[-1])
  self.assertAlmostEqual(100.0,backend.move_targets[-1][0,3])
 def test_apriltag_station_selects_each_saved_tag_id(self):
  e=self.make(apriltag_backend_factory=FakeAprilTagBackend)
  self.save_tag_group(e,'1',1,100.0)
  self.save_tag_group(e,'2',7,100.0)
  e.execution_enabled=True
  e.submit({
      'request_id':'multi-tag-station',
      'action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'multi-tag-station')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual([1,7],e._apriltag().tag_ids)
  self.assertEqual(
      [1,7],
      [point['tag_id'] for point in result['metrics']['points']])
 def test_station_point_query_returns_both_point_types(self):
  e=self.make()
  self.save_tag_group(e,'2',0,100.0)
  path=e._station_manifest_path('map-1','station-1')
  with open(path) as stream:
   points=json.load(stream)
  points.insert(0,{
      'schema_version':4,'mapid':'map-1','poseid':'station-1',
      'command':'1','point_type':1,'reference':{},'work_points':[]})
  from vision_arm_executor.store import atomic_json
  atomic_json(path,points)
  result=e.submit({
      'request_id':'point-query','action':'vision_station_points',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':True})
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(
      [('1',1),('2',0)],
      [(point['command'],point['point_type'])
       for point in result['metrics']['points']])
 def test_station_executes_all_taught_commands_in_manifest_order(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  sequence=[]
  for label in ('P1','P2','P3'):
   sequence.extend([
       ('%s-start' % label,'start',
        {'mapid':'map-1','poseid':'station-1'}),
       ('%s-a' % label,1,{}),
       ('%s-b' % label,2,{}),
       ('%s-finish' % label,0,{})])
  for request_id,command,extra in sequence:
    params={'command':command}
    params.update(extra)
    if command == 'start':
     params['point_type']=1
    e.submit({'request_id':request_id,'action':'vision_point_teach',
              'params':params,'timeout_sec':2,'dry_run':False})
    self.assertEqual(
        'succeeded',self.wait(e,request_id)['status'])
  e.execution_enabled=True
  # Drop the teaching-time reset events so we assert only on the production
  # execution order below.
  e._icp().execution_events = []
  request={'request_id':'station-run',
           'action':'vision_icp_align_and_move_b',
           'params':{'mapid':'map-1','poseid':'station-1'},
           'timeout_sec':2,'dry_run':False}
  e.submit(request)
  result=self.wait(e,'station-run')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['1','2','3'],result['metrics']['commands'])
  self.assertEqual(3,len(e._icp().align_calls))
  self.assertEqual(3,len(e._icp().move_calls))
  self.assertEqual(
      ['recover_a','align','move_b'] * 3,
      e._icp().execution_events)
  self.assertEqual(
      ['map-1_station-1_1_1','map-1_station-1_2_1',
       'map-1_station-1_3_1'],
      sorted(name for name in os.listdir(os.path.join(e.root,'records'))
             if name.startswith('map-1_station-1_')))
 def test_production_executes_mixed_points_in_manifest_order(self):
  e=self.make(
      icp_backend_factory=FakeTeachingBackend,
      apriltag_backend_factory=FakeAprilTagBackend,
      cfg_overrides={'reset_after_each_point':True})

  def teach_icp(label):
   for suffix,command,extra in [
       ('start','start',{
           'mapid':'map-1','poseid':'station-1',
           'label':label,'point_type':1}),
       ('a',1,{}),
       ('b',2,{}),
       ('finish',0,{})]:
    request_id='%s-%s' % (label,suffix)
    params={'command':command}
    params.update(extra)
    e.submit({
        'request_id':request_id,'action':'vision_point_teach',
        'params':params,'timeout_sec':2,'dry_run':False})
    self.assertEqual('succeeded',self.wait(e,request_id)['status'])

  teach_icp('P1')
  self.save_tag_group(e,'2',0,100.0)
  teach_icp('P3')
  self.save_tag_group(e,'4',0,200.0)

  # Drop the two teaching-time reset events so the count below reflects only
  # the production per-point resets.
  e._icp().execution_events = []
  execution_order=[]
  icp=e._icp()
  original_align=icp.align
  def ordered_align(path,*args,**kwargs):
   execution_order.append(os.path.basename(os.path.dirname(path)))
   return original_align(path,*args,**kwargs)
  icp.align=ordered_align

  apriltag=e._apriltag()
  original_observation=apriltag.move_to_observation
  def ordered_observation(pose,safety_check,joints=None):
   execution_order.append(
       'map-1_station-1_2_0'
       if pose[0] == 100 else 'map-1_station-1_4_0')
   return original_observation(pose,safety_check,joints=joints)
  apriltag.move_to_observation=ordered_observation

  e.execution_enabled=True
  e.submit({
      'request_id':'mixed-station-run',
      'action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':5,'dry_run':False})
  result=self.wait(e,'mixed-station-run')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['1','2','3','4'],result['metrics']['commands'])
  self.assertEqual(
      [('1',1),('2',0),('3',1),('4',0)],
      [(point['command'],point['point_type'])
       for point in result['metrics']['points']])
  self.assertEqual(
      ['map-1_station-1_1_1',
       'map-1_station-1_2_0',
       'map-1_station-1_3_1',
       'map-1_station-1_4_0'],
      execution_order)
  self.assertEqual(4,e._icp().execution_events.count('reset'))
  self.assertTrue(all(
      point['reset_completed'] for point in result['metrics']['points']))
 def test_failed_icp_station_attempt_resets_before_returning_failure(self):
  e=self.make(
      icp_backend_factory=FakeFailedAlignmentBackend,
      cfg_overrides={'reset_after_each_point':True})
  for request_id,command,extra in [
      ('failed-start','start',{
          'mapid':'map-1','poseid':'station-1','point_type':1}),
      ('failed-a',1,{}),
      ('failed-b',2,{}),
      ('failed-finish',0,{})]:
   params={'command':command}; params.update(extra)
   e.submit({'request_id':request_id,'action':'vision_point_teach',
             'params':params,'timeout_sec':2,'dry_run':False})
   self.assertEqual('succeeded',self.wait(e,request_id)['status'])
  e._icp().execution_events=[]
  e.execution_enabled=True
  e.submit({
      'request_id':'failed-station-run',
      'action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'failed-station-run')
  self.assertEqual('failed',result['status'])
  self.assertIn('simulated ICP failure',result['message'])
  self.assertEqual(
      ['recover_a','align','reset'],e._icp().execution_events)
 def test_station_task_command_filter_selects_one_group(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  template=os.path.join(e.root,'template.npz')
  with open(template,'wb') as stream:
   stream.write(b'template')
  checksum=__import__('vision_arm_executor.store',
                      fromlist=['digest']).digest(template)
  a={'record_id':'a','template_path':template,
     'template_sha256':checksum,'handeye_sha256':checksum,
     'T_base_flange_A':np.eye(4).tolist()}
  # The test calibration path is /dev/null, so use its actual checksum.
  a['handeye_sha256']=__import__(
      'vision_arm_executor.store',fromlist=['digest']).digest('/dev/null')
  b={'record_id':'b','a_record_id':'a',
     'template_sha256':checksum,
     'handeye_sha256':a['handeye_sha256'],
     'T_A_to_B':np.eye(4).tolist()}
  path=e._station_manifest_path('map-1','station-1')
  from vision_arm_executor.store import atomic_json
  atomic_json(path,[
      {'schema_version':4,'mapid':'map-1','poseid':'station-1',
       'command':command,'point_type':1,'reference':dict(a),
       'work_points':[dict(
           b,work_id='W1',index=1,command=2)]}
      for command in ('1','2','3')])
  e.execution_enabled=True
  e.submit({'request_id':'selected',
            'action':'vision_icp_align_and_move_b',
            'params':{'mapid':'map-1','poseid':'station-1',
                      'task_command':'3'},
            'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'selected')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['3'],result['metrics']['commands'])
