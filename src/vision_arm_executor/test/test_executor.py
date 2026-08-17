import os
import json
import tempfile
import unittest

import numpy as np

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
  self.events=[]; self.execution_events=[]
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
 def initialize_robot(self):
  return True
 def align(self,path,max_iters,should_stop,progress_callback,motion_guard):
  self.execution_events.append('align')
  self.align_calls.append(path)
  progress_callback(1,max_iters,{'reason':'converged'})
  return IcpAlignment(True,{'reason':'converged'})
 def move_to_reference(self,transform,label,motion_guard):
  target=np.asarray(transform,dtype=float)
  motion_guard(target)
  self.execution_events.append('recover_a')
  return target
 def move_relative(self,delta,label,motion_guard):
  target=np.asarray(delta,dtype=float)
  motion_guard(target)
  self.execution_events.append('move_b')
  self.move_calls.append(label)
  return target
 def reset_robot(self):
  self.execution_events.append('reset')
  return True


class FakeAprilTagBackend(object):
 def __init__(self, unused_node, **unused):
  self.locate_calls=0; self.pick_calls=0; self.motion_disabled=True
  self.observation_calls=[]; self.last_offset=None; self.tag_ids=[]
  self.target=np.eye(4); self.target[0,3]=100.0
 def identity(self,tag_id=None):
  selected=0 if tag_id is None else int(tag_id)
  return AprilTagIdentity(
      selected,'tag36h11:%d' % selected,'/dev/null','/dev/null')
 def robot_state(self):
  return RobotState(5,[100,0,0,0,0,0],False,5,6)
 def disable_motion(self):
  self.motion_disabled=True
 def locate(self,tag_id=None,tag_offset_xyz_mm=None):
  self.locate_calls+=1
  self.tag_ids.append(0 if tag_id is None else int(tag_id))
  self.last_offset=list(tag_offset_xyz_mm or [0,0,0])
  self.target[:3,3]=np.asarray(self.last_offset,dtype=float)
  return AprilTagLocation(
      self.target.copy(),
      {'frames':10,'spread_mm':0.2,
       'flange_xyzrpy':[100,0,0,0,0,0]},
      self.identity(tag_id))
 def move_to_observation(self,pose,safety_check):
  target=np.eye(4); target[:3,3]=np.asarray(pose[:3],dtype=float)
  safety_check('before_observation',target)
  self.observation_calls.append(list(pose))
  return target
 def prepare_pick(self,unused_localization):
  self.motion_disabled=False
 def localization_drift(self,unused_localization):
  return 0.0,0.0
 def pick(self,safety_check):
  safety_check('pick',self.target)
  self.pick_calls+=1
  self.motion_disabled=True


class ExecutorTests(unittest.TestCase):
 def make(self, robot=None, icp_backend_factory=None,
          apriltag_backend_factory=None, cfg_overrides=None):
  robot=robot or FakeRobot()
  cfg={'data_dir':tempfile.mkdtemp(),'robot_speed':15,'workspace_min_xyz_mm':[-1e3]*3,'workspace_max_xyz_mm':[1e3]*3,'max_move_translation_mm':300,'max_move_rotation_deg':45,'icp_frames':2,'icp_max_iters':1,'apriltag_cache_ttl_sec':1,'apriltag_max_robot_drift_mm':10,'apriltag_max_robot_drift_deg':3,'handeye_path':'/dev/null','tcp_calibration_path':'/dev/null'}
  cfg.update(cfg_overrides or {})
  executor=VisionExecutor(None,cfg,robot_factory=lambda *a:robot,
                          icp_backend_factory=icp_backend_factory,
                          apriltag_backend_factory=apriltag_backend_factory)
  executor.fake_robot=robot
  return executor
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
  original_save=e._save_station_icp_point
  def save_station(*args,**kwargs):
   backend.events.append('save_station')
   return original_save(*args,**kwargs)
  e._save_station_icp_point=save_station
  e.execution_enabled=True
  for request_id,command in [
      ('teach-start','start'),('teach-a',1),('teach-b',2)]:
   params={'command':command}
   if command == 'start':
    params.update(
        mapid='map-1',poseid='station-1',label='P1',point_type=1)
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
  self.assertFalse(result['metrics']['lock_held'])
  self.assertIsNone(result['metrics']['owner'])
  self.assertLess(
      backend.events.index('read_b'),
      backend.events.index('stop_drag_and_enable'))
  self.assertLess(
      backend.events.index('stop_drag_and_enable'),
      backend.events.index('save_station'))
  point_dir=os.path.join(
      e.root,'records','map-1_station-1_P1_1')
  with open(os.path.join(point_dir,'icp_a.json')) as stream:
   a=json.load(stream)
  with open(os.path.join(point_dir,'icp_b.json')) as stream:
   b=json.load(stream)
  self.assertTrue(os.path.isfile(
      os.path.join(point_dir,'icp_a.npz')))
  self.assertFalse(os.path.isdir(
      os.path.join(e.root,'records','icp_a')))
  self.assertFalse(os.path.isdir(
      os.path.join(e.root,'records','icp_b')))
  self.assertEqual(a['record_id'],b['a_record_id'])
  self.assertEqual(point_dir,a['record_dir'])
  self.assertEqual(point_dir,b['record_dir'])
  self.assertAlmostEqual(25.0,b['T_A_to_B'][0][3])
  self.assertTrue(a['teaching_drag'])
  self.assertTrue(b['teaching_drag'])
  manifest=os.path.join(
      e.root,'tasks','map-1','station-1','map-1_station-1.json')
  with open(manifest) as stream:
   points=json.load(stream)
  self.assertEqual(['P1'],[point['label'] for point in points])
  self.assertEqual(a['record_id'],points[0]['icp_a']['record_id'])
  self.assertEqual(b['record_id'],points[0]['icp_b']['record_id'])
  self.assertEqual(1,points[0]['point_type'])
  self.assertNotIn('mode',points[0])
  self.assertEqual(
      os.path.join(point_dir,'icp_a.npz'),
      points[0]['icp_a']['template_path'])
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
  self.assertIn('before A',result['message'])
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
  self.assertEqual('awaiting_b',e.icp_teaching['phase'])
  self.assertIsNotNone(e.icp_teaching['a_record_id'])
 def test_finish_is_abort_and_is_idempotent_when_idle(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  request={'request_id':'idle-finish','action':'vision_point_teach',
           'params':{'command':3,'point_type':1},
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
  e=self.make(icp_backend_factory=FakeTeachingBackend)
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
  self.assertEqual('P1',result['metrics']['saved_label'])
  self.assertFalse(result['metrics']['teaching_active'])
  manifest=e._station_manifest_path('map-1','station-1')
  with open(manifest) as stream:
   points=json.load(stream)
  self.assertEqual(1,len(points))
  self.assertEqual(0,points[0]['point_type'])
  self.assertEqual(
      [120.0,220.0,320.0,10.0,20.0,30.0],
      points[0]['apriltag']['observe_flange_pose'])
  self.assertEqual(
      [5.0,-10.0,0.0],
      points[0]['apriltag']['tag_offset_xyz_mm'])
  self.assertFalse(os.path.isdir(
      os.path.join(e.root,'records','apriltag_point')))
 def test_apriltag_teaching_requires_tag_id(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  e.submit({
      'request_id':'tag-id-required','action':'vision_point_teach',
      'params':{
          'command':'start','mapid':'map-1','poseid':'station-1',
          'point_type':0},
      'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'tag-id-required')
  self.assertEqual('failed',result['status'])
  self.assertIn('tag_id is required',result['message'])
 def test_apriltag_station_execution_uses_saved_pose_and_offset(self):
  e=self.make(apriltag_backend_factory=FakeAprilTagBackend)
  e.station_store.save_apriltag(
      'map-1','station-1','P1',
      [100,0,100,0,0,0],0,[3.0,4.0,5.0])
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
  self.assertEqual([3.0,4.0,5.0],backend.last_offset)
  self.assertEqual(1,backend.locate_calls)
  self.assertEqual(1,backend.pick_calls)
  self.assertEqual(0,result['metrics']['points'][0]['point_type'])
 def test_apriltag_station_selects_each_saved_tag_id(self):
  e=self.make(apriltag_backend_factory=FakeAprilTagBackend)
  e.station_store.save_apriltag(
      'map-1','station-1','P1',
      [100,0,100,0,0,0],1,[0,0,0])
  e.station_store.save_apriltag(
      'map-1','station-1','P2',
      [100,0,100,0,0,0],7,[5,0,0])
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
  e.station_store.save_apriltag(
      'map-1','station-1','P2',
      [100,0,100,0,0,0],0,[0,0,0])
  path=e._station_manifest_path('map-1','station-1')
  with open(path) as stream:
   points=json.load(stream)
  points.insert(0,{
      'schema_version':2,'mapid':'map-1','poseid':'station-1',
      'label':'P1','point_type':1,'icp_a':{},'icp_b':{}})
  from vision_arm_executor.store import atomic_json
  atomic_json(path,points)
  result=e.submit({
      'request_id':'point-query','action':'vision_station_points',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':2,'dry_run':True})
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(
      [('P1',1),('P2',0)],
      [(point['label'],point['point_type'])
       for point in result['metrics']['points']])
 def test_station_executes_all_taught_labels_in_manifest_order(self):
  e=self.make(icp_backend_factory=FakeTeachingBackend)
  sequence=[]
  for label in ('P1','P2','P3'):
   sequence.extend([
       ('%s-start' % label,'start',
        {'mapid':'map-1','poseid':'station-1'}),
       ('%s-a' % label,1,{}),
       ('%s-b' % label,2,{})])
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
  request={'request_id':'station-run',
           'action':'vision_icp_align_and_move_b',
           'params':{'mapid':'map-1','poseid':'station-1'},
           'timeout_sec':2,'dry_run':False}
  e.submit(request)
  result=self.wait(e,'station-run')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['P1','P2','P3'],result['metrics']['labels'])
  self.assertEqual(3,len(e._icp().align_calls))
  self.assertEqual(3,len(e._icp().move_calls))
  self.assertEqual(
      ['recover_a','align','move_b'] * 3,
      e._icp().execution_events)
  self.assertEqual(
      ['map-1_station-1_P1_1','map-1_station-1_P2_1',
       'map-1_station-1_P3_1'],
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
       ('b',2,{})]:
    request_id='%s-%s' % (label,suffix)
    params={'command':command}
    params.update(extra)
    e.submit({
        'request_id':request_id,'action':'vision_point_teach',
        'params':params,'timeout_sec':2,'dry_run':False})
    self.assertEqual('succeeded',self.wait(e,request_id)['status'])

  teach_icp('P1')
  e.station_store.save_apriltag(
      'map-1','station-1','P2',
      [100,0,100,0,0,0],0,[1,2,3])
  teach_icp('P3')
  e.station_store.save_apriltag(
      'map-1','station-1','P4',
      [200,0,100,0,0,0],0,[4,5,6])

  execution_order=[]
  icp=e._icp()
  original_align=icp.align
  def ordered_align(path,*args,**kwargs):
   execution_order.append(os.path.basename(os.path.dirname(path)))
   return original_align(path,*args,**kwargs)
  icp.align=ordered_align

  apriltag=e._apriltag()
  original_observation=apriltag.move_to_observation
  def ordered_observation(pose,safety_check):
   execution_order.append(
       'map-1_station-1_P2_0'
       if pose[0] == 100 else 'map-1_station-1_P4_0')
   return original_observation(pose,safety_check)
  apriltag.move_to_observation=ordered_observation

  e.execution_enabled=True
  e.submit({
      'request_id':'mixed-station-run',
      'action':'vision_station_execute',
      'params':{'mapid':'map-1','poseid':'station-1'},
      'timeout_sec':5,'dry_run':False})
  result=self.wait(e,'mixed-station-run')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['P1','P2','P3','P4'],result['metrics']['labels'])
  self.assertEqual(
      [('P1',1),('P2',0),('P3',1),('P4',0)],
      [(point['label'],point['point_type'])
       for point in result['metrics']['points']])
  self.assertEqual(
      ['map-1_station-1_P1_1',
       'map-1_station-1_P2_0',
       'map-1_station-1_P3_1',
       'map-1_station-1_P4_0'],
      execution_order)
  self.assertEqual(4,e._icp().execution_events.count('reset'))
  self.assertTrue(all(
      point['reset_completed'] for point in result['metrics']['points']))
 def test_station_label_filter_preserves_requested_order(self):
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
      {'schema_version':2,'mapid':'map-1','poseid':'station-1',
       'label':label,'point_type':1,
       'icp_a':dict(a),'icp_b':dict(b)}
      for label in ('P1','P2','P3')])
  e.execution_enabled=True
  e.submit({'request_id':'selected',
            'action':'vision_icp_align_and_move_b',
            'params':{'mapid':'map-1','poseid':'station-1',
                      'label':'P3,P1'},
            'timeout_sec':2,'dry_run':False})
  result=self.wait(e,'selected')
  self.assertEqual('succeeded',result['status'])
  self.assertEqual(['P3','P1'],result['metrics']['labels'])
