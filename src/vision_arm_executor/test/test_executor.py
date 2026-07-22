import tempfile
import unittest
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
class ExecutorTests(unittest.TestCase):
 def make(self, robot=None):
  robot=robot or FakeRobot()
  cfg={'data_dir':tempfile.mkdtemp(),'robot_speed':15,'workspace_min_xyz_mm':[-1e3]*3,'workspace_max_xyz_mm':[1e3]*3,'max_move_translation_mm':300,'max_move_rotation_deg':45,'icp_frames':2,'icp_max_iters':1,'apriltag_cache_ttl_sec':1,'apriltag_max_robot_drift_mm':10,'apriltag_max_robot_drift_deg':3,'handeye_path':'/dev/null','tcp_calibration_path':'/dev/null'}
  executor=VisionExecutor(None,cfg,robot_factory=lambda *a:robot)
  executor.fake_robot=robot
  return executor
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
 def test_dry_run_record_does_not_replace_active_a(self):
  e=self.make(); req={'request_id':'dry','action':'vision_icp_record_a','params':{},'timeout_sec':1,'dry_run':True}
  e.submit(req)
  import time; time.sleep(.02)
  self.assertEqual('succeeded',e.tasks['dry']['status'])
  self.assertFalse(__import__('os').path.exists(__import__('os').path.join(e.root,'icp_a.json')))
 def test_robot_enable_is_serialized_and_marks_robot_initialized(self):
  e=self.make(); req={'request_id':'robot-enable','action':'robot_enable','params':{},'timeout_sec':1,'dry_run':False}
  self.assertEqual('accepted',e.submit(req)['status'])
  import time
  for unused in range(20):
   if e.tasks['robot-enable']['status'] != 'accepted' and e.tasks['robot-enable']['status'] != 'running': break
   time.sleep(.01)
  self.assertEqual('succeeded',e.tasks['robot-enable']['status'])
  self.assertTrue(e._icp().initialized)
  self.assertEqual(['enable'],e.fake_robot.calls)
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
