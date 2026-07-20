import tempfile
import unittest
from vision_arm_executor.executor import VisionExecutor

class FakeRobot(object):
 MODE_ENABLED=5; MODE_BACKDRIVE=6
 def __init__(self,*a): self.dragging=False
 def init(self): return True
 def get_mode(self): return 5
 def get_tool(self,**kw): return [0,0,100,0,0,0]
class ExecutorTests(unittest.TestCase):
 def make(self):
  cfg={'data_dir':tempfile.mkdtemp(),'robot_speed':15,'workspace_min_xyz_mm':[-1e3]*3,'workspace_max_xyz_mm':[1e3]*3,'max_move_translation_mm':300,'max_move_rotation_deg':45,'icp_frames':2,'icp_max_iters':1,'apriltag_cache_ttl_sec':1,'handeye_path':'/dev/null'}
  return VisionExecutor(None,cfg,robot_factory=lambda *a:FakeRobot())
 def test_enable_does_not_survive_new_executor(self):
  e=self.make(); self.assertFalse(e.execution_enabled); self.assertEqual('succeeded',e.submit({'request_id':'x','action':'execution_enable','params':{},'timeout_sec':1,'dry_run':False})['status']); self.assertTrue(e.execution_enabled); self.assertFalse(self.make().execution_enabled)
 def test_request_id_idempotent(self):
  e=self.make(); req={'request_id':'same','action':'health','params':{},'timeout_sec':1,'dry_run':False}; self.assertEqual(e.submit(req),e.submit(req))
 def test_cancel_unknown(self):
  e=self.make(); r=e.submit({'request_id':'c','action':'task_cancel','params':{'request_id':'none'},'timeout_sec':1,'dry_run':False}); self.assertEqual('failed',r['status'])
 def test_motion_is_rejected_until_explicit_enable(self):
  e=self.make(); r=e.submit({'request_id':'b','action':'vision_icp_record_a','params':{},'timeout_sec':1,'dry_run':False});
  # The asynchronous worker must fail closed before it can operate the CR5.
  import time; time.sleep(.02); self.assertEqual('failed',e.tasks['b']['status'])
