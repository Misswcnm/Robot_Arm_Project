import json
import os
import tempfile
import unittest
from vision_arm_executor.protocol import decode_line, encode_line
from vision_arm_executor.store import atomic_json, load_json

class ProtocolTests(unittest.TestCase):
 def test_round_trip_and_generated_id(self):
  item=decode_line(b'{"action":"health"}')
  self.assertTrue(item['request_id']); self.assertEqual('health',decode_line(encode_line(item).strip())['action'])
 def test_rejects_bad_payload(self):
  with self.assertRaises(ValueError): decode_line(b'{"action":"nope"}')
 def test_atomic_json(self):
  root=tempfile.mkdtemp(); path=os.path.join(root,'x.json'); atomic_json(path,{'schema_version':1}); self.assertEqual(1,load_json(path)['schema_version'])
