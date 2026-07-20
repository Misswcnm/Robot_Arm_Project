import json
import uuid

MAX_MESSAGE_BYTES = 65536
VALID_ACTIONS = set(['arm_status','vision_icp_record_a','vision_icp_record_b',
 'vision_icp_align','vision_icp_align_and_move_b','apriltag_locate','apriltag_validate',
 'apriltag_pick','task_status','task_cancel','execution_enable','execution_disable','health'])

def request_id(value=None): return value or str(uuid.uuid4())
def decode_line(data, limit=MAX_MESSAGE_BYTES):
    if len(data) > limit: raise ValueError('message_too_large')
    item = json.loads(data.decode('utf-8') if hasattr(data,'decode') else data)
    if not isinstance(item, dict): raise ValueError('request_must_be_object')
    item['request_id'] = request_id(item.get('request_id'))
    item.setdefault('params', {}); item.setdefault('timeout_sec', 120); item.setdefault('dry_run', False)
    if item.get('action') not in VALID_ACTIONS: raise ValueError('unsupported_action')
    if not isinstance(item['params'], dict): raise ValueError('params_must_be_object')
    return item
def encode_line(item): return (json.dumps(item, ensure_ascii=False, sort_keys=True, separators=(',', ':'))+'\n').encode('utf-8')
