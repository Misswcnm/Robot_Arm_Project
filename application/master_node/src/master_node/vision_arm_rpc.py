#!/usr/bin/env python
# coding=utf-8
"""Python 2 compatible JSON-lines client for the ROS2 vision executor."""
from __future__ import print_function
import json
import socket
import threading
import time
import uuid

class VisionArmRpcClient(object):
    def __init__(self, host='127.0.0.1', port=17881, timeout=5.0, max_bytes=65536):
        self.host, self.port, self.timeout, self.max_bytes = host, int(port), float(timeout), int(max_bytes)
    def call(self, action, params=None, request_id=None, timeout_sec=120, dry_run=False):
        request_id = request_id or str(uuid.uuid4())
        request = {'request_id':request_id, 'action':action, 'params':params or {},
                   'timeout_sec':float(timeout_sec), 'dry_run':bool(dry_run)}
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.settimeout(self.timeout); sock.connect((self.host, self.port))
            sock.sendall((json.dumps(request, ensure_ascii=False)+'\n').encode('utf-8'))
            data = b''
            while b'\n' not in data:
                part = sock.recv(4096)
                if not part: raise IOError('vision executor disconnected')
                data += part
                if len(data) > self.max_bytes: raise IOError('vision response too large')
            return json.loads(data.split(b'\n', 1)[0].decode('utf-8'))
        except Exception as e:
            return {'request_id':request_id, 'action':action, 'backend':'vision', 'status':'failed',
                    'error_code':'rpc_unavailable', 'message':str(e), 'metrics':{}, 'artifacts':[]}
        finally:
            try: sock.close()
            except Exception: pass
    def health(self): return self.call('health')

class MechanicalArmResourceLock(object):
    """Process-local backend owner guard; never holds while a read-only query runs."""
    def __init__(self): self._lock=threading.RLock(); self.owner=None; self.started_at=None
    def acquire(self, backend, request_id):
        with self._lock:
            if self.owner and self.owner.get('request_id') != request_id: return False
            self.owner={'backend':backend, 'request_id':request_id}; self.started_at=time.time(); return True
    def release(self, request_id=None):
        with self._lock:
            if request_id is None or (self.owner and self.owner.get('request_id') == request_id): self.owner=None; self.started_at=None
    def status(self):
        with self._lock: return {'owner':self.owner, 'started_at':self.started_at, 'locked':bool(self.owner)}
