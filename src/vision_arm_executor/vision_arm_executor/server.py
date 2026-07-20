import socket, threading
from .protocol import decode_line, encode_line

class JsonLineServer(object):
 def __init__(self, host, port, handler, timeout=5., max_bytes=65536): self.host,self.port,self.handler,self.timeout,self.max_bytes=host,int(port),handler,timeout,max_bytes; self.stop=threading.Event(); self.sock=None
 def start(self):
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM); self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); self.sock.bind((self.host,self.port)); self.sock.listen(16); self.sock.settimeout(.5); threading.Thread(target=self._accept,daemon=True).start()
 def close(self): self.stop.set(); self.sock and self.sock.close()
 def _accept(self):
  while not self.stop.is_set():
   try: c,_=self.sock.accept()
   except socket.timeout: continue
   except OSError: return
   threading.Thread(target=self._client,args=(c,),daemon=True).start()
 def _client(self,c):
  try:
   c.settimeout(self.timeout); buf=b''
   while not self.stop.is_set():
    b=c.recv(4096)
    if not b:return
    buf+=b
    if len(buf)>self.max_bytes: raise ValueError('message_too_large')
    while b'\n' in buf:
     line,buf=buf.split(b'\n',1)
     if line: c.sendall(encode_line(self.handler(decode_line(line,self.max_bytes))))
  except Exception as e:
   try:c.sendall(encode_line({'status':'failed','error_code':'rpc_error','message':str(e)}))
   except Exception:pass
  finally:c.close()
