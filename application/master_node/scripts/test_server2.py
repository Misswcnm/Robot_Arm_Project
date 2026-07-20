#!/usr/bin/env python2
#coding=utf-8

import tornado.ioloop 
import tornado.web 
import tornado.websocket
from tornado.httpserver import HTTPServer
from tornado import gen 
from tornado.options import define, options, parse_command_line 

define("port", default=8888, help="run on given port", type=int) 

clients = dict() 
class IndexHandler(tornado.web.RequestHandler):
    def get(self):
        print("访问系统首页...") 
        self.render("index.html") 

class MyWebSocketHandler(tornado.websocket.WebSocketHandler):
    def open(self, *args, **kwargs): 
        self.id = self.get_argument("id") 
        self.stream.set_nodelay(True) 
        clients[self.id] = {"id": self.id, "object": self} 
        print(clients) 
        print("建立连接...")
    
    def on_message(self, message): 
        print("client %s received a message: %s" % (self.id, message)) # 关闭连接时被调用 
    
    def on_close(self): 
        if self.id in clients: 
            del clients[self.id] 
        print("client %s is closed" % self.id) 
    
    def check_origin(self, origin): 
        return True 

import threading 
import time 
import datetime 

def send_time(): 
    while True: 
        for key in clients.keys(): 
            msg = str(datetime.datetime.now()) 
            clients[key]["object"].write_message(msg) 
            print("write to client %s:%s" % (key, msg)) 
        time.sleep(1) 
        
app = tornado.web.Application([(r"/d-robot/NavigationStatus", MyWebSocketHandler) ]) 
server = HTTPServer(app,max_buffer_size=504857600, max_body_size=504857600)
if __name__ == "__main__": 
    threading.Thread(target=send_time).start()
    
    server.bind(options.port)
    server.start(1)
        # app.listen(port, address)

    tornado.ioloop.IOLoop.instance().start()
