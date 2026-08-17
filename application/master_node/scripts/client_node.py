#!/usr/bin/env python


from tornado import httpclient
from rosbridge_library.util import json, bson


http_client = httpclient.HTTPClient()
point={}
point['x'] = 5.0
point['y'] = 0.0
position = {}
position['point'] = point
position['angle'] = 0.0
msg = {}
msg['position'] = position
http_request_body = json.dumps(msg)
http_request = httpclient.HTTPRequest("http://127.0.0.1:1819/d-robot/path_manage/get_path?path_name=test_yuanqu&return_method=restart","GET")

response = http_client.fetch(http_request)

print(response.body)
