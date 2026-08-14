#!/usr/bin/env python
#coding=utf-8

import os

def safe_str(e):
    """
    安全地转换异常为字符串，避免编码错误

    Args:
        e: 异常对象

    Returns:
        str: 安全的字符串表示
    """
    try:
        return str(e)
    except UnicodeError:
        return repr(e)
import socket
import json
import threading
import time
import struct
from datetime import datetime
# from typing import Optional, Dict, Any  # 注释掉类型注解导入以兼容低版本Python

class MechanicalArmController:
    """
    机械臂控制器 - TCP/IP版本
    负责在机器人到达指定点位时通过TCP/IP发送消息给NX
    接收NX发来的TCP消息
    使用TCP/IP通信替代ROS和SSH
    """

    def __init__(self, nx_host, nx_port, arm_vendor):
        """
        初始化机械臂控制器

        Args:
            nx_host: NX板IP地址
            nx_port: NX板TCP端口
            arm_vendor: 机械臂厂商 estun | dobot
        """
        self.nx_host = nx_host
        self.nx_port = nx_port
        self.arm_vendor = str(arm_vendor).strip().lower()
        if self.arm_vendor not in ("estun", "dobot"):
            raise ValueError("未知 arm_vendor={}".format(self.arm_vendor))

        # TCP连接相关
        self.tcp_socket = None
        self.tcp_connected = False
        self.connection_lock = threading.Lock()

        # 消息处理
        self.message_handlers = {}
        self.receive_thread = None
        self.shutdown_event = threading.Event()

        # 状态管理
        self.is_mechanical_arm_running = False
        self.status_lock = threading.Lock()

        # 机械臂完成信号管理
        self.mechanical_arm_completed = False
        self.last_completion_response = None
        self.completion_lock = threading.Lock()
        self.completion_condition = threading.Condition()

        # 请求-响应等待（越疆 CR5：一次请求等一次最终响应）
        self.pending_response = None
        self.waiting_for_response = False
        self.response_condition = threading.Condition()

        # 消息序列号（用于确保消息顺序和完整性）
        self.message_sequence = 0
        self.sequence_lock = threading.Lock()

        # 点位数据缓存（用于批量写入）
        self.points_cache = []
        self.cache_lock = threading.Lock()

        # 启动TCP连接
        self._start_tcp_connection()

        # 注册默认消息处理器
        self._register_default_handlers()

        print("机械臂控制器已初始化 - 目标: {}:{}, vendor={}".format(nx_host, nx_port, self.arm_vendor))

    def get_arm_vendor(self):
        """返回当前机械臂厂商"""
        return self.arm_vendor

    def set_current_message_info(self, map_id, point_id, timestamp=''):
        """保存最近示教上下文，供接收线程关联写盘"""
        self.last_sent_message = {
            'mapid': str(map_id),
            'poseid': str(point_id),
            'timestamp': timestamp,
        }
        print("[INFO] 设置当前消息信息: {}".format(self.last_sent_message))

    def is_dobot(self):
        """是否为越疆 CR5 协议"""
        return self.arm_vendor == "dobot"

    def _start_tcp_connection(self):
        """启动TCP连接"""
        try:
            with self.connection_lock:
                if self.tcp_socket:
                    self.tcp_socket.close()

                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp_socket.settimeout(10.0)  # 10秒连接超时

                print("正在连接到 {}:{}...".format(self.nx_host, self.nx_port))
                self.tcp_socket.connect((self.nx_host, self.nx_port))

                self.tcp_connected = True
                print("[SUCCESS] TCP连接成功: {}:{}".format(self.nx_host, self.nx_port))

                # 启动接收线程
                self._start_receive_thread()

                return True

        except Exception as e:
            print("[ERROR] TCP连接失败: {}".format(safe_str(e)))
            self.tcp_connected = False
            return False

    def _start_receive_thread(self):
        """启动接收消息的线程"""
        if self.receive_thread and self.receive_thread.is_alive():
            return

        self.receive_thread = threading.Thread(target=self._receive_messages)
        self.receive_thread.daemon = True  # Python 2.7兼容
        self.receive_thread.start()
        print("[SUCCESS] 消息接收线程已启动")

    def _receive_messages(self):
        """接收TCP消息的线程函数"""
        buffer = b""

        while not self.shutdown_event.is_set() and self.tcp_connected:
            try:
                # 接收数据
                data = self.tcp_socket.recv(1024)
                if not data:
                    print("[WARNING] 连接已关闭")
                    break

                buffer += data

                # 尝试处理消息
                while buffer:
                    # 方法0: 检查是否有待处理的图片数据
                    if self._check_pending_image_data(buffer):
                        # 处理了待处理的图片数据，从缓冲区移除已处理的数据
                        file_size = getattr(self, '_last_processed_file_size', 0)
                        if file_size > 0:
                            buffer = buffer[file_size:]
                            self._last_processed_file_size = 0
                            print("[DEBUG] 已从缓冲区移除 {} bytes 数据，剩余: {} bytes".format(file_size, len(buffer)))
                        continue

                    # 方法1: 检查是否是JSON格式的图片数据（nx当前使用的格式）
                    elif self._is_json_image_data(buffer):
                        # 处理JSON格式的图片数据
                        self._process_json_image_data(buffer)
                        break

                    # 方法2: 检查是否是二进制图片数据包（兼容旧格式）
                    elif self._is_binary_image_packet(buffer):
                        # 解析完整的二进制数据包
                        header_end = buffer.find(b'\n')
                        header_data = buffer[:header_end]
                        header_str = header_data.decode('utf-8')

                        # 尝试解析包头
                        header_info = self._parse_header(header_str)
                        if header_info:
                            file_size = header_info.get('file_size', 0)

                            # 计算完整数据包大小
                            total_packet_size = header_end + 1 + file_size

                            if len(buffer) >= total_packet_size:
                                # 提取完整数据包
                                message_data = buffer[:total_packet_size]
                                buffer = buffer[total_packet_size:]

                                # 解析消息
                                self._parse_message(message_data)
                                continue
                            else:
                                # 数据包不完整，等待更多数据
                                print("[DEBUG] 数据包不完整，等待更多数据。当前: {}, 需要: {}".format(
                                    len(buffer), total_packet_size))
                                break

                    # 方法1.5: 检查是否是拆分发送的图片数据包（只有包头）
                    elif self._is_split_image_packet_header(buffer):
                        # 这是拆分发送的第一个包，只包含包头
                        header_end = buffer.find(b'\n')
                        if header_end != -1:
                            # 提取包头
                            header_data = buffer[:header_end]
                            buffer = buffer[header_end + 1:]

                            # 解析包头
                            header_str = header_data.decode('utf-8')
                            header_json = json.loads(header_str)

                            # 保存包头信息，等待后续数据
                            self._pending_image_header = header_json
                            print("[DEBUG] 收到拆分图片包头: {}".format(header_json))
                            continue

                    # 方法1.6: 检查是否有待处理的图片包头
                    elif hasattr(self, '_pending_image_header') and self._pending_image_header:
                        file_size = self._pending_image_header.get('file_size', 0)
                        if len(buffer) >= file_size:
                            # 提取完整的图片数据
                            image_data = buffer[:file_size]
                            buffer = buffer[file_size:]

                            # 重新构建包头数据
                            header_json_str = json.dumps(self._pending_image_header)
                            header_data = header_json_str.encode('utf-8')

                            # 创建完整的消息
                            message_data = {
                                'type': self._pending_image_header.get('type', 'unknown'),
                                'tcp_packet': header_data + b'\n' + image_data,
                                'header': self._pending_image_header,
                                'body_data': image_data,
                                'file_size': file_size
                            }

                            # 清除待处理的包头
                            self._pending_image_header = None

                            # 解析消息
                            self._parse_message(message_data)
                            continue
                        else:
                            # 图片数据不完整，等待更多数据
                            print("[DEBUG] 图片数据不完整，等待更多数据。当前: {}, 需要: {}".format(
                                len(buffer), file_size))
                            break

                    # 方法3: 尝试解析带长度前缀的消息
                    if len(buffer) >= 4:
                        try:
                            msg_length = struct.unpack('!I', buffer[:4])[0]
                            if len(buffer) >= msg_length + 4:
                                # 提取消息内容
                                message_data = buffer[4:msg_length + 4]
                                buffer = buffer[msg_length + 4:]

                                # 解析消息
                                self._parse_message(message_data)
                                continue
                        except:
                            pass  # 不是带长度前缀的消息，继续尝试其他方法

                    # 方法4: 尝试解析纯文本消息（以换行符分隔）
                    if b'\n' in buffer:
                        line_end = buffer.find(b'\n')
                        message_data = buffer[:line_end]
                        buffer = buffer[line_end + 1:]

                        # 检查是否是有效的文本消息（不是二进制数据）
                        if self._is_valid_text_message(message_data):
                            # 解析消息
                            self._parse_message(message_data)
                        else:
                            print("[DEBUG] 跳过无效的文本消息，数据长度: {}".format(len(message_data)))
                        continue

                    # 方法5: 尝试解析JSON消息（无分隔符）
                    if len(buffer) > 0:
                        try:
                            # 尝试解析整个缓冲区作为JSON
                            message_str = buffer.decode('utf-8')
                            json.loads(message_str)  # 验证是否为有效JSON

                            # 如果是有效JSON，处理它
                            self._parse_message(buffer)
                            buffer = b""  # 清空缓冲区
                            continue
                        except:
                            pass  # 不是有效JSON，继续等待更多数据

                    # 如果所有方法都失败，检查是否是无效的小数据包
                    if len(buffer) < 50:  # 如果缓冲区数据很小
                        print("[DEBUG] 清空无效的小数据包，大小: {} bytes".format(len(buffer)))
                        buffer = b""  # 清空缓冲区
                        break
                    else:
                        # 等待更多数据
                        break

            except socket.timeout:
                continue
            except Exception as e:
                print("[ERROR] 接收消息时发生错误: {}".format(safe_str(e)))
                break

        print("消息接收线程已停止")
        self.tcp_connected = False

    def _is_binary_image_packet(self, data):
        """
        判断是否是二进制图片数据包
        格式：包头（描述信息）+ 包长（文件大小）+ 包体（二进制数据）

        Args:
            data: 接收到的数据

        Returns:
            bool: 是否是二进制图片数据包
        """
        try:
            # 检查数据长度，如果太短可能不是完整的数据包
            if len(data) < 50:  # 至少需要包头 + 一些数据
                return False

            # 检查是否包含包头分隔符
            if b'\n' not in data:
                return False

            # 尝试解析包头
            header_end = data.find(b'\n')
            header_data = data[:header_end]

            # 检查包头长度是否合理
            if len(header_data) > 1000:  # 包头不应该超过1KB
                return False

            try:
                header_str = header_data.decode('utf-8')

                # 尝试解析为JSON
                try:
                    header_json = json.loads(header_str)

                    # 检查是否包含图片相关字段
                    if 'type' in header_json and header_json['type'] in ['result', 'raw']:
                        if 'file_size' in header_json:
                            file_size = header_json.get('file_size', 0)
                            # 检查文件大小是否合理
                            if 0 < file_size < 50 * 1024 * 1024:  # 0到50MB之间
                                # 检查总数据长度是否匹配
                                expected_total = header_end + 1 + file_size
                                if len(data) >= expected_total:
                                    print("[DEBUG] 检测到JSON格式二进制图片数据包: type={}, size={}".format(
                                        header_json['type'], file_size))
                                    return True
                except json.JSONDecodeError:
                    # 如果不是JSON，尝试解析为键值对格式
                    if self._is_key_value_header(header_str):
                        print("[DEBUG] 检测到键值对格式二进制图片数据包")
                        return True

            except Exception as e:
                # 如果解码失败，可能是普通二进制数据
                pass

            return False

        except Exception as e:
            print("[DEBUG] 检查二进制数据包时发生错误: {}".format(safe_str(e)))
            return False

    def _is_key_value_header(self, header_str):
        """
        检查是否是键值对格式的包头
        例如：type: raw\nmapid: 1\nposeid: 2\nfilename: image.jpg\nfilesize: 12345

        Args:
            header_str: 包头字符串

        Returns:
            bool: 是否是键值对格式
        """
        try:
            # 检查是否包含图片相关字段
            if 'type:' in header_str and ('raw' in header_str or 'result' in header_str):
                if 'filesize:' in header_str or 'file_size:' in header_str:
                    print("[DEBUG] 检测到键值对格式包头: {}".format(header_str[:100]))
                    return True
            return False
        except:
            return False

    def _parse_header(self, header_str):
        """
        解析包头信息，支持JSON和键值对格式

        Args:
            header_str: 包头字符串

        Returns:
            dict: 解析后的包头信息，失败返回None
        """
        try:
            # 尝试解析为JSON
            try:
                header_json = json.loads(header_str)
                if 'type' in header_json and header_json['type'] in ['result', 'raw']:
                    return header_json
            except json.JSONDecodeError:
                pass

            # 尝试解析为键值对格式
            header_info = {}
            lines = header_str.strip().split('\n')

            for line in lines:
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()

                    # 处理不同的字段名
                    if key == 'type':
                        header_info['type'] = value
                    elif key in ['filesize', 'file_size']:
                        try:
                            header_info['file_size'] = int(value)
                        except ValueError:
                            pass
                    elif key == 'mapid':
                        header_info['map_id'] = value
                    elif key == 'poseid':
                        header_info['pose_id'] = value
                    elif key == 'filename':
                        header_info['file_name'] = value
                    elif key == 'file_extension':
                        header_info['file_extension'] = value

            # 检查是否包含必要的字段
            if 'type' in header_info and header_info['type'] in ['result', 'raw']:
                if 'file_size' in header_info:
                    print("[DEBUG] 解析键值对包头成功: {}".format(header_info))
                    return header_info

            return None

        except Exception as e:
            print("[ERROR] 解析包头时发生错误: {}".format(safe_str(e)))
            return None

    def _is_json_image_data(self, data):
        """
        检查是否是JSON格式的图片数据
        格式：JSON元数据 + 换行符 + 二进制图片数据

        Args:
            data: 接收到的数据

        Returns:
            bool: 是否是JSON格式的图片数据
        """
        try:
            print("[DEBUG] 检查JSON图片数据，数据长度: {}".format(len(data)))
            print("[DEBUG] 数据前100字节: {}".format(data[:100]))

            # 检查是否包含换行符
            if b'\n' not in data:
                print("[DEBUG] 未找到换行符，不是JSON格式")
                return False

            # 尝试解析JSON部分
            first_line_end = data.find(b'\n')
            json_str = data[:first_line_end].decode('utf-8', errors='ignore')
            print("[DEBUG] JSON字符串: {}".format(json_str))

            # 尝试解析为JSON
            try:
                import json
                json_data = json.loads(json_str)
                print("[DEBUG] 解析的JSON数据: {}".format(json_data))

                # 首先检查是否是机械臂状态消息，如果是则不是图片数据
                if 'status' in json_data and 'mapid' in json_data and 'poseid' in json_data:
                    print("[DEBUG] 这是机械臂状态消息，不是图片数据")
                    return False

                # 检查是否包含图片相关字段
                if 'type' in json_data and json_data['type'] in ['raw', 'result']:
                    if 'filesize' in json_data or 'file_size' in json_data:
                        print("[DEBUG] 检测到JSON格式的图片数据: {}".format(json_str))
                        return True
                    else:
                        print("[DEBUG] JSON缺少filesize字段")
                else:
                    print("[DEBUG] JSON缺少type字段或type不正确")
            except json.JSONDecodeError as e:
                print("[DEBUG] JSON解析失败: {}".format(e))

            return False

        except Exception as e:
            print("[DEBUG] 检查JSON图片数据时发生错误: {}".format(safe_str(e)))
            return False

    def _process_json_image_data(self, data):
        """
        处理JSON格式的图片数据

        Args:
            data: 接收到的数据
        """
        try:
            print("[JSON_IMAGE] 开始处理JSON格式的图片数据")

            # 解析JSON部分
            first_line_end = data.find(b'\n')
            json_str = data[:first_line_end].decode('utf-8', errors='ignore')

            import json
            json_data = json.loads(json_str)

            # 转换为统一的图片信息格式
            image_info = {
                'type': json_data.get('type', 'unknown'),
                'file_size': json_data.get('filesize', json_data.get('file_size', 0)),
                'map_id': json_data.get('mapid', 'default_map'),
                'pose_id': json_data.get('poseid', 'default_pose'),
                'file_name': json_data.get('filename', 'image.png')
            }

            print("[JSON_IMAGE] 解析的图片信息: {}".format(image_info))

            # 检查是否有必要的字段
            if 'type' in image_info and image_info['type'] in ['result', 'raw']:
                if 'file_size' in image_info and image_info['file_size'] > 0:
                    # 保存图片信息，等待后续的二进制数据
                    self._pending_image_info = image_info
                    print("[JSON_IMAGE] 图片信息已保存，等待二进制数据")
                else:
                    print("[WARNING] 缺少文件大小信息")
            else:
                print("[WARNING] 缺少类型信息或类型不正确")

        except Exception as e:
            print("[ERROR] 处理JSON图片数据时发生错误: {}".format(safe_str(e)))

    def _check_pending_image_data(self, buffer):
        """
        检查是否有待处理的图片数据

        Args:
            buffer: 接收缓冲区

        Returns:
            bool: 是否处理了待处理的图片数据
        """
        try:
            if not hasattr(self, '_pending_image_info') or not self._pending_image_info:
                return False

            file_size = self._pending_image_info.get('file_size', 0)
            print("[DEBUG] 检查待处理图片数据，需要大小: {}, 当前缓冲区大小: {}".format(file_size, len(buffer)))

            # 必须先定位到上一条图片JSON头部的结束位置
            json_end = buffer.find(b'\n')
            if json_end == -1:
                print("[DEBUG] 未找到JSON结束标记")
                return False

            start_pos = json_end + 1
            print("[DEBUG] JSON结束位置: {}, 图片数据开始位置: {}".format(json_end, start_pos))

            # 若缓冲区长度不足以包含完整图片体，则等待更多数据
            if len(buffer) < start_pos + file_size:
                print("[DEBUG] 数据不足，等待更多数据。当前: {}，需要: {}".format(len(buffer), start_pos + file_size))
                return False

            # 防止把后续JSON/文本消息误认为图片体：检测开始位置的若干字节是否像文本
            lookahead_end = min(len(buffer), start_pos + 64)
            lookahead = buffer[start_pos:lookahead_end]
            try:
                lookahead_text = lookahead.decode('utf-8')
                # 规则：若以'{'或'['起始，或可打印比例高且很快出现换行，判定为文本/JSON
                printable_ratio = sum(1 for c in lookahead_text if (c.isprintable() if hasattr(c, 'isprintable') else ord(c) < 128)) / float(max(1, len(lookahead_text)))
                if (lookahead_text.lstrip().startswith('{') or lookahead_text.lstrip().startswith('[') or
                    (printable_ratio > 0.8 and b'\n' in lookahead)):
                    print("[DEBUG] 后续数据疑似文本/JSON，暂不按图片体消费，保留pending状态")
                    return False
            except Exception:
                # 解码失败通常意味着是二进制，继续当作图片体处理
                pass

            # 从二进制数据开始位置提取图片数据
            image_data = buffer[start_pos:start_pos + file_size]
            print("[DEBUG] 跳过JSON信息，从位置 {} 开始提取图片数据".format(start_pos))
            print("[DEBUG] 提取的数据前20字节: {}".format(image_data[:20]))
            print("[DEBUG] 提取的数据类型: {}".format(type(image_data)))

            # 确保image_data是字节类型
            if isinstance(image_data, str):
                print("[DEBUG] 提取的数据是字符串类型，在Python 2.x中这是正常的字节类型")
                # 在Python 2.x中，str就是字节类型，不需要转换
                # 在Python 3.x中，需要转换为bytes
                try:
                    # 尝试检查是否是有效的字节数据
                    if len(image_data) > 0 and ord(image_data[0]) > 127:
                        print("[DEBUG] 数据包含非ASCII字节，这是有效的二进制数据")
                except:
                    pass
            elif not isinstance(image_data, bytes):
                print("[WARNING] 提取的数据不是字节类型，尝试转换")
                image_data = bytes(image_data)

            # 创建完整的消息
            message_data = {
                'type': self._pending_image_info.get('type', 'unknown'),
                'tcp_packet': image_data,
                'header': self._pending_image_info,
                'body_data': image_data,
                'file_size': file_size
            }

            print("[DEBUG] 创建的消息对象: {}".format({
                'type': message_data['type'],
                'has_tcp_packet': 'tcp_packet' in message_data,
                'has_body_data': 'body_data' in message_data,
                'header_keys': list(message_data['header'].keys()) if 'header' in message_data else []
            }))

            print("[DEBUG] 开始处理图片消息: type={}, size={}".format(
                self._pending_image_info.get('type'), file_size))

            # 记录已处理的数据大小（包括跳过的JSON信息）
            self._last_processed_file_size = start_pos + file_size

            # 清除待处理的图片信息
            self._pending_image_info = None

            # 处理图片消息
            print("[DEBUG] 准备调用_handle_image_message")
            self._handle_image_message(message_data)
            print("[DEBUG] _handle_image_message调用完成")
            return True

            return False

        except Exception as e:
            print("[ERROR] 检查待处理图片数据时发生错误: {}".format(safe_str(e)))
            return False

    def _is_valid_text_message(self, data):
        """
        检查是否是有效的文本消息（不是二进制数据）

        Args:
            data: 要检查的数据

        Returns:
            bool: 是否是有效的文本消息
        """
        try:
            # 如果数据太短，可能是二进制数据的片段
            if len(data) < 10:
                return False

            # 尝试解码为UTF-8
            try:
                text = data.decode('utf-8')
            except UnicodeDecodeError:
                # 如果无法解码为UTF-8，可能是二进制数据
                return False

            # 检查是否包含可打印字符（兼容不同Python版本）
            try:
                printable_chars = sum(1 for c in text if c.isprintable())
            except AttributeError:
                # 兼容Python 2或某些unicode处理方式
                import string
                printable_chars = sum(1 for c in text if c in string.printable)

            if printable_chars < len(text) * 0.8:  # 至少80%是可打印字符
                return False

            # 检查是否包含常见的文本消息特征
            if any(keyword in text.lower() for keyword in ['type:', 'status:', 'command:', 'response:', 'error:']):
                return True

            # 检查是否是JSON格式
            try:
                json.loads(text)
                return True
            except:
                pass

            # 如果包含大量非ASCII字符，可能是二进制数据
            non_ascii_chars = sum(1 for c in text if ord(c) > 127)
            if non_ascii_chars > len(text) * 0.3:  # 超过30%是非ASCII字符
                return False

            return True

        except Exception as e:
            print("[DEBUG] 检查文本消息时发生错误: {}".format(safe_str(e)))
            return False

    def _is_split_image_packet_header(self, data):
        """
        判断是否是拆分发送的图片数据包头（只有包头，没有数据）

        Args:
            data: 接收到的数据

        Returns:
            bool: 是否是拆分发送的图片包头
        """
        try:
            # 检查是否包含JSON包头
            if b'\n' not in data:
                return False

            # 尝试解析包头
            header_end = data.find(b'\n')
            header_data = data[:header_end]

            # 检查包头长度是否合理
            if len(header_data) > 1000:
                return False

            try:
                header_str = header_data.decode('utf-8')
                header_json = json.loads(header_str)

                # 检查是否包含图片相关字段
                if 'type' in header_json and header_json['type'] in ['result', 'raw']:
                    if 'file_size' in header_json:
                        file_size = header_json.get('file_size', 0)
                        # 检查文件大小是否合理
                        if 0 < file_size < 50 * 1024 * 1024:  # 0到50MB之间
                            # 检查是否只有包头（没有后续数据）
                            if len(data) == header_end + 1:  # 只有包头 + 换行符
                                print("[DEBUG] 检测到拆分图片包头: type={}, size={}".format(
                                    header_json['type'], file_size))
                                return True
            except Exception as e:
                # 如果JSON解析失败，不是图片包头
                pass

            return False

        except Exception as e:
            print("[DEBUG] 检查拆分图片包头时发生错误: {}".format(safe_str(e)))
            return False

    def _process_binary_image_packet(self, data):
        """
        处理二进制图片数据包

        Args:
            data: 二进制数据包
        """
        try:
            print("[BINARY_PACKET] 开始处理二进制图片数据包")

            # 解析包头
            header_end = data.find(b'\n')
            header_data = data[:header_end]
            header_str = header_data.decode('utf-8')
            header_json = json.loads(header_str)

            print("[BINARY_PACKET] 包头信息: {}".format(header_json))

            # 获取文件大小
            file_size = header_json.get('file_size', 0)
            print("[BINARY_PACKET] 文件大小: {} bytes".format(file_size))

            # 获取包体数据
            body_start = header_end + 1
            body_data = data[body_start:body_start + file_size]

            print("[BINARY_PACKET] 包体数据长度: {} bytes".format(len(body_data)))

            # 创建消息对象
            message = {
                'type': header_json.get('type', 'unknown'),
                'tcp_packet': data,  # 保存完整的数据包
                'header': header_json,
                'body_data': body_data,
                'file_size': file_size
            }

            # 调用图片消息处理器
            self._handle_image_message(message)

        except Exception as e:
            print("[ERROR] 处理二进制图片数据包时发生错误: {}".format(safe_str(e)))

    def _parse_message(self, message_data):
        """解析接收到的消息"""
        try:
            # 首先检查是否是二进制图片数据包
            if self._is_binary_image_packet(message_data):
                print("[RECEIVE] 识别为二进制图片数据包")
                self._process_binary_image_packet(message_data)
                return

            # 转换为字符串，处理编码问题
            if hasattr(message_data, 'decode'):
                try:
                    message_str = message_data.decode('utf-8').strip()
                except UnicodeDecodeError:
                    # 如果UTF-8解码失败，尝试其他编码
                    try:
                        message_str = message_data.decode('gbk').strip()
                    except UnicodeDecodeError:
                        message_str = message_data.decode('latin-1').strip()
            else:
                message_str = str(message_data).strip()

            # 安全地打印消息（处理中文编码）
            try:
                print("[RECEIVE] 收到原始消息: {}".format(message_str))
            except UnicodeEncodeError:
                print("[RECEIVE] 收到原始消息: {}".format(repr(message_str)))

            # 尝试解析JSON消息
            try:
                message = json.loads(message_str)
                print("[RECEIVE] 解析为JSON: {}".format(message))

                # 检查解析后的数据类型
                if isinstance(message, list):
                    print("[INFO] 收到JSON数组，包含 {} 个元素".format(len(message)))
                    # 如果是数组，检查是否是示教点位数据
                    if len(message) > 0 and isinstance(message[0], dict) and 'label' in message[0] and 'joints' in message[0]:
                        print("[INFO] 识别为示教点位数据数组，直接处理")
                        self._handle_mechanical_arm_points(message)
                    else:
                        # 如果是其他类型的数组，处理每个元素
                        for i, item in enumerate(message):
                            if isinstance(item, dict):
                                # 美化显示，去除u'前缀
                                clean_item = self._clean_unicode_display(item)
                                print("[INFO] 处理数组元素 {}: {}".format(i, clean_item))
                                # 递归处理每个字典元素
                                self._process_json_message(item)
                            else:
                                print("[INFO] 数组元素 {} 不是字典: {}".format(i, type(item)))
                elif isinstance(message, dict):
                    # 如果是字典，正常处理
                    self._process_json_message(message)
                else:
                    print("[WARNING] 未知的JSON数据类型: {}".format(type(message)))

            except (ValueError, TypeError) as json_error:
                print("[ERROR] JSON解析失败: {}".format(str(json_error)))
                # 不是JSON，作为纯文本消息处理
                try:
                    print("[RECEIVE] 解析为纯文本: {}".format(message_str))
                except UnicodeEncodeError:
                    print("[RECEIVE] 解析为纯文本: {}".format(repr(message_str)))

                # 注册一个通用的文本消息处理器
                if 'text' in self.message_handlers:
                    try:
                        self.message_handlers['text'](message_str)
                    except Exception as e:
                        print("[ERROR] 处理文本消息时发生错误: {}".format(safe_str(e)))
                else:
                    try:
                        print("[INFO] 收到文本消息: {}".format(message_str))
                    except UnicodeEncodeError:
                        print("[INFO] 收到文本消息: {}".format(repr(message_str)))

        except Exception as e:
            print("[ERROR] 解析消息时发生错误: {}".format(safe_str(e)))

    def _clean_unicode_display(self, data):
        """
        清理Unicode字符串显示，去除u'前缀

        Args:
            data: 要清理的数据（字典或列表）

        Returns:
            清理后的数据
        """
        # 简化处理，直接返回原始数据
        # u'前缀只是Python内部表示，不影响实际使用
        return data

    def _process_json_message(self, message):
        """
        处理解析后的JSON消息

        Args:
            message: 解析后的JSON消息（字典）
        """
        try:
            # 若有等待中的请求，先投递最终响应（CR5 以 status 为准）
            if isinstance(message, dict) and 'status' in message:
                self._notify_pending_response(message)

            # 检查是否是机械臂点位数据
            if self._is_mechanical_arm_point_data(message):
                print("[INFO] 识别为机械臂点位数据")
                self._handle_mechanical_arm_points(message)
                return

            # 优先检查状态消息（如status: "done"）
            if 'status' in message:
                print("[INFO] 识别为状态消息: {}".format(message))
                self._handle_status_message(message)
                return

            # 根据消息类型调用对应的处理器
            msg_type = message.get('type', 'unknown')
            if msg_type in self.message_handlers:
                try:
                    self.message_handlers[msg_type](message)
                except Exception as e:
                    print("[ERROR] 处理消息时发生错误: {}".format(safe_str(e)))
            else:
                print("[WARNING] 未知消息类型: {}".format(msg_type))
                # 如果没有特定的处理器，尝试通用处理
                self._handle_generic_message(message)
        except Exception as e:
            print("[ERROR] 处理JSON消息时发生错误: {}".format(safe_str(e)))

    def _notify_pending_response(self, message):
        """将带 status 的最终响应通知等待线程"""
        try:
            with self.response_condition:
                if self.waiting_for_response:
                    self.pending_response = message
                    self.waiting_for_response = False
                    self.response_condition.notify_all()
                    print("[RESPONSE] 已投递最终响应: status={}".format(message.get('status')))
        except Exception as e:
            print("[ERROR] 通知等待响应失败: {}".format(safe_str(e)))

    def _wait_for_request_response(self, timeout=70):
        """
        等待一次请求的最终响应

        Returns:
            dict|None: NX 响应，超时返回 None
        """
        with self.response_condition:
            self.response_condition.wait(timeout)
            response = self.pending_response
            self.pending_response = None
            self.waiting_for_response = False
            return response

    def _is_success_status(self, status):
        """判断响应 status 是否成功"""
        if status is None:
            return False
        status = str(status).lower()
        return status in ('succeeded', 'success', 'done', 'ok')

    def _is_failure_status(self, status):
        """判断响应 status 是否失败"""
        if status is None:
            return False
        status = str(status).lower()
        return status in ('failed', 'timeout', 'cancelled', 'error')

    def _is_mechanical_arm_point_data(self, message):
        """
        判断是否是机械臂点位数据

        Args:
            message: 消息数据（可能是字典或数组）

        Returns:
            bool: 是否是机械臂点位数据
        """
        # 检查是否是数组格式的示教点位数据
        if isinstance(message, list) and len(message) > 0:
            first_item = message[0]
            if isinstance(first_item, dict) and 'label' in first_item and 'joints' in first_item:
                print("[INFO] 检测到数组格式的示教点位数据")
                return True

        # 检查是否是单个对象的机械臂点位数据
        if isinstance(message, dict):
            # 检查是否包含机械臂点位数据的特征字段
            required_fields = ['name', 'joint', 'coordinate']
            optional_fields = ['armorientation', 'armOrientation', 'tool', 'alias', 'user', 'id']

            # 检查必需字段
            has_required = all(field in message for field in required_fields)

            # 检查可选字段（至少有一个）
            has_optional = any(field in message for field in optional_fields)

            return has_required and (has_optional or len(message) >= 3)

        return False

    def _handle_generic_message(self, message):
        """
        处理通用消息（没有特定类型的消息）

        Args:
            message: 消息字典
        """
        try:
            print("[GENERIC] 收到通用消息: {}".format(message))
            # 可以在这里添加通用的消息处理逻辑
        except Exception as e:
            print("[ERROR] 处理通用消息时发生错误: {}".format(safe_str(e)))

    def _register_default_handlers(self):
        """注册默认的消息处理器"""
        self.message_handlers.update({
            'status': self._handle_status_message,
            'command': self._handle_command_message,
            'response': self._handle_response_message,
            'error': self._handle_error_message,
            'text': self._handle_text_message,
            'mechanical_arm_points': self._handle_mechanical_arm_points,
            'arm_points': self._handle_mechanical_arm_points,
            'points': self._handle_mechanical_arm_points,
            'result': self._handle_image_message,  # 处理result类型图片消息
            'raw': self._handle_image_message     # 处理raw类型图片消息
        })

    def _handle_status_message(self, message):
        """处理状态消息"""
        print("[STATUS] 收到状态消息: {}".format(message))

        # 检查是否是机械臂完成/失败终态信号
        status_value = message.get('status')
        print("[STATUS] 状态值: '{}'".format(status_value))

        if status_value in ('done', 'failed', 'timeout', 'cancelled'):
            print("[MECHANICAL_ARM] 收到机械臂终态信号: {}".format(message))
            self._handle_mechanical_arm_completion(message)
        else:
            print("[STATUS] 非终态状态: {}".format(status_value))

    def _handle_command_message(self, message):
        """处理命令消息"""
        print("[COMMAND] 收到命令消息: {}".format(message))

    def _handle_response_message(self, message):
        """处理响应消息"""
        print("[RESPONSE] 收到响应消息: {}".format(message))

    def _handle_error_message(self, message):
        """处理错误消息"""
        print("[ERROR] 收到错误消息: {}".format(message))

    def _handle_text_message(self, message):
        """处理文本消息"""
        try:
            print("[TEXT] 收到文本消息: {}".format(message))
        except UnicodeEncodeError:
            print("[TEXT] 收到文本消息: {}".format(repr(message)))

    def _handle_image_message(self, message):
        """
        处理图片消息（type为result或raw）
        支持TCP二进制数据包格式：包头（描述信息）+ 包长（文件大小）+ 包体（二进制数据）

        Args:
            message: 包含图片信息的消息
        """
        try:
            print("[IMAGE] ===== 开始处理图片消息 =====")
            print("[IMAGE] 收到图片消息: {}".format(message))

            # 获取消息类型（result或raw）
            msg_type = message.get('type', 'unknown')
            print("[IMAGE] 图片类型: {}".format(msg_type))

            # 获取当前时间，用于创建日期文件夹
            from datetime import datetime
            current_date = datetime.now().strftime("%Y-%m-%d")
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            # 获取地图和点位信息（从消息中获取）
            header = message.get('header', {})
            map_id = header.get('map_id', getattr(self, 'current_map_id', 'default_map'))
            pose_id = header.get('pose_id', getattr(self, 'current_pose_id', 'default_pose'))

            print("[IMAGE] 当前地图ID: {}, 点位ID: {}".format(map_id, pose_id))
            print("[IMAGE] 地图ID类型: {}, 长度: {}".format(type(map_id), len(str(map_id))))
            print("[IMAGE] 点位ID类型: {}, 长度: {}".format(type(pose_id), len(str(pose_id))))

            # 构建保存路径：/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node/mapid/poseid/type/日期/
            import os
            base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"

            # 使用现有的mapid和poseid文件夹（示教时已创建）
            map_folder = os.path.join(base_path, map_id)
            pose_folder = os.path.join(map_folder, pose_id)

            # 创建type文件夹路径（raw或result）
            type_folder = os.path.join(pose_folder, msg_type)

            # 创建日期文件夹路径
            date_folder = os.path.join(type_folder, current_date)

            # 最终保存路径
            type_folder = date_folder

            # 确保目录存在
            try:
                os.makedirs(type_folder)
            except OSError:
                # 目录已存在，忽略错误
                pass
            print("[IMAGE] 图片保存路径: {}".format(type_folder))

            # 处理TCP二进制数据包
            print("[IMAGE] 检查消息字段: {}".format(list(message.keys())))

            if 'body_data' in message:
                print("[IMAGE] 使用body_data处理图片数据")
                image_data = message.get('body_data')
                file_name = message.get('header', {}).get('file_name', 'image_{}.jpg'.format(int(time.time())))

                print("[IMAGE] 文件名: {}, 数据大小: {} bytes".format(file_name, len(image_data)))

                # 检查数据开头几个字节
                if len(image_data) >= 8:
                    header_bytes = image_data[:8]
                    print("[IMAGE] 数据开头8字节: {}".format(header_bytes))
                    print("[IMAGE] 数据类型: {}".format(type(image_data)))
                    if isinstance(image_data, bytes):
                        print("[IMAGE] 数据开头8字节(hex): {}".format(header_bytes.encode('hex')))
                        if header_bytes.startswith(b'\x89PNG\r\n\x1a\n'):
                            print("[IMAGE]  数据开头是有效的PNG文件头")
                        else:
                            print("[IMAGE]  数据开头不是PNG文件头")
                    elif isinstance(image_data, str):
                        print("[IMAGE] 数据是字符串类型，在Python 2.x中这是正常的字节类型")
                        # 在Python 2.x中，str就是字节类型，直接检查PNG文件头
                        if header_bytes.startswith(b'\x89PNG\r\n\x1a\n'):
                            print("[IMAGE]  数据开头是有效的PNG文件头")
                        else:
                            print("[IMAGE]  数据开头不是PNG文件头")
                    else:
                        print("[IMAGE]  数据不是字节或字符串类型，无法检查PNG格式")

                # 保存图片文件
                image_path = os.path.join(type_folder, file_name)
                with open(image_path, 'wb') as f:
                    if isinstance(image_data, bytes):
                        f.write(image_data)
                    elif isinstance(image_data, str):
                        # 在Python 2.x中，str就是字节类型，直接写入
                        f.write(image_data)
                    else:
                        # 其他类型，尝试转换
                        f.write(str(image_data))

                print("[IMAGE] 图片已保存: {}".format(image_path))
                print("[IMAGE] 文件大小: {} bytes".format(len(image_data)))

                # 验证保存的文件
                print("[IMAGE] 检查文件路径: {}".format(image_path))
                if os.path.exists(image_path):
                    actual_size = os.path.getsize(image_path)
                    print("[IMAGE] 实际文件大小: {} bytes".format(actual_size))

                    # 验证保存的图片文件
                    print("[IMAGE] 图片文件保存成功")
                    print("[IMAGE] 文件大小: {} bytes".format(actual_size))
                else:
                    print("[IMAGE]  文件保存失败")


            elif 'image_data' in message:
                # 如果消息中包含图片数据（兼容旧格式）
                image_data = message.get('image_data')
                image_name = message.get('image_name', 'image_{}.jpg'.format(int(time.time())))

                # 保存图片文件
                image_path = os.path.join(type_folder, image_name)
                with open(image_path, 'wb') as f:
                    f.write(image_data)

                print("[IMAGE] 图片已保存: {}".format(image_path))

            elif 'image_path' in message:
                # 如果消息中包含图片路径
                source_path = message.get('image_path')
                image_name = os.path.basename(source_path)
                target_path = os.path.join(type_folder, image_name)

                # 复制图片文件
                import shutil
                shutil.copy2(source_path, target_path)

                print("[IMAGE] 图片已复制: {} -> {}".format(source_path, target_path))

            else:
                print("[WARNING] 图片消息中未找到图片数据或路径")
                print("[WARNING] 消息字段: {}".format(list(message.keys())))

            # 记录图片信息到日志
            log_info = {
                'timestamp': current_time,
                'type': msg_type,
                'map_id': map_id,
                'pose_id': pose_id,
                'date_folder': current_date,
                'type_folder': msg_type,
                'message': message
            }

            print("[IMAGE] 图片处理完成: {}".format(log_info))
            print("[IMAGE] ===== 图片消息处理完成 =====")

        except Exception as e:
            print("[ERROR] 处理图片消息时发生错误: {}".format(safe_str(e)))
            print("[ERROR] ===== 图片消息处理失败 =====")

    def _process_tcp_image_packet(self, message, type_folder, msg_type):
        """
        处理TCP二进制数据包格式的图片数据
        格式：包头（描述信息）+ 包长（文件大小）+ 包体（二进制数据）

        Args:
            message: 包含TCP数据包的消息
            type_folder: 保存文件夹路径
            msg_type: 图片类型（result或raw）
        """
        try:
            tcp_packet = message.get('tcp_packet')
            if not tcp_packet:
                print("[ERROR] TCP数据包为空")
                return

            print("[TCP_PACKET] 开始解析TCP数据包，数据长度: {}".format(len(tcp_packet)))

            # 解析包头（描述信息）
            # 假设包头是JSON格式的描述信息
            header_end = tcp_packet.find(b'\n')
            if header_end == -1:
                print("[ERROR] 未找到包头结束标志")
                return

            header_data = tcp_packet[:header_end].decode('utf-8')
            print("[TCP_PACKET] 包头信息: {}".format(header_data))

            # 解析包头JSON
            import json
            try:
                header_info = json.loads(header_data)
                print("[TCP_PACKET] 解析包头成功: {}".format(header_info))
            except json.JSONDecodeError as e:
                print("[ERROR] 包头JSON解析失败: {}".format(e))
                return

            # 获取包长（文件大小）
            packet_length = header_info.get('file_size', 0)
            print("[TCP_PACKET] 文件大小: {} bytes".format(packet_length))

            # 获取包体（二进制数据）
            body_start = header_end + 1
            body_data = tcp_packet[body_start:body_start + packet_length]

            print("[TCP_PACKET] 包体数据长度: {}".format(len(body_data)))

            # 验证数据完整性
            if len(body_data) != packet_length:
                print("[WARNING] 数据长度不匹配，期望: {}, 实际: {}".format(packet_length, len(body_data)))

            # 生成文件名
            timestamp = int(time.time())
            file_extension = header_info.get('file_extension', '.jpg')
            image_name = "image_{}_{}.{}".format(msg_type, timestamp, file_extension.lstrip('.'))

            # 保存图片文件
            image_path = os.path.join(type_folder, image_name)
            with open(image_path, 'wb') as f:
                f.write(body_data)

            print("[TCP_PACKET] 图片已保存: {}".format(image_path))
            print("[TCP_PACKET] 文件大小: {} bytes".format(len(body_data)))

            # 记录详细信息
            packet_info = {
                'header': header_info,
                'file_size': packet_length,
                'actual_size': len(body_data),
                'saved_path': image_path,
                'timestamp': timestamp
            }

            print("[TCP_PACKET] 数据包处理完成: {}".format(packet_info))

        except Exception as e:
            print("[ERROR] 处理TCP数据包时发生错误: {}".format(safe_str(e)))

    def _handle_mechanical_arm_completion(self, message):
        """
        处理机械臂完成信号

        Args:
            message: 完成信号消息，包含status="done"/failed/timeout/cancelled
        """
        try:
            print("[MECHANICAL_ARM] 处理机械臂完成信号: {}".format(message))

            with self.completion_condition:
                self.last_completion_response = message
                # 仅 status=done 视为成功；失败终态也要唤醒等待线程
                self.mechanical_arm_completed = message.get('status') == 'done'
                print("[MECHANICAL_ARM] 机械臂终态: {}, success={}".format(
                    message.get('status'), self.mechanical_arm_completed))
                # 通知所有等待的线程
                self.completion_condition.notify_all()
                print("[MECHANICAL_ARM] 已通知所有等待线程")

        except Exception as e:
            print("[ERROR] 处理机械臂完成信号时发生错误: {}".format(safe_str(e)))

    def _handle_mechanical_arm_points(self, message):
        """处理机械臂点位数据消息"""
        try:
            print("[ARM_POINTS] 收到机械臂点位数据:")

            # 检查是否是数组格式的示教数据
            if isinstance(message, list):
                print("[ARM_POINTS] 检测到数组格式的示教数据，包含 {} 个点位".format(len(message)))

                # 处理数组中的每个点位
                for i, point in enumerate(message):
                    if isinstance(point, dict) and 'label' in point and 'joints' in point:
                        print("  - 点位 {}: 标签={}, 关节数据={}".format(i, point.get('label'), point.get('joints')))
                        # 将每个点位添加到缓存（禁用自动写入）
                        self._add_point_to_cache(point, auto_write=False)

                # 数组处理完成后，检查是否需要写入
                print("[ARM_POINTS] 数组处理完成，缓存数量: {}".format(len(self.points_cache)))
                # 如果缓存中有数据，触发写入
                if len(self.points_cache) > 0:
                    print("[ARM_POINTS] 触发批量写入，缓存内容: {}".format(self.points_cache))
                    self._write_all_points_to_folder()
                else:
                    print("[ARM_POINTS] 缓存为空，跳过写入")
                return

            # 处理单个点位数据（原有逻辑）
            # 清理显示数据
            clean_message = self._clean_unicode_display(message)

            # 处理点位数据
            if 'name' in message:
                point_name = message.get('name', '')
                print("  - 点位名称: {}".format(point_name))

            if 'joint' in message:
                joint_data = message.get('joint', [])
                print("  - 关节数据: {}".format(joint_data))

            if 'coordinate' in message:
                coordinate_data = message.get('coordinate', [])
                print("  - 坐标数据: {}".format(coordinate_data))

            # 检查不同的姿态字段名
            orientation_data = None
            if 'armorientation' in message:
                orientation_data = message.get('armorientation', [])
                print("  - 姿态数据(armorientation): {}".format(orientation_data))
            elif 'armOrientation' in message:
                orientation_data = message.get('armOrientation', [])
                print("  - 姿态数据(armOrientation): {}".format(orientation_data))

            if 'tool' in message:
                tool_data = message.get('tool', 0)
                print("  - 工具数据: {}".format(tool_data))

            if 'user' in message:
                user_data = message.get('user', 0)
                print("  - 用户数据: {}".format(user_data))

            if 'id' in message:
                id_data = message.get('id', '')
                print("  - ID: {}".format(id_data))

            # 将单个点位数据添加到缓存中，而不是立即写入文件
            self._add_point_to_cache(message)

            print("[ARM_POINTS] 点位数据处理完成")

        except Exception as e:
            print("[ERROR] 处理机械臂点位数据时发生错误: {}".format(safe_str(e)))

    def _add_point_to_cache(self, point_data, auto_write=True):
        """
        将点位数据添加到缓存中

        Args:
            point_data: 点位数据
            auto_write: 是否自动写入文件（当缓存达到阈值时）
        """
        try:
            with self.cache_lock:
                self.points_cache.append(point_data)
                print("[CACHE] 点位数据已添加到缓存，当前缓存数量: {}".format(len(self.points_cache)))

                # 如果启用自动写入且缓存达到一定数量，写入文件
                if auto_write and len(self.points_cache) >= 5:  # 假设最多5个点位
                    self._write_all_points_to_folder()

        except Exception as e:
            print("[ERROR] 添加点位到缓存时发生错误: {}".format(safe_str(e)))

    def _write_all_points_to_folder(self):
        """
        将所有缓存中的点位数据写入文件夹
        """
        try:
            with self.cache_lock:
                if not self.points_cache:
                    print("[CACHE] 缓存为空，无需写入")
                    return

                print("[CACHE] 开始写入所有点位数据，共 {} 个点位".format(len(self.points_cache)))

                # 获取当前时间戳
                import time
                timestamp = int(time.time())
                current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

                # 基础路径
                import os
                base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"

                # 从最近发送的消息中获取map_id和pose_id
                map_id = "default_map"
                pose_id = "default_pose"

                if hasattr(self, 'last_sent_message') and self.last_sent_message:
                    if 'mapid' in self.last_sent_message:
                        map_id = self.last_sent_message['mapid']
                    elif 'map_id' in self.last_sent_message:
                        map_id = self.last_sent_message['map_id']

                    if 'poseid' in self.last_sent_message:
                        pose_id = self.last_sent_message['poseid']
                    elif 'pose_id' in self.last_sent_message:
                        pose_id = self.last_sent_message['pose_id']

                # 构造文件夹路径
                map_folder = os.path.join(base_path, map_id)
                pose_folder = os.path.join(map_folder, pose_id)

                print("[CACHE] 目标文件夹: {}".format(pose_folder))

                # 检查文件夹是否存在，如果不存在则创建
                if not os.path.exists(pose_folder):
                    print("[CACHE] 示教录点文件夹不存在，尝试创建: {}".format(pose_folder))
                    try:
                        os.makedirs(pose_folder, exist_ok=True)
                        print("[CACHE] 成功创建示教录点文件夹: {}".format(pose_folder))
                    except Exception as e:
                        print("[ERROR] 创建示教录点文件夹失败: {}".format(safe_str(e)))
                        return

                # 构造JSON文件名
                json_filename = "nx_response_all_points_{}.json".format(timestamp)
                json_filepath = os.path.join(pose_folder, json_filename)

                print("[CACHE] 目标文件: {}".format(json_filepath))

                # 构造要写入的JSON数据
                json_data = {
                    "timestamp": current_time,
                    "unix_timestamp": timestamp,
                    "map_id": map_id,
                    "pose_id": pose_id,
                    "message_type": "nx_response_all_points",
                    "total_points": len(self.points_cache),
                    "points_data": self.points_cache,
                    "description": "NX返回的所有机械臂点位数据"
                }

                # 写入JSON文件
                try:
                    # 尝试使用encoding参数（Python 3+）
                    with open(json_filepath, 'w', encoding='utf-8') as f:
                        json.dump(json_data, f, ensure_ascii=False, indent=2)
                except TypeError:
                    # 如果encoding参数不支持，使用兼容方式
                    print("[CACHE] 使用兼容模式写入文件（不支持encoding参数）")
                    with open(json_filepath, 'w') as f:
                        json.dump(json_data, f, ensure_ascii=False, indent=2)

                print("[CACHE] 所有点位数据写入成功: {}".format(json_filepath))

                # 清空缓存
                self.points_cache = []

        except Exception as e:
            print("[ERROR] 写入所有点位数据时发生错误: {}".format(safe_str(e)))

    def _write_nx_response_to_folder(self, nx_message):
        """
        将NX返回的JSON消息写入示教录点文件夹

        Args:
            nx_message: NX返回的消息
        """
        try:
            import json
            import time
            import os

            # 获取当前时间戳
            timestamp = int(time.time())
            current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

            # 基础路径
            base_path = "/root/Drobot_Navigation_Module/WORKSPACE/src/application/master_node"

            # 从最近发送的消息中获取map_id和pose_id（这些是示教录点时创建的文件夹）
            map_id = "default_map"
            pose_id = "default_pose"

            if hasattr(self, 'last_sent_message') and self.last_sent_message:
                if 'mapid' in self.last_sent_message:
                    map_id = self.last_sent_message['mapid']
                    print("[NX响应写入] 使用示教录点的地图ID: {}".format(map_id))
                elif 'map_id' in self.last_sent_message:
                    map_id = self.last_sent_message['map_id']
                    print("[NX响应写入] 使用示教录点的地图ID: {}".format(map_id))

                if 'poseid' in self.last_sent_message:
                    pose_id = self.last_sent_message['poseid']
                    print("[NX响应写入] 使用示教录点的点位ID: {}".format(pose_id))
                elif 'pose_id' in self.last_sent_message:
                    pose_id = self.last_sent_message['pose_id']
                    print("[NX响应写入] 使用示教录点的点位ID: {}".format(pose_id))
            else:
                print("[NX响应写入] 警告：无法获取示教录点的map_id和pose_id，使用默认值")

            # 构造文件夹路径（使用示教录点时创建的文件夹）
            map_folder = os.path.join(base_path, map_id)
            pose_folder = os.path.join(map_folder, pose_id)

            print("[NX响应写入] 目标文件夹: {}".format(pose_folder))

            # 检查示教录点时创建的文件夹是否存在
            if not os.path.exists(pose_folder):
                print("[NX响应写入] 示教录点文件夹不存在: {}".format(pose_folder))
                print("[NX响应写入] 请确保示教录点已完成并创建了文件夹")
                return False

            print("[NX响应写入] 示教录点文件夹存在，准备写入NX响应")

            # 构造JSON文件名
            json_filename = "nx_response_{}.json".format(timestamp)
            json_filepath = os.path.join(pose_folder, json_filename)

            print("[NX响应写入] 目标文件: {}".format(json_filepath))

            # 构造要写入的JSON数据
            json_data = {
                "timestamp": current_time,
                "unix_timestamp": timestamp,
                "map_id": map_id,
                "pose_id": pose_id,
                "message_type": "nx_response",
                "nx_message": nx_message,
                "description": "NX返回的机械臂点位数据，写入到示教录点文件夹"
            }

            # 写入JSON文件
            try:
                # 尝试使用encoding参数（Python 3+）
                with open(json_filepath, 'w', encoding='utf-8') as f:
                    json.dump(json_data, f, ensure_ascii=False, indent=2)
            except TypeError:
                # 如果encoding参数不支持，使用兼容方式
                print("[NX响应写入] 使用兼容模式写入文件（不支持encoding参数）")
                with open(json_filepath, 'w') as f:
                    json.dump(json_data, f, ensure_ascii=False, indent=2)

            print("[NX响应写入] NX响应消息写入成功: {}".format(json_filepath))
            return True

        except Exception as e:
            print("[ERROR] 写入NX响应消息时发生错误: {}".format(safe_str(e)))
            return False

    def register_message_handler(self, msg_type, handler):
        """
        注册自定义消息处理器

        Args:
            msg_type: 消息类型
            handler: 处理函数
        """
        self.message_handlers[msg_type] = handler
        print("[SUCCESS] 已注册消息处理器: {}".format(msg_type))

    def _ensure_connection(self):
        """
        确保TCP连接可用，如果连接断开则尝试重连

        Returns:
            bool: 连接是否可用
        """
        try:
            # 检查连接状态
            if not self.tcp_connected:
                print("[CONNECTION] TCP连接未建立，尝试重新连接...")
                return self._start_tcp_connection()

            # 用 getpeername 验证套接字仍有效；不要 send(b'')，
            # 部分 NX 服务端会因空包断开连接
            try:
                with self.connection_lock:
                    if not self.tcp_socket:
                        raise RuntimeError("socket is None")
                    self.tcp_socket.getpeername()
                print("[CONNECTION] 连接测试成功")
                return True
            except Exception as test_e:
                print("[CONNECTION] 连接测试失败: {}, 尝试重新连接...".format(safe_str(test_e)))
                self.tcp_connected = False
                return self._start_tcp_connection()

        except Exception as e:
            print("[CONNECTION] 确保连接时发生错误: {}".format(safe_str(e)))
            return False

    def _send_message(self, message, wait_response=None, timeout=70):
        """
        发送TCP消息

        Args:
            message: 要发送的消息字典
            wait_response: 是否等待最终响应；None 时 dobot 默认等待，estun 不等待
            timeout: 等待响应超时（秒）

        Returns:
            bool 或 dict: 不等待时返回 bool；等待时成功返回响应 dict，失败返回 False
        """
        try:
            # 确保连接可用
            if not self._ensure_connection():
                print("[ERROR] 无法建立TCP连接")
                return False

            if wait_response is None:
                wait_response = self.is_dobot()

            # 保存最近发送的消息，用于后续接收NX响应时确定文件夹路径
            self.last_sent_message = message.copy()
            print("[SEND] 保存最近发送的消息: {}".format(self.last_sent_message))

            # 转换为JSON字符串，CR5 约定换行结尾；estun 同样加换行兼容接收端
            message_str = json.dumps(message, ensure_ascii=False)
            if not message_str.endswith('\n'):
                message_str = message_str + '\n'
            if hasattr(message_str, 'encode'):
                message_bytes = message_str.encode('utf-8')
            else:
                message_bytes = message_str

            if wait_response:
                with self.response_condition:
                    self.pending_response = None
                    self.waiting_for_response = True

            # 直接发送JSON字符串
            with self.connection_lock:
                self.tcp_socket.sendall(message_bytes)

            print("[SEND] 消息发送成功: {}".format(message))
            print("[DEBUG] 发送的字节数: {}, 原始JSON: {}".format(len(message_bytes), message_str.rstrip('\n')))

            if not wait_response:
                return True

            response = self._wait_for_request_response(timeout)
            if response is None:
                print("[ERROR] 等待 NX 响应超时 ({}s)".format(timeout))
                return False

            status = response.get('status')
            print("[SEND] 收到 NX 响应: status={}".format(status))
            if self._is_failure_status(status):
                print("[ERROR] NX 返回失败: {}".format(response))
                return False
            if self._is_success_status(status):
                return response

            # 未知 status：仍把响应返回给上层判断
            print("[WARNING] 未识别的 status: {}".format(status))
            return response

        except Exception as e:
            print("[ERROR] 发送消息失败: {}".format(safe_str(e)))
            # 发送失败时，标记连接为断开状态
            self.tcp_connected = False
            with self.response_condition:
                self.waiting_for_response = False
                self.pending_response = None
            return False

    def send_request_and_wait(self, message, timeout=60):
        """
        使用独立短连接发送一条 NX 请求并等待最终 JSON 响应。
        HTTP 示教/查询/命令需要拿到 NX 真实结果，且避免与后台接收线程争抢长连接。
        """
        request_socket = None
        try:
            request_socket = socket.create_connection(
                (self.nx_host, self.nx_port), 10.0)
            request_socket.settimeout(float(timeout))

            payload = (json.dumps(
                message, ensure_ascii=False) + '\n')
            if hasattr(payload, 'encode'):
                payload = payload.encode('utf-8')
            request_socket.sendall(payload)

            buffer = b''
            while b'\n' not in buffer:
                chunk = request_socket.recv(4096)
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > 1024 * 1024:
                    raise RuntimeError("NX响应超过1MB限制")

            response_data = buffer.split(b'\n', 1)[0].strip()
            if not response_data:
                raise RuntimeError("NX未返回响应")
            response = json.loads(response_data.decode('utf-8'))
            print("[NX同步请求] request={}, response={}".format(
                message, response))
            return response
        except Exception as e:
            print("[ERROR] NX同步请求失败: {}".format(safe_str(e)))
            return {
                'type': 'response',
                'status': 'failed',
                'error_code': 'tcp_request_failed',
                'message': safe_str(e),
                'metrics': {},
                'artifacts': []
            }
        finally:
            if request_socket:
                try:
                    request_socket.close()
                except Exception:
                    pass

    def send_teaching_start(self, map_id, pose_id, point_type,
                            params=None, timeout=60):
        """按 CR5 协议开始一个 ICP 或 AprilTag 示教点（独立短连接等响应）。"""
        if not map_id or not pose_id:
            return {
                'status': 'failed',
                'error_code': 'invalid_request',
                'message': 'mapid和poseid不能为空'
            }
        try:
            point_type = int(point_type)
        except (TypeError, ValueError):
            point_type = -1
        if point_type not in (0, 1):
            return {
                'status': 'failed',
                'error_code': 'invalid_request',
                'message': 'point_type必须为0(AprilTag)或1(ICP)'
            }

        request_params = dict(params or {})
        if point_type == 0:
            if request_params.get('tag_id') in (None, ''):
                return {
                    'status': 'failed',
                    'error_code': 'invalid_request',
                    'message': 'AprilTag示教必须提供params.tag_id'
                }
            offset = request_params.get(
                'tag_offset_xyz_mm', [0, 0, 0])
            if not isinstance(offset, (list, tuple)) or len(offset) != 3:
                return {
                    'status': 'failed',
                    'error_code': 'invalid_request',
                    'message': 'params.tag_offset_xyz_mm必须包含3个数值'
                }
            request_params['tag_offset_xyz_mm'] = list(offset)

        return self.send_request_and_wait({
            'type': 1,
            'mapid': str(map_id),
            'poseid': str(pose_id),
            'point_type': point_type,
            'params': request_params
        }, timeout)

    def query_station_points(self, map_id, pose_id, timeout=15):
        """查询 NX 本地保存的站点点位，不触发机械臂动作。"""
        return self.send_request_and_wait({
            'type': 2,
            'mapid': str(map_id),
            'poseid': str(pose_id)
        }, timeout)

    def send_mechanical_arm_command_wait(self, command, timeout=90):
        """发送 command 1~6 并返回 NX 最终执行结果（短连接）。"""
        return self.send_request_and_wait({
            'type': 'mechanical_arm_command',
            'command': int(command)
        }, timeout)

    def send_navigation_point_reached(self, map_id, point_id, wait_response=None, timeout=None):
        """
        发送机器人到达导航点的消息

        Args:
            map_id: 地图ID
            point_id: 导航点ID
            wait_response: 是否等待响应
            timeout: 超时秒数；dobot 生产默认 300s

        Returns:
            bool 或 dict: 发送/响应结果
        """
        print("=== 机械臂控制器发送消息 - 开始 ===")
        print("接收参数 - map_id: {}, point_id: {}".format(map_id, point_id))
        print("point_id类型: {}, 值: {}".format(type(point_id), point_id))

        # 确保值不为空
        if not map_id or not point_id:
            print("[ERROR] map_id或point_id为空 - map_id: {}, point_id: {}".format(map_id, point_id))
            return False

        # 检查point_id是否为undefined
        if point_id == 'undefined' or point_id == 'null':
            print("[ERROR] point_id为undefined或null: {}".format(point_id))
            return False


        message = {
            'mapid': str(map_id),
            'poseid': str(point_id),
        }

        print("构造的消息: {}".format(message))
        print("发送给NX: mapid={}, poseid={}".format(map_id, point_id))

        if timeout is None:
            timeout = 300 if self.is_dobot() else 200
        # 生产到站：dobot 等最终响应；estun 由 execute 层等 done
        if wait_response is None:
            wait_response = self.is_dobot()

        result = self._send_message(message, wait_response=wait_response, timeout=timeout)
        print("发送结果: {}".format(result))
        return result

    def send_demo_point(self, map_id, pose_id, msg_type=1, point_type=None, params=None, timeout=70):
        """
        发送示教开始消息

        Args:
            map_id: 地图ID
            pose_id: 站点ID
            msg_type: 示教类型字段（1/type1）
            point_type: 越疆点位类型 0=AprilTag 1=ICP
            params: AprilTag 等额外参数
            timeout: 等待响应超时
        """
        if self.is_dobot():
            # 越疆：严格校验 + 短连接等响应
            return self.send_teaching_start(
                map_id, pose_id, point_type, params=params, timeout=timeout)

        message = {
            'type': msg_type,
            'mapid': str(map_id),
            'poseid': str(pose_id),
        }
        return self._send_message(message, wait_response=False)

    def query_demo_points(self, map_id, pose_id, timeout=20):
        """
        查询站点点位（越疆 type=2；埃斯顿返回 None 由上层读本地文件）
        """
        if not self.is_dobot():
            return None
        return self.query_station_points(map_id, pose_id, timeout=timeout)

    def send_mechanical_arm_command(self, command, parameters=None, wait_response=None, timeout=100):
        """
        发送机械臂控制命令

        Args:
            command: 命令类型
            parameters: 命令参数
            wait_response: 是否等待响应
            timeout: 超时

        Returns:
            bool 或 dict: 发送是否成功 / NX 响应
        """
        if wait_response is None:
            wait_response = self.is_dobot()

        # 越疆 HTTP 控制：独立短连接等真实结果（无额外 parameters 时）
        if self.is_dobot() and wait_response and not parameters:
            return self.send_mechanical_arm_command_wait(command, timeout=timeout)

        message = {
            'type': 'mechanical_arm_command',
            'command': command
        }

        if parameters:
            message.update(parameters)

        return self._send_message(message, wait_response=wait_response, timeout=timeout)

    def execute_navigation_point_action(self, map_id, point_id, wait_for_completion=True, timeout=None):
        """
        执行导航点对应的机械臂动作

        Args:
            map_id: 地图ID
            point_id: 导航点ID
            wait_for_completion: 是否等待完成信号
            timeout: 等待超时时间（秒）；None 时 dobot=300, estun=200
        """
        if timeout is None:
            timeout = 300 if self.is_dobot() else 200
        try:
            # 添加详细的机械臂动作执行日志
            print("=== 机械臂动作执行开始 ===")
            print("地图ID: {}".format(map_id))
            print("导航点ID: {}".format(point_id))
            print("等待完成信号: {}".format(wait_for_completion))
            print("超时时间: {}秒".format(timeout))
            print("执行时间: {}".format(datetime.now().strftime('%Y-%m-%d %H:%M:%S')))

            with self.status_lock:
                if self.is_mechanical_arm_running:
                    # 安全地处理字符串格式化，避免编码问题
                    try:
                        safe_point_id = str(point_id).encode('utf-8').decode('utf-8')
                        print("[WARNING] 机械臂正在执行其他动作，跳过: {}".format(safe_point_id))
                    except (UnicodeError, UnicodeDecodeError):
                        print("[WARNING] 机械臂正在执行其他动作，跳过: {}".format(repr(point_id)))
                    return False

                self.is_mechanical_arm_running = True
                print("机械臂状态: 设置为运行中")

            # 重置完成标志
            with self.completion_condition:
                self.mechanical_arm_completed = False
                self.last_completion_response = None
                print("机械臂完成标志: 重置为False")

            # 记录开始时间
            start_time = datetime.now()
            # 安全地处理字符串格式化，避免编码问题
            try:
                safe_map_id = str(map_id).encode('utf-8').decode('utf-8')
                safe_point_id = str(point_id).encode('utf-8').decode('utf-8')
                print("[INFO] 开始执行机械臂动作 - 地图: {}, 导航点: {}, 时间: {}".format(safe_map_id, safe_point_id, start_time.strftime('%Y-%m-%d %H:%M:%S')))
            except (UnicodeError, UnicodeDecodeError):
                print("[INFO] 开始执行机械臂动作 - 地图: {}, 导航点: {}, 时间: {}".format(repr(map_id), repr(point_id), start_time.strftime('%Y-%m-%d %H:%M:%S')))

            # 生产到站：长连接发送 mapid/poseid，再等终态（与朋友版任务管理器路径一致）
            print("正在发送导航点到达消息到NX...")
            success = self.send_navigation_point_reached(
                map_id, point_id, wait_response=False)
            if success is True or (isinstance(success, dict) and self._is_success_status(success.get('status'))):
                success = True
            else:
                success = bool(success)
            print("发送结果: {}".format("成功" if success else "失败"))

            if success:
                if wait_for_completion:
                    print("机械臂动作触发成功，等待NX完成信号...")
                    completion_success = self._wait_for_completion(timeout)
                    if completion_success:
                        end_time = datetime.now()
                        duration = (end_time - start_time).total_seconds() if hasattr(end_time - start_time, 'total_seconds') else (end_time - start_time).seconds
                        try:
                            safe_map_id = str(map_id).encode('utf-8').decode('utf-8')
                            safe_point_id = str(point_id).encode('utf-8').decode('utf-8')
                            print("[SUCCESS] 机械臂动作执行完成 - 地图: {}, 导航点: {}, 耗时: {:.2f}秒".format(safe_map_id, safe_point_id, duration))
                            print("=== 机械臂动作执行成功 ===")
                        except (UnicodeError, UnicodeDecodeError):
                            print("[SUCCESS] 机械臂动作执行完成 - 地图: {}, 导航点: {}, 耗时: {:.2f}秒".format(repr(map_id), repr(point_id), duration))
                            print("=== 机械臂动作执行成功 ===")
                        return True
                    else:
                        if self.last_completion_response is None:
                            print("[ERROR] 等待机械臂完成信号超时")
                            print("=== 机械臂动作执行超时 ===")
                        else:
                            print("[ERROR] NX机械臂任务执行失败: {}".format(
                                self.last_completion_response))
                            print("=== 机械臂动作执行失败 ===")
                        return False
                else:
                    print("机械臂动作触发成功，不等待完成信号")
                    print("=== 机械臂动作触发成功 ===")
                    return True
            else:
                print("[ERROR] 发送导航点消息失败，机械臂动作未执行")
                print("=== 机械臂动作执行失败 ===")
                return False

        except Exception as e:
            print("[ERROR] 机械臂动作执行过程中发生异常")
            print("异常类型: {}".format(type(e).__name__))
            print("异常信息: {}".format(safe_str(e)))
            try:
                safe_map_id = str(map_id).encode('utf-8').decode('utf-8')
                safe_point_id = str(point_id).encode('utf-8').decode('utf-8')
                print("[ERROR] 执行机械臂动作异常 - 地图: {}, 导航点: {}, 异常: {}".format(safe_map_id, safe_point_id, safe_str(e)))
            except (UnicodeError, UnicodeDecodeError):
                print("[ERROR] 执行机械臂动作异常 - 地图: {}, 导航点: {}, 异常: {}".format(repr(map_id), repr(point_id), safe_str(e)))
            print("=== 机械臂动作执行异常 ===")
            return False
        finally:
            with self.status_lock:
                self.is_mechanical_arm_running = False
                print("机械臂状态: 重置为空闲")
                print("=== 机械臂动作执行结束 ===")

    def _wait_for_completion(self, timeout=300):
        """
        等待机械臂完成信号（兼容 Python2 Condition.wait 返回 None）

        Args:
            timeout: 超时时间（秒）

        Returns:
            bool: 是否在超时前收到 done
        """
        try:
            print("[MECHANICAL_ARM] 开始等待完成信号，超时时间: {}秒".format(timeout))

            deadline = time.time() + float(timeout)
            with self.completion_condition:
                while self.last_completion_response is None:
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        print("[MECHANICAL_ARM] 等待完成信号超时")
                        return False
                    # Python 2 的 Condition.wait 返回 None，不能用返回值判断
                    self.completion_condition.wait(remaining)

                if self.mechanical_arm_completed:
                    print("[MECHANICAL_ARM] 收到done信号，机械臂动作完成")
                    return True

                print("[MECHANICAL_ARM] NX返回失败终态: {}".format(
                    self.last_completion_response))
                return False

        except Exception as e:
            print("[ERROR] 等待完成信号时发生异常: {}".format(safe_str(e)))
            return False

    def test_connection(self):
        """
        测试TCP连接

        Returns:
            bool: 连接是否正常
        """
        try:
            # 发送测试消息
            test_message = {
                'type': 'test',
                'message': 'connection_test',
                'source': 'master_node'
            }

            return self._send_message(test_message, wait_response=False)

        except Exception as e:
            print("[ERROR] 连接测试失败: {}".format(safe_str(e)))
            return False

    def reconnect(self):
        """
        重新连接TCP

        Returns:
            bool: 重连是否成功
        """
        print("[INFO] 正在重新连接TCP...")

        # 关闭现有连接
        self._close_connection()

        # 等待一段时间后重连
        time.sleep(2)

        # 尝试重连
        return self._start_tcp_connection()

    def _close_connection(self):
        """关闭TCP连接"""
        try:
            with self.connection_lock:
                if self.tcp_socket:
                    self.tcp_socket.close()
                    self.tcp_socket = None

                self.tcp_connected = False

        except Exception as e:
            print("[ERROR] 关闭连接时发生错误: {}".format(safe_str(e)))

    def get_controller_status(self):
        """
        获取控制器状态

        Returns:
            dict: 控制器状态信息
        """
        return {
            'tcp_connected': self.tcp_connected,
            'nx_host': self.nx_host,
            'nx_port': self.nx_port,
            'arm_vendor': self.arm_vendor,
            'is_running': self.is_mechanical_arm_running,
            'message_sequence': self.message_sequence,
            'connection_status': 'connected' if self.tcp_connected else 'disconnected'
        }

    def check_connection_status(self):
        """
        检查TCP连接状态，如果断开则尝试重连

        Returns:
            dict: 连接状态信息
        """
        try:
            # 尝试确保连接可用
            connection_ok = self._ensure_connection()

            status_info = {
                'tcp_connected': self.tcp_connected,
                'connection_ok': connection_ok,
                'nx_host': self.nx_host,
                'nx_port': self.nx_port,
                'status': 'connected' if connection_ok else 'disconnected',
                'message': 'TCP连接正常' if connection_ok else 'TCP连接异常，请检查NX设备状态'
            }

            print("[CONNECTION_STATUS] 连接状态检查结果: {}".format(status_info))
            return status_info

        except Exception as e:
            print("[CONNECTION_STATUS] 检查连接状态时发生错误: {}".format(safe_str(e)))
            return {
                'tcp_connected': False,
                'connection_ok': False,
                'nx_host': self.nx_host,
                'nx_port': self.nx_port,
                'status': 'error',
                'message': '连接状态检查失败: {}'.format(safe_str(e))
            }

    def shutdown(self):
        """关闭控制器"""
        print("[INFO] 正在关闭机械臂控制器...")

        # 设置关闭标志
        self.shutdown_event.set()

        # 关闭TCP连接
        self._close_connection()

        # 等待接收线程结束
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2)

        print("[SUCCESS] 机械臂控制器已关闭")

    def __del__(self):
        """析构函数"""
        try:
            if hasattr(self, 'shutdown_event'):
                self.shutdown()
        except:
            pass

# 兼容性方法（保持向后兼容）
class MechanicalArmControllerCompat(MechanicalArmController):
    """兼容性版本的机械臂控制器，提供旧接口"""

    def send_navigation_point(self, point_id):
        """兼容旧接口"""
        print("[WARNING] send_navigation_point 方法已过时，请使用 send_navigation_point_reached")
        return self.send_navigation_point_reached("unknown_map", point_id)

    def add_mechanical_arm_point(self, point_name, point_id, launch_file_name=None):
        """兼容旧接口"""
        print("[WARNING] add_mechanical_arm_point 方法已过时")
        return True

    def remove_mechanical_arm_point(self, point_id):
        """兼容旧接口"""
        print("[WARNING] remove_mechanical_arm_point 方法已过时")
        return True

    def clear_mechanical_arm_points(self):
        """兼容旧接口"""
        print("[WARNING] clear_mechanical_arm_points 方法已过时")
        return True

    def is_mechanical_arm_point(self, point_id):
        """兼容旧接口"""
        print("[WARNING] is_mechanical_arm_point 方法已过时")
        return True

    def on_robot_reached_point(self, point_id, point_name=None):
        """兼容旧接口"""
        print("[WARNING] on_robot_reached_point 方法已过时，请使用 execute_navigation_point_action")
        return self.execute_navigation_point_action("unknown_map", point_id)

    def execute_task_mechanical_arm_action(self, point_id, point_name, action_config):
        """兼容旧接口"""
        print("[WARNING] execute_task_mechanical_arm_action 方法已过时，请使用 execute_navigation_point_action")
        return self.execute_navigation_point_action("unknown_map", point_id)

    def send_single_point(self, point_name):
        """兼容旧接口"""
        print("[WARNING] send_single_point 方法已过时，请使用 send_navigation_point_reached")
        return self.send_navigation_point_reached("unknown_map", point_name)

    def send_multiple_points(self, point_names):
        """兼容旧接口"""
        print("[WARNING] send_multiple_points 方法已过时，请使用 send_navigation_point_reached 逐个发送")
        success_count = 0
        for point_name in point_names:
            if self.send_navigation_point_reached("unknown_map", point_name):
                success_count += 1
        return success_count == len(point_names)

    def send_custom_sequence(self, all_points, selected_points):
        """兼容旧接口"""
        print("[WARNING] send_custom_sequence 方法已过时，请使用 send_navigation_point_reached 逐个发送")
        success_count = 0
        for point_name in selected_points:
            if self.send_navigation_point_reached("unknown_map", point_name):
                success_count += 1
        return success_count == len(selected_points)

    def get_mechanical_arm_status(self):
        """兼容旧接口"""
        print("[WARNING] get_mechanical_arm_status 方法已过时，请使用 get_controller_status")
        return self.get_controller_status()

    def reset_mechanical_arm_points(self):
        """兼容旧接口"""
        print("[WARNING] reset_mechanical_arm_points 方法已过时")
        return True

    def check_script_status(self):
        """兼容旧接口"""
        print("[WARNING] check_script_status 方法已过时，TCP连接状态: " + ("已连接" if self.tcp_connected else "未连接"))
        return {
            'success': self.tcp_connected,
            'output': "TCP连接状态: {}".format('已连接' if self.tcp_connected else '未连接'),
            'error': '' if self.tcp_connected else 'TCP连接未建立'
        }

    def get_script_logs(self):
        """兼容旧接口"""
        print("[WARNING] get_script_logs 方法已过时")
        return {
            'success': True,
            'output': 'TCP通信模式，无脚本日志',
            'error': ''
        }

    def _call_tcp_ros_demo_script(self, action):
        """兼容旧接口"""
        print("[WARNING] _call_tcp_ros_demo_script 方法已过时，TCP通信模式不支持脚本调用")
        return {
            'success': False,
            'error': 'TCP通信模式不支持脚本调用'
        }
