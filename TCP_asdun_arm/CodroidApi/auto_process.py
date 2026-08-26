#!/usr/bin/env python3
import os
import time
import json
import subprocess
from pathlib import Path
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# 配置路径
DOWNLOADS_DIR = "/home/nvidia/Downloads"
SCRIPT_DIR = "/home/nvidia/TCP_asdun_arm/CodroidApi"
EXTRACT_SCRIPT = os.path.join(SCRIPT_DIR, "jiexi.py")  # 修正为jiexi.py
TASK_RECORD_FILE = "/home/nvidia/TCP_asdun_arm/CodroidApi/current_task.json"
BASE_PATH = "/home/nvidia/TCP_asdun_arm/CodroidApi/src"

class CrpFileHandler(FileSystemEventHandler):
    def __init__(self):
        self.processed_files = set()
        self.load_processed_files()
    
    def load_processed_files(self):
        """加载已处理文件记录"""
        record_file = os.path.join(SCRIPT_DIR, "processed_files.json")
        if os.path.exists(record_file):
            with open(record_file, 'r') as f:
                self.processed_files = set(json.load(f))
    
    def save_processed_files(self):
        """保存已处理文件记录"""
        record_file = os.path.join(SCRIPT_DIR, "processed_files.json")
        with open(record_file, 'w') as f:
            json.dump(list(self.processed_files), f)
    
    def get_current_task(self):
        """从任务记录文件获取当前mapid和poseid"""
        try:
            if os.path.exists(TASK_RECORD_FILE):
                with open(TASK_RECORD_FILE, 'r') as f:
                    task_info = json.load(f)
                    return task_info.get('mapid'), task_info.get('poseid')
        except Exception as e:
            print(f"读取任务记录失败: {e}")
        return None, None
    
    def on_created(self, event):
        """监控新文件创建"""
        if event.is_directory:
            return
            
        file_path = event.src_path
        if file_path.endswith('.crp') and file_path not in self.processed_files:
            print(f"检测到新的示教文件: {file_path}")
            
            # 等待文件完全写入
            time.sleep(2)
            self.process_crp_file(file_path)
    
    def process_crp_file(self, file_path):
        """处理.crp文件"""
        mapid, poseid = self.get_current_task()
        
        if not mapid or not poseid:
            print("⚠️ 未找到当前任务信息，跳过处理")
            return
        
        print(f"当前任务: mapid={mapid}, poseid={poseid}")
        
        # 构建目标路径
        target_dir = Path(BASE_PATH) / mapid / poseid
        target_dir.mkdir(parents=True, exist_ok=True)
        
        # 输出文件名
        output_file = target_dir / f"{mapid}_{poseid}.json"
        
        # 调用解析脚本
        cmd = [
            "python3", EXTRACT_SCRIPT,
            str(file_path),
            str(output_file)
        ]
        
        try:
            print(f"开始解析文件: {file_path}")
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"解析成功: {output_file}")
                self.processed_files.add(file_path)
                self.save_processed_files()
                
                # 可选：将原文件移动到备份目录
                backup_dir = Path(DOWNLOADS_DIR) / "processed_backup"
                backup_dir.mkdir(exist_ok=True)
                backup_file = backup_dir / Path(file_path).name
                os.rename(file_path, backup_file)
                print(f"原文件已备份到: {backup_file}")
            else:
                print(f"解析失败: {result.stderr}")
                
        except subprocess.CalledProcessError as e:
            print(f"解析脚本执行失败: {e}")
        except Exception as e:
            print(f"处理过程中发生错误: {e}")

def start_monitoring():
    """启动文件监控"""
    print("启动示教文件自动监控...")
    print(f"监控目录: {DOWNLOADS_DIR}")
    print(f"解析脚本: {EXTRACT_SCRIPT}")
    print(f"基础路径: {BASE_PATH}")
    
    event_handler = CrpFileHandler()
    observer = Observer()
    observer.schedule(event_handler, DOWNLOADS_DIR, recursive=False)
    observer.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("停止监控")
        observer.stop()
    observer.join()

if __name__ == "__main__":
    # 检查依赖
    if not os.path.exists(EXTRACT_SCRIPT):
        print(f"解析脚本不存在: {EXTRACT_SCRIPT}")
        exit(1)
    
    if not os.path.exists(DOWNLOADS_DIR):
        print(f"监控目录不存在: {DOWNLOADS_DIR}")
        exit(1)
    
    start_monitoring()