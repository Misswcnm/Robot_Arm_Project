import json
import sys
from pathlib import Path

def find_points(node):
    """递归查找 type=points 的 children"""
    if isinstance(node, dict):
        if node.get("type") == "points":
            return node.get("children", [])
        for v in node.values():
            res = find_points(v)
            if res:
                return res
    elif isinstance(node, list):
        for item in node:
            res = find_points(item)
            if res:
                return res
    return None

def extract_points(file_path, output_path):
    # 读取原始 .crp 文件
    try:
        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            data = json.load(f)

        # 提取 points
        points = find_points(data)
        if not points:
            print("⚠️ 没有找到 points 节点")
            return False  # 返回False表示失败

        # 只保留 label 和 apos 关节信息
        result = []
        for p in points:
            label = p.get("label")
            apos = p.get("data", {}).get("value", {}).get("posvalue", {}).get("apos", {})
            result.append({
                "label": label,
                "joints": apos
            })

        # 保存到新的 JSON 文件
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=4)

        print(f"✅ 提取完成！结果已保存到 {output_path}")
        return True  # 返回True表示成功
        
    except Exception as e:
        print(f"❌ 解析过程中发生错误: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python jiexi.py 输入文件 [输出文件]")
        sys.exit(1)

    input_file = Path(sys.argv[1])
    output_file = Path(sys.argv[2]) if len(sys.argv) > 2 else input_file.with_suffix(".points.json")

    # 调用函数并获取返回值
    success = extract_points(input_file, output_file)
    
    # 根据解析结果返回适当的退出码
    if success:
        sys.exit(0)  # 成功返回0
    else:
        sys.exit(1)  # 失败返回1