#!/usr/bin/env python3
import base64
import json
import os
import requests

# 配置信息
GITHUB_USERNAME = "hzeyu3958-cloud"
REPO_NAME = "mecha-humanoid-21dof"
PROJECT_ROOT = r"D:\zhuijinde\Desktop\yyds"

# 要上传的文件列表
FILES = [
    "README.md",
    ".gitignore",
    "controllers/walk_controller/walk_controller.py",
    "controllers/walk_controller/walk_controller.cpp",
    "controllers/walk_controller/Makefile",
    "controllers/walk_controller/runtime.ini",
    "protos/MechaHumanoid21.proto",
    "worlds/mecha_humanoid_21dof.wbt"
]

def upload_project():
    print("🚀 开始上传人形机器人项目到 GitHub...")
    
    # 检查文件是否存在
    print("📁 检查项目文件...")
    for file_path in FILES:
        full_path = os.path.join(PROJECT_ROOT, file_path)
        if os.path.exists(full_path):
            size = os.path.getsize(full_path)
            print(f"   ✅ {file_path} ({size} bytes)")
        else:
            print(f"   ❌ {file_path} (文件不存在)")
    
    print("\n📋 项目信息:")
    print(f"   仓库名: {REPO_NAME}")
    print(f"   描述: 21自由度人形机器人Webots仿真项目")
    print(f"   包含文件: {len(FILES)} 个核心文件")
    
    print("\n🔗 上传完成后，仓库地址将是:")
    print(f"   https://github.com/{GITHUB_USERNAME}/{REPO_NAME}")
    
    print("\n💡 要完成上传，你需要:")
    print("1. 获取 GitHub token: https://github.com/settings/tokens")
    print("2. 设置环境变量: $env:GITHUB_TOKEN='your_token_here'")
    print("3. 运行上传脚本")
    
    # 显示项目特色
    print("\n🎯 项目特色:")
    print("   ✅ 21自由度完整人形机器人")
    print("   ✅ 优化的行走控制器(已修复轻飘飘问题)")
    print("   ✅ Python 和 C++ 双版本控制器")
    print("   ✅ 完整的仿真环境")
    print("   ✅ 详细的机器人物理参数")
    
    print("\n📊 优化内容:")
    print("   • 电机速度: 2.0 → 6.0")
    print("   • 地面摩擦力: 2.0 → 5.0")
    print("   • 踝关节力矩: 20 → 60")
    print("   • 脚部质量: 0.4 → 0.8")
    print("   • 步态参数: 全面优化")

if __name__ == "__main__":
    upload_project()