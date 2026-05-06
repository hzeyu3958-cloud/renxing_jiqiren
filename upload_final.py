#!/usr/bin/env python3
import base64
import json
import os
import requests

# GitHub 配置
GITHUB_USERNAME = "hzeyu3958-cloud"
REPO_NAME = "mecha-humanoid-21dof"
PROJECT_ROOT = r"D:\zhuijinde\Desktop\yyds"

# 项目文件列表
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

def check_files():
    """检查项目文件"""
    print("🔍 检查项目文件...")
    file_info = []
    for file_path in FILES:
        full_path = os.path.join(PROJECT_ROOT, file_path)
        if os.path.exists(full_path):
            size = os.path.getsize(full_path)
            file_info.append((file_path, size, True))
        else:
            file_info.append((file_path, 0, False))
    return file_info

def generate_upload_script():
    """生成上传脚本"""
    script_content = """#!/bin/bash
# 人形机器人项目上传脚本
# 使用方法: bash upload.sh <github_token>

if [ $# -eq 0 ]; then
    echo "❌ 请提供 GitHub token"
    echo "💡 使用方法: bash upload.sh <your_github_token>"
    echo "💡 获取 token: https://github.com/settings/tokens"
    exit 1
fi

TOKEN=$1
USERNAME="hzeyu3958-cloud"
REPO="mecha-humanoid-21dof"

# 创建仓库
echo "🚀 创建 GitHub 仓库..."
curl -X POST \\
  -H "Authorization: token $TOKEN" \\
  -H "Accept: application/vnd.github.v3+json" \\
  https://api.github.com/user/repos \\
  -d '{
    "name": "'$REPO'",
    "description": "21自由度人形机器人Webots仿真项目",
    "private": false,
    "auto_init": false
  }'

echo "✅ 仓库创建完成"
echo "🔗 仓库地址: https://github.com/$USERNAME/$REPO"

echo ""
echo "📋 手动上传步骤:"
echo "1. 访问: https://github.com/$USERNAME/$REPO"
echo "2. 点击 'Add file' -> 'Upload files'"
echo "3. 上传以下文件:"
"""
    
    # 添加文件列表
    file_info = check_files()
    for file_path, size, exists in file_info:
        if exists:
            script_content += f'echo "   📄 {file_path} ({size} bytes)"\n'
    
    script_content += """
echo ""
echo "🎯 项目特色:"
echo "   ✅ 21自由度完整人形机器人"
echo "   ✅ 优化的行走控制器(已修复轻飘飘问题)"
echo "   ✅ Python 和 C++ 双版本控制器"
echo "   ✅ 完整的仿真环境"
echo "   ✅ 详细的机器人物理参数"
"""
    
    return script_content

def main():
    print("🚀 人形机器人项目上传准备")
    print("=" * 50)
    
    # 检查文件
    file_info = check_files()
    
    print("\n📁 项目文件清单:")
    for file_path, size, exists in file_info:
        if exists:
            print(f"   ✅ {file_path} ({size} bytes)")
        else:
            print(f"   ❌ {file_path} (缺失)")
    
    # 生成上传脚本
    script_content = generate_upload_script()
    script_path = os.path.join(PROJECT_ROOT, "upload.sh")
    
    with open(script_path, "w", encoding="utf-8") as f:
        f.write(script_content)
    
    print(f"\n📄 上传脚本已生成: upload.sh")
    
    # 显示项目信息
    print("\n🎯 项目特色:")
    print("   • 21自由度完整人形机器人")
    print("   • 优化的行走控制器(已修复轻飘飘问题)")
    print("   • Python 和 C++ 双版本控制器")
    print("   • 完整的仿真环境")
    print("   • 详细的机器人物理参数")
    
    print("\n🔧 优化内容:")
    print("   • 电机速度: 2.0 → 6.0")
    print("   • 地面摩擦力: 2.0 → 5.0") 
    print("   • 踝关节力矩: 20 → 60")
    print("   • 脚部质量: 0.4 → 0.8")
    print("   • 步态参数: 全面优化")
    
    print(f"\n🔗 仓库地址: https://github.com/{GITHUB_USERNAME}/{REPO_NAME}")
    
    print("\n💡 上传方法:")
    print("1. 获取 GitHub token: https://github.com/settings/tokens")
    print("2. 运行: bash upload.sh <your_token>")
    print("3. 或手动上传文件到 GitHub")

if __name__ == "__main__":
    main()