#!/usr/bin/env python3
import base64
import json
import os
import requests
import sys

# GitHub API 配置
GITHUB_USERNAME = "hzeyu3958-cloud"
REPO_NAME = "mecha-humanoid-21dof"
REPO_DESCRIPTION = "21自由度人形机器人Webots仿真项目"

# 项目根目录
PROJECT_ROOT = r"D:\zhuijinde\Desktop\yyds"

# 要上传的文件列表（相对路径）
FILES_TO_UPLOAD = [
    "README.md",
    ".gitignore", 
    "controllers/walk_controller/walk_controller.py",
    "controllers/walk_controller/walk_controller.cpp",
    "controllers/walk_controller/Makefile",
    "controllers/walk_controller/runtime.ini",
    "protos/MechaHumanoid21.proto",
    "worlds/mecha_humanoid_21dof.wbt"
]

def get_github_token():
    """获取 GitHub 访问令牌"""
    # 尝试从环境变量获取
    token = os.environ.get('GITHUB_TOKEN')
    if token and not token.startswith('gho_'):
        print("❌ 请设置有效的 GitHub token")
        print("💡 获取 token: https://github.com/settings/tokens")
        print("💡 设置 token: $env:GITHUB_TOKEN='your_token_here'")
        return None
    return token

def create_github_repo(token):
    """创建 GitHub 仓库"""
    url = f"https://api.github.com/user/repos"
    headers = {
        'Authorization': f'token {token}',
        'Accept': 'application/vnd.github.v3+json'
    }
    data = {
        'name': REPO_NAME,
        'description': REPO_DESCRIPTION,
        'private': False,
        'auto_init': False
    }
    
    response = requests.post(url, headers=headers, json=data)
    
    if response.status_code == 201:
        repo_info = response.json()
        print(f"✅ 仓库创建成功: {repo_info['html_url']}")
        return True
    elif response.status_code == 422 and "already exists" in response.text:
        print("ℹ️ 仓库已存在，继续上传文件...")
        return True
    else:
        print(f"❌ 创建仓库失败: {response.status_code} - {response.text}")
        return False

def upload_file_to_github(token, file_path, github_path):
    """上传单个文件到 GitHub"""
    full_path = os.path.join(PROJECT_ROOT, file_path)
    
    if not os.path.exists(full_path):
        print(f"⚠️ 文件不存在: {full_path}")
        return False
    
    # 读取文件内容
    with open(full_path, 'rb') as f:
        content = base64.b64encode(f.read()).decode('utf-8')
    
    url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{github_path}"
    headers = {
        'Authorization': f'token {token}',
        'Accept': 'application/vnd.github.v3+json'
    }
    data = {
        'message': f'Add {github_path}',
        'content': content,
        'branch': 'main'
    }
    
    response = requests.put(url, headers=headers, json=data)
    
    if response.status_code == 201:
        print(f"✅ 上传成功: {github_path}")
        return True
    else:
        print(f"❌ 上传失败 {github_path}: {response.status_code} - {response.text}")
        return False

def main():
    print("🚀 开始直接上传人形机器人项目到 GitHub...")
    
    # 获取 GitHub token
    token = get_github_token()
    if not token:
        print("\n📋 手动上传步骤:")
        print("1. 获取 GitHub token: https://github.com/settings/tokens")
        print("2. 设置环境变量: $env:GITHUB_TOKEN='your_token_here'")
        print("3. 重新运行此脚本")
        return
    
    # 创建仓库
    if not create_github_repo(token):
        return
    
    # 上传文件
    success_count = 0
    for file_path in FILES_TO_UPLOAD:
        if upload_file_to_github(token, file_path, file_path):
            success_count += 1
    
    print(f"\n📊 上传完成: {success_count}/{len(FILES_TO_UPLOAD)} 个文件成功上传")
    print(f"🔗 仓库地址: https://github.com/{GITHUB_USERNAME}/{REPO_NAME}")
    
    # 显示项目信息
    print(f"\n📁 项目包含以下核心文件:")
    for file_path in FILES_TO_UPLOAD:
        full_path = os.path.join(PROJECT_ROOT, file_path)
        if os.path.exists(full_path):
            file_size = os.path.getsize(full_path)
            print(f"   📄 {file_path} ({file_size} bytes)")

if __name__ == "__main__":
    main()