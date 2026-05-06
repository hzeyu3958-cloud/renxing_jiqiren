#!/usr/bin/env python3
import base64
import json
import os
import urllib.request
import urllib.error

# GitHub API settings
# 你需要先设置 GitHub token
token = "gho_************************************"  # 替换为你的 GitHub token
repo = "hzeyu3958-cloud/mecha-humanoid-21dof"
branch = "main"

# 项目根目录
project_root = r"D:\zhuijinde\Desktop\yyds"

# 要上传的文件列表
files_to_upload = [
    "README.md",
    ".gitignore",
    "controllers/walk_controller/walk_controller.py",
    "controllers/walk_controller/walk_controller.cpp", 
    "controllers/walk_controller/Makefile",
    "controllers/walk_controller/runtime.ini",
    "protos/MechaHumanoid21.proto",
    "worlds/mecha_humanoid_21dof.wbt"
]

headers = {
    'Authorization': f'token {token}',
    'Accept': 'application/vnd.github.v3+json',
    'Content-Type': 'application/json'
}

def create_repo():
    """创建 GitHub 仓库"""
    url = "https://api.github.com/user/repos"
    data = {
        'name': 'mecha-humanoid-21dof',
        'description': '21自由度人形机器人Webots仿真项目',
        'private': False,
        'auto_init': False
    }
    
    req = urllib.request.Request(url, data=json.dumps(data).encode(), headers=headers, method='POST')
    
    try:
        with urllib.request.urlopen(req) as response:
            result = json.loads(response.read().decode())
            print(f"✅ 仓库创建成功: {result['html_url']}")
            return True
    except urllib.error.HTTPError as e:
        error_data = e.read().decode()
        if "already exists" in error_data:
            print("ℹ️ 仓库已存在，继续上传文件...")
            return True
        else:
            print(f"❌ 创建仓库失败: {error_data}")
            return False

def upload_file(github_path, local_path):
    """上传单个文件到 GitHub"""
    full_local_path = os.path.join(project_root, local_path)
    
    if not os.path.exists(full_local_path):
        print(f"⚠️ 文件不存在: {full_local_path}")
        return False
    
    with open(full_local_path, 'rb') as f:
        content = base64.b64encode(f.read()).decode('utf-8')
    
    data = {
        'message': f'Add {github_path}',
        'content': content,
        'branch': branch
    }
    
    url = f'https://api.github.com/repos/{repo}/contents/{github_path}'
    req = urllib.request.Request(url, data=json.dumps(data).encode(), headers=headers, method='PUT')
    
    try:
        with urllib.request.urlopen(req) as response:
            print(f"✅ 上传成功: {github_path}")
            return True
    except urllib.error.HTTPError as e:
        error_data = e.read().decode()
        print(f"❌ 上传失败 {github_path}: {error_data}")
        return False

def main():
    print("🚀 开始上传人形机器人项目到 GitHub...")
    
    # 检查 token
    if token.startswith("gho_****************"):
        print("❌ 请先设置你的 GitHub token")
        print("💡 获取 token: https://github.com/settings/tokens")
        print("💡 设置 token: export GITHUB_TOKEN=your_token_here")
        return
    
    # 创建仓库
    if not create_repo():
        return
    
    # 上传文件
    success_count = 0
    for local_path in files_to_upload:
        github_path = local_path
        if upload_file(github_path, local_path):
            success_count += 1
    
    print(f"\n📊 上传完成: {success_count}/{len(files_to_upload)} 个文件成功上传")
    print(f"🔗 仓库地址: https://github.com/{repo}")

if __name__ == "__main__":
    main()