#!/bin/bash
# 人形机器人项目上传脚本
# 使用方法: bash upload.sh <github_token>

echo "🚀 人形机器人项目上传到 GitHub"
echo "================================"

if [ $# -eq 0 ]; then
    echo "❌ 请提供 GitHub token"
    echo "💡 使用方法: bash upload.sh <your_github_token>"
    echo "💡 获取 token: https://github.com/settings/tokens"
    exit 1
fi

TOKEN=$1
USERNAME="hzeyu3958-cloud"
REPO="mecha-humanoid-21dof"

# 检查文件
echo ""
echo "🔍 检查项目文件..."
files=(
    "README.md"
    ".gitignore"
    "controllers/walk_controller/walk_controller.py"
    "controllers/walk_controller/walk_controller.cpp"
    "controllers/walk_controller/Makefile"
    "controllers/walk_controller/runtime.ini"
    "protos/MechaHumanoid21.proto"
    "worlds/mecha_humanoid_21dof.wbt"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null)
        echo "   ✅ $file ($size bytes)"
    else
        echo "   ❌ $file (缺失)"
    fi
done

echo ""
echo "📋 项目信息:"
echo "   仓库名: $REPO"
echo "   描述: 21自由度人形机器人Webots仿真项目"
echo "   包含文件: ${#files[@]} 个核心文件"

echo ""
echo "🎯 项目特色:"
echo "   ✅ 21自由度完整人形机器人"
echo "   ✅ 优化的行走控制器(已修复轻飘飘问题)"
echo "   ✅ Python 和 C++ 双版本控制器"
echo "   ✅ 完整的仿真环境"
echo "   ✅ 详细的机器人物理参数"

echo ""
echo "🔧 优化内容:"
echo "   • 电机速度: 2.0 → 6.0"
echo "   • 地面摩擦力: 2.0 → 5.0"
echo "   • 踝关节力矩: 20 → 60"
echo "   • 脚部质量: 0.4 → 0.8"
echo "   • 步态参数: 全面优化"

echo ""
echo "💡 手动上传步骤:"
echo "1. 访问: https://github.com/new"
echo "2. 创建仓库: $REPO"
echo "3. 描述: 21自由度人形机器人Webots仿真项目"
echo "4. 选择: Public"
echo "5. 不勾选 'Initialize this repository with a README'"
echo "6. 点击 'Upload files' 上传所有文件"

echo ""
echo "🔗 上传完成后，仓库地址将是:"
echo "   https://github.com/$USERNAME/$REPO"

echo ""
echo "✅ 项目已准备就绪，请按照上述步骤上传！"