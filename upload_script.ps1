# PowerShell 脚本用于上传人形机器人项目到 GitHub

# 设置变量
$ProjectPath = "D:\zhuijinde\Desktop\yyds"
$RepoName = "mecha-humanoid-21dof"
$Description = "21自由度人形机器人Webots仿真项目"

# 切换到项目目录
Set-Location $ProjectPath

# 检查是否已经是 Git 仓库
if (Test-Path ".git") {
    Write-Host "ℹ️ 已经是 Git 仓库"
} else {
    # 初始化 Git 仓库
    Write-Host "🚀 初始化 Git 仓库..."
    git init
    
    # 配置用户信息
    git config user.name "hzeyu3958-cloud"
    git config user.email "hzeyu3958@example.com"
}

# 添加文件到 Git
Write-Host "📁 添加文件到 Git..."
git add .

# 提交更改
Write-Host "💾 提交更改..."
git commit -m "初始提交: 21自由度人形机器人Webots仿真项目"

# 检查是否已连接到远程仓库
$remoteExists = git remote -v | Select-String "origin"
if ($remoteExists) {
    Write-Host "ℹ️ 已连接到远程仓库"
} else {
    Write-Host "🔗 连接到 GitHub 仓库..."
    Write-Host "💡 请手动执行以下命令:"
    Write-Host ""
    Write-Host "1. 创建 GitHub 仓库:"
    Write-Host "   gh repo create $RepoName --public --description '$Description'"
    Write-Host ""
    Write-Host "2. 添加远程仓库:"
    Write-Host "   git remote add origin https://github.com/hzeyu3958-cloud/$RepoName.git"
    Write-Host ""
    Write-Host "3. 推送代码:"
    Write-Host "   git push -u origin main"
    Write-Host ""
}

Write-Host "✅ 本地 Git 仓库已准备就绪"
Write-Host "📊 项目包含以下文件:"
Get-ChildItem -Recurse -File | ForEach-Object { Write-Host "   - $($_.FullName.Replace($ProjectPath, ''))" }

Write-Host ""
Write-Host "🎯 下一步操作:"
Write-Host "1. 确保已安装 GitHub CLI (gh)"
Write-Host "2. 登录 GitHub: gh auth login"
Write-Host "3. 执行上述命令将代码推送到 GitHub"