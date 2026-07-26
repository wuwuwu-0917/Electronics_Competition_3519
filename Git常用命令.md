# Git 常用命令速查

## 基础操作

```bash
# 查看仓库状态
git status

# 查看提交历史（简洁模式）
git log --oneline --graph --all

# 查看当前分支
git branch

# 查看远程仓库地址
git remote -v
```

## 暂存与提交

```bash
# 暂存指定文件
git add <file>

# 暂存所有修改（包括新文件）
git add -A

# 暂存所有修改（不包括删除的文件）
git add .

# 提交暂存的内容
git commit -m "提交信息"

# 查看工作区与暂存区的差异
git diff

# 查看暂存区与最新提交的差异
git diff --staged
```

## 撤销操作

```bash
# 将文件从暂存区移出（保留修改）
git restore --staged <file>

# 撤销工作区修改（恢复到上次提交的状态）
git restore <file>

# 撤销所有暂存
git reset HEAD

# 修改最近一次提交信息
git commit --amend -m "新的提交信息"

# 回退版本（保留修改）
git reset --soft HEAD~1

# 回退版本（丢弃修改）
git reset --hard HEAD~1
```

## 分支管理

```bash
# 创建新分支
git branch <branch-name>

# 切换到指定分支
git switch <branch-name>

# 创建并切换到新分支
git switch -c <branch-name>

# 删除本地分支
git branch -d <branch-name>

# 删除远程分支
git push origin --delete <branch-name>

# 合并分支
git merge <branch-name>
```

## 远程操作

```bash
# 推送当前分支到远程
git push

# 首次推送并设置上游分支
git push -u origin <branch-name>

# 强制推送（谨慎使用）
git push --force

# 拉取远程更新并合并
git pull

# 仅拉取不合并
git fetch
```

## 代理配置

```bash
# 设置代理（端口根据实际情况调整，7890 为 Clash 常见端口）
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy http://127.0.0.1:7890

# 取消代理
git config --global --unset http.proxy
git config --global --unset https.proxy

# 查看当前代理配置
git config --global --get http.proxy
```

## 凭据与认证

```bash
# 清除 Windows 凭据管理器中的 Git 凭据
git credential-manager erase

# 查看凭据管理器
git config --global credential.helper
```

## .gitignore

```bash
# 忽略构建产物（Keil MDK 项目示例）
project/mdk/Objects/
project/mdk/Listings/
*.o
*.axf
*.crf
*.dep
*.scvd
*.iex
*.build_log.htm
*.uvguix.*
```

## 实用场景

```bash
# 查看某个文件的修改历史
git log -p -- <file>

# 查看某人最近的提交
git log --author="用户名" --oneline

# 暂存当前修改，切换到其他分支处理紧急任务
git stash
git stash pop

# 比较两个分支的差异
git diff branch1..branch2

# 将某次提交应用到当前分支（挑选提交）
git cherry-pick <commit-hash>
```
