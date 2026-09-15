#!/bin/bash
# 推送 robot_ws 到 GitHub (liu-big/robot_ws)
# 首次使用：先把 SSH 公钥添加到 https://github.com/settings/keys
set -euo pipefail

GITHUB_USER="${GITHUB_USER:-liu-big}"
REPO_NAME="${REPO_NAME:-robot_ws}"
REMOTE="git@github.com:${GITHUB_USER}/${REPO_NAME}.git"

echo "=== GitHub 推送: ${GITHUB_USER}/${REPO_NAME} ==="

if ! ssh -o BatchMode=yes -T git@github.com 2>&1 | grep -qiE 'successfully authenticated|Hi '; then
  echo
  echo "[!] SSH 尚未授权。请把下面公钥添加到 GitHub："
  echo "    https://github.com/settings/keys  →  New SSH key"
  echo
  cat ~/.ssh/id_ed25519_github.pub
  echo
  echo "添加完成后重新运行: $0"
  exit 1
fi

cd "$(dirname "$0")/.."

if ! git remote get-url origin &>/dev/null; then
  git remote add origin "$REMOTE"
else
  git remote set-url origin "$REMOTE"
fi

if ! git rev-parse HEAD &>/dev/null; then
  git add -A
  git commit -m "Initial commit: robot_ws quadruped control stack"
fi

if ! curl -fsS "https://api.github.com/repos/${GITHUB_USER}/${REPO_NAME}" &>/dev/null; then
  echo
  echo "[!] 远程仓库尚未创建。请先在浏览器打开（登录 ${GITHUB_USER}）："
  echo "    https://github.com/new?name=${REPO_NAME}&description=Quadruped+robot+ROS2+workspace&private=true"
  echo
  echo "    不要勾选 README / .gitignore，创建空仓库后重新运行: $0"
  exit 1
fi

echo "推送到 origin main ..."
git push -u origin main
echo "完成: https://github.com/${GITHUB_USER}/${REPO_NAME}"
