#!/bin/bash
# 推送 robot_ws 到 GitHub (liu-big/robot_ws)
# 首次使用：先把 SSH 公钥添加到 https://github.com/settings/keys
set -euo pipefail

GITHUB_USER="${GITHUB_USER:-liu-big}"
REPO_NAME="${REPO_NAME:-robot_ws}"
REMOTE="git@github.com:${GITHUB_USER}/${REPO_NAME}.git"

echo "=== GitHub 推送: ${GITHUB_USER}/${REPO_NAME} ==="

if ! ssh -o BatchMode=yes -T git@github.com 2>&1 | grep -qi 'successfully authenticated'; then
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

# 远程仓库不存在时尝试创建（需 gh 已登录）
if command -v gh &>/dev/null && gh auth status &>/dev/null; then
  if ! gh repo view "${GITHUB_USER}/${REPO_NAME}" &>/dev/null; then
    echo "创建远程仓库 ${GITHUB_USER}/${REPO_NAME} ..."
    gh repo create "${REPO_NAME}" --private --source=. --remote=origin --push
    exit 0
  fi
fi

echo "推送到 origin main ..."
git push -u origin main
echo "完成: https://github.com/${GITHUB_USER}/${REPO_NAME}"
