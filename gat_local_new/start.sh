#!/usr/bin/env bash
# Only starts OpenCode; model and authentication are inherited from OpenCode.
set -euo pipefail
gat_local_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
command -v opencode >/dev/null || { echo 'OpenCode未在PATH中，请先安装/配置OpenCode。' >&2; exit 1; }
[[ -d "$gat_local_root/.git" ]] || { echo '缺少独立.git边界，先在本目录git init。' >&2; exit 1; }
cd "$gat_local_root"
exec opencode "$@"
