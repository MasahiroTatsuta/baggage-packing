#!/usr/bin/env bash
# bp_push.sh — results/ の新規ファイルとコード変更をスコープを絞ってpushする。
# 「過去分は遡って追加しない」方針(results-dir-tracking-policy)を守るため、
# results/ 全体の一括addはしない。呼び出し側で対象ファイルを明示すること。
#
# 使い方: bash scripts/bp_push.sh "<commit message>" <file1> [file2 ...]
#   例: bash scripts/bp_push.sh "phase39: ..." results/phase39_report.md \
#         results/phase39_baseline.json agents/mysolver/ordering.py
set -euo pipefail
cd "$(dirname "$0")/.."

MSG="${1:?usage: bp_push.sh \"<commit message>\" <file1> [file2 ...]}"
shift
FILES=("$@")
if [ "${#FILES[@]}" -eq 0 ]; then
  echo "エラー: 対象ファイルを1つ以上指定すること(results/の一括addは方針違反)" >&2
  exit 1
fi

git add -f "${FILES[@]}"
git status --short
git commit -m "$MSG"
git push

echo ""
echo "== push検証 =="
git fetch origin
echo "results/ 追跡ファイル数: $(git ls-tree -r origin/main --name-only | grep -c '^results/')"
for f in "${FILES[@]}"; do
  case "$f" in
    *.zip)
      local_sha=$(sha256sum "$f" | awk '{print $1}')
      remote_sha=$(git show "origin/main:$f" | sha256sum | awk '{print $1}')
      if [ "$local_sha" = "$remote_sha" ]; then
        echo "OK  $f (SHA256 $local_sha)"
      else
        echo "NG  $f: local=$local_sha remote=$remote_sha" >&2
      fi
      ;;
  esac
done
