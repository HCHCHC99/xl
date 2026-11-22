#!/bin/bash
# ============================================================================
#  backup.sh — 工程整目录全量备份
#
#  用法：  bash tools/backup.sh <阶段名>
#  示例：  bash tools/backup.sh veh_obj_fix
#          → D:\WorkBuddyBuffer\备份\JC_LW_BLDC_2026_11_12_18_51_58_veh_obj_fix
#
#  命名规范：  JC_LW_BLDC_<YYYY_MM_DD>_<HH_MM_SS>_<阶段名>
#             时间戳由 date 实时取，**不允许手填**（手填必然重复或写错）
#
#  为什么必须整目录全量：
#    只备份 applications/ 会漏掉 board/、rt-thread/ 补丁、.cproject、Debug/*.mk，
#    这些恰恰是"编译不过时最需要找回"的东西。约 3180 文件 / 59 MB，代价可接受。
#
#  ⚠️ 已知坑（本脚本已规避）：
#    1. Git Bash 会把命令行里的 Windows 反斜杠路径转成正斜杠，
#       导致 robocopy 报「无效参数 #3」并以 exit=16 失败。
#       ⇒ 本脚本用 cygpath -w 转成 Windows 路径，再交给 powershell 调用 robocopy。
#    2. robocopy 的退出码 0~7 都是**成功**（1=有文件被复制，2=有额外文件，3=两者都有），
#       只有 >=8 才是真失败。直接用 $? 判断会把成功当失败。
# ============================================================================
set -u

# ---------- 参数 ----------
STAGE="${1:-}"
if [ -z "$STAGE" ]; then
    echo "用法: bash tools/backup.sh <阶段名>"
    echo "示例: bash tools/backup.sh veh_obj_fix"
    exit 1
fi

# 阶段名含空格/特殊字符会让目录名难用，直接拒绝
case "$STAGE" in
    *[[:space:]]*|*/*|*\\*)
        echo "!! 阶段名不允许含空格或路径分隔符: '$STAGE'"
        exit 1
        ;;
esac

# ---------- 路径 ----------
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKUP_BASE="/d/WorkBuddyBuffer/备份"

if [ ! -d "$ROOT/applications" ]; then
    echo "!! 找不到工程根目录（$ROOT 下无 applications/）"
    exit 1
fi

# ---------- 时间戳（唯一来源，禁止手填）----------
TS="$(date +%Y_%m_%d_%H_%M_%S)"
NAME="JC_LW_BLDC_${TS}_${STAGE}"
DEST="$BACKUP_BASE/$NAME"

# ---------- 前置检查：同名目录已存在则拒绝（防覆盖）----------
if [ -e "$DEST" ]; then
    echo "!! 目标已存在，拒绝覆盖: $DEST"
    echo "   （同一秒内重复执行才会出现，请等 1 秒再试）"
    exit 1
fi

# ---------- 转 Windows 路径 ----------
SRC_W="$(cygpath -w "$ROOT")"
DEST_W="$(cygpath -w "$DEST")"

echo "=============================================================="
echo " 工程   : $SRC_W"
echo " 目标   : $DEST_W"
echo " 时间戳 : $TS"
echo "=============================================================="

# ---------- 执行 ----------
# /E    含空子目录
# /NFL /NDL /NJH /NJS  不打印文件/目录/头/汇总明细（只看结果）
# /R:1 /W:1            失败重试 1 次、等 1 秒（默认 100 万次/30 秒，卡死风险）
powershell -NoProfile -Command \
    "robocopy '$SRC_W' '$DEST_W' /E /NFL /NDL /NJH /NJS /R:1 /W:1 | Out-Null; exit \$LASTEXITCODE"
RC=$?

if [ "$RC" -ge 8 ]; then
    echo "!! robocopy 失败 (exit=$RC)  —— 0~7 才是成功，>=8 是真错"
    exit 1
fi

# ---------- 结果核对 ----------
if [ ! -d "$DEST" ]; then
    echo "!! 目标目录未生成"
    exit 1
fi

FILES=$(find "$DEST" -type f 2>/dev/null | wc -l | tr -d ' ')
SIZE=$(du -sm "$DEST" 2>/dev/null | cut -f1)

echo ""
echo ">>> 备份完成 (robocopy exit=$RC，0~7 均为成功)"
echo ">>> 目录 : $DEST_W"
echo ">>> 文件 : $FILES 个"
echo ">>> 大小 : ${SIZE} MB"
echo ""
echo "回滚提示（单文件）："
echo "  robocopy \"$DEST_W\\\\applications\\\\vehicle\" \"$SRC_W\\\\applications\\\\vehicle\" /E"
echo "回滚提示（整目录）："
echo "  先 robocopy 反向同步，再核对 git status"