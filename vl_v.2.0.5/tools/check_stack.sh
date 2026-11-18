#!/bin/bash
# ============================================================================
#  check_stack.sh — 线程栈尺寸自动核算
#
#  用途：每次改动线程栈 / 新增线程 / 修改热路径函数后，跑一遍确认余量。
#        静态分析思路：反汇编整个 ELF，构建调用图，算出每个线程入口的
#        最坏静态栈深，再叠加中断/切换开销，与实际分配值对比。
#
#  用法：  bash tools/check_stack.sh            # 只核算
#          bash tools/check_stack.sh --build    # 先编译再核算
#
#  输出：  每个线程的 normal / with-print / peak / alloc / 使用率 / 判定
#          以及 finsh 命令的函数指针盲区检查
#
#  ⚠️ 局限：静态调用图看不到函数指针调用（pack 回调、状态机、MSH 命令、
#     bg_modules 轮询模块）。脚本已对 MSH 命令和 pack 回调单独处理，
#     但若新增大局部数组的回调，仍需用 MSH `t_stack` 实测复核。
# ============================================================================
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TC="D:/RT-ThreadStudio/platform/env_released/env/tools/gnu_gcc/arm_gcc/mingw/bin"
MAKE="D:/RT-ThreadStudio/platform/env_released/env/tools/bin/make.exe"
ELF="$ROOT/Debug/rtthread.elf"
WORK="$ROOT/build_tmp"

mkdir -p "$WORK"

BUILD=0
[ "${1:-}" = "--build" ] && BUILD=1

if [ $BUILD -eq 1 ]; then
    echo ">>> 编译中 ..."
    export PATH="$TC:$(dirname "$MAKE"):$PATH"
    ( cd "$ROOT/Debug" && "$MAKE" all > "$WORK/build.log" 2>&1 )
    rc=$?
    if [ $rc -ne 0 ]; then
        echo "!! 编译失败 (exit=$rc)，详见 $WORK/build.log"
        grep -iE "error:|undefined reference|multiple definition" "$WORK/build.log" | head -20
        exit 1
    fi
    echo ">>> 编译成功"
fi

if [ ! -f "$ELF" ]; then
    echo "!! 找不到 $ELF，请先编译"
    exit 1
fi

echo ">>> 反汇编 $ELF ..."
"$TC/arm-none-eabi-objdump.exe" -d --no-show-raw-insn "$ELF" > "$WORK/full.dis" 2>&1

cd "$WORK" || exit 1
python stackana.py  | tail -3
echo
python verify.py
echo
python finsh_check.py