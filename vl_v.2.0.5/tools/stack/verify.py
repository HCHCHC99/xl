import json, os, re, math

import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
WORK = os.path.join(ROOT, 'build_tmp')
funcs = json.load(open(os.path.join(WORK, 'cg.json')))
for n in ('rt_list_init', 'rt_list_isempty', 'rt_list_remove'):
    if n in funcs:
        funcs[n]['frame'] = 8

PRINTF = {
    'rt_kprintf', 'rt_vsnprintf', 'skip_atoi', 'print_number', 'divide',
    '_vfiprintf_r', '_svfprintf_r', '_dtoa_r', 'fiprintf', 'fputs',
    'SEGGER_RTT_vprintf', 'rt_hw_console_output', '__assert_func',
    'rt_assert_handler', 'abort', 'raise', '_exit', '_write',
    'rt_device_write', 'ulog_output', 'snprintf', 'rt_snprintf',
    'vsnprintf', '_vsnprintf_r', '__sfputs_r', 'malloc', '_malloc_r',
    'rt_malloc', 'rt_malloc_align',
}

def frame_of(name):
    return funcs[name]['frame'] if name in funcs else 0

# ---- 从源码读取真实分配值 ----
def read_rtconfig(key):
    txt = open(os.path.join(ROOT, 'rtconfig.h'), encoding='utf-8', errors='replace').read()
    m = re.search(r'#define\s+%s\s+(\d+)' % key, txt)
    return int(m.group(1)) if m else None

def read_macro(path, key):
    txt = open(os.path.join(ROOT, path), encoding='utf-8', errors='replace').read()
    m = re.search(r'#define\s+%s\s+(\d+)' % key, txt)
    return int(m.group(1)) if m else None

def read_create(path, tname):
    txt = open(os.path.join(ROOT, path), encoding='utf-8', errors='replace').read()
    m = re.search(r'rt_thread_create\(\s*"%s".*?,\s*(\d+)\s*,\s*\d+\s*,\s*\d+\s*\)' % tname, txt, re.S)
    return int(m.group(1)) if m else None

allocs = {
    'idle':       read_rtconfig('IDLE_THREAD_STACK_SIZE'),
    'main':       read_rtconfig('RT_MAIN_THREAD_STACK_SIZE'),
    'finsh':      read_rtconfig('FINSH_THREAD_STACK_SIZE'),
    'can_tx':     read_macro('applications/CAN/can_tx.c', 'CAN_TX_THREAD_STACK'),
    'keyboard':   read_macro('applications/keyboard/keyboard_sm.c', 'KB_THREAD_STACK'),
    'can_parser': read_create('applications/CAN/ut_can_scheduler.c', 'can_parser'),
    'app_main':   read_create('applications/boot/boot_table.c', 'app_main'),
    'bg_sched':   read_create('applications/boot/boot_table.c', 'bg_sched'),
    'ctrl_loop':  read_macro('applications/ctrl_loop/ctrl_loop_init.c', 'CL_THREAD_STACK'),
}

entries = {
    'idle': 'rt_thread_idle_entry', 'main': 'main_thread_entry',
    'can_parser': 'can_parser_thread_entry', 'keyboard': '_kb_thread_entry',
    'can_tx': '_tx_thread_entry', 'app_main': 'app_main_thread_entry',
    'bg_sched': 'bg_scheduler_thread_entry', 'finsh': 'finsh_thread_entry',
    'ctrl_loop': '_cl_thread_entry',
}

def worst(name, block, memo, on):
    key = (name, block)
    if key in memo: return memo[key]
    if name not in funcs: return 0
    if block and name in PRINTF: return 0
    if name in on: return 0
    on.add(name)
    best = 0
    for c in funcs[name]['calls']:
        if c == name: continue
        v = worst(c, block, memo, on)
        if v > best: best = v
    on.discard(name)
    tot = funcs[name]['frame'] + best
    memo[key] = tot
    return tot

# 中断/切换开销：硬件压栈104(r0-r3,r12,lr,pc,xPSR) + 切帧r4-r11=32 + FPU flag=4
# 取保守值 240（含对齐与余量）
OVERHEAD = 240

print('%-12s %8s %8s %8s %8s %8s  %s' % ('thread', 'normal', 'w/print', 'peak', 'alloc', 'use%', 'verdict'))
print('-' * 84)
ok = True
for label in ('idle', 'main', 'can_parser', 'keyboard', 'can_tx', 'app_main',
              'bg_sched', 'ctrl_loop', 'finsh'):
    entry = entries[label]
    alloc = allocs[label]
    if entry not in funcs or alloc is None:
        print('%-12s  (missing: entry=%s alloc=%s)' % (label, entry in funcs, alloc))
        continue
    normal = worst(entry, True, {}, set())
    full = worst(entry, False, {}, set())
    peak = full + OVERHEAD
    use = peak * 100.0 / alloc
    if peak > alloc:
        verdict = '*** OVERFLOW ***'; ok = False
    elif use > 80:
        verdict = 'TIGHT'; ok = False
    elif use > 65:
        verdict = 'ok-thin'
    else:
        verdict = 'ok'
    print('%-12s %8d %8d %8d %8d %7.0f%%  %s' % (label, normal, full, peak, alloc, use, verdict))

print('-' * 84)
tot = sum(v for v in allocs.values() if v)
print('全部线程栈合计: %d B' % tot)
print('结论: %s' % ('全部通过' if ok else '存在风险项，见上'))