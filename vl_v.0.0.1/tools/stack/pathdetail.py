import json, os

import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
WORK = os.path.join(ROOT, 'build_tmp')
funcs = json.load(open(os.path.join(WORK, 'cg.json')))
for n in ('rt_list_init', 'rt_list_isempty', 'rt_list_remove'):
    if n in funcs:
        funcs[n]['frame'] = 8

# 打印/断言族——判定"是否为出错路径"
PRINTF = {
    'rt_kprintf', 'rt_vsnprintf', 'skip_atoi', 'print_number', 'divide',
    '_vfiprintf_r', '_svfprintf_r', '_dtoa_r', 'fiprintf', 'fputs',
    'SEGGER_RTT_vprintf', 'rt_hw_console_output', '__assert_func',
    'rt_assert_handler', 'abort', 'raise', '_exit', '_write',
    'rt_device_write', 'ulog_output', 'snprintf', 'rt_snprintf',
    'vsnprintf', '_vsnprintf_r', '__sfputs_r', 'malloc', '_malloc_r',
    'rt_malloc', 'rt_malloc_align',
}

def path(name, block, memo, on, trace):
    key = (name, block)
    if key in memo: return memo[key]
    if name not in funcs: return 0
    if block and name in PRINTF: return 0
    if name in on: return 0
    on.add(name)
    best, bestc = 0, None
    for c in funcs[name]['calls']:
        if c == name: continue
        v = path(c, block, memo, on, trace)
        if v > best: best, bestc = v, c
    on.discard(name)
    t = funcs[name]['frame'] + best
    memo[key] = t
    trace[name] = (bestc, t)
    return t

def trace_chain(root, block):
    """找出最耗栈的那条链"""
    memo, trace = {}, {}
    path(root, block, memo, set(), trace)
    chain, cur = [], root
    while cur and cur in trace:
        chain.append(cur)
        nxt, _ = trace[cur]
        if nxt is None: break
        cur = nxt
    return chain

for label, root in (('keyboard', '_kb_thread_entry'), ('can_tx', '_tx_thread_entry')):
    print('=' * 76)
    print('%s 含打印路径最耗栈的链：' % label)
    memo = {}
    total = path(root, False, memo, set(), {})
    chain = trace_chain(root, False)
    for f in chain:
        mark = '  <== 断言/打印' if f in PRINTF else ''
        print('   %-44s %5d B%s' % (f, funcs[f]['frame'], mark))
    print('   累计: %d B' % total)
    print()