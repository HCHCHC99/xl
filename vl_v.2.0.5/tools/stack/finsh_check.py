import json, os, re

import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
WORK = os.path.join(ROOT, 'build_tmp')
funcs = json.load(open(os.path.join(WORK, 'cg.json')))
for n in ('rt_list_init', 'rt_list_isempty', 'rt_list_remove'):
    if n in funcs:
        funcs[n]['frame'] = 8

memo = {}
def worst(name, on=None):
    if name in memo: return memo[name]
    if name not in funcs: return 0
    if on is None: on = set()
    if name in on: return 0
    on.add(name)
    b = 0
    for c in funcs[name]['calls']:
        if c != name:
            v = worst(c, on)
            if v > b: b = v
    on.discard(name)
    t = funcs[name]['frame'] + b
    memo[name] = t
    return t

# MSH 命令通过函数表执行 —— 静态调用图完全看不到
pat = re.compile(r'^(t_|cmd_)')
cmds = sorted(((worst(c), c) for c in funcs if pat.match(c)), reverse=True)

base_finsh = worst('finsh_thread_entry')
print('finsh 基础链              : %d B' % base_finsh)
print('最耗栈命令                : %d B  (%s)' % cmds[0])
print('最耗栈命令前 5 名:')
for v, n in cmds[:5]:
    print('    %-40s %5d B' % (n, v))

# msh_exec 通过指针调用命令，前面还有 shell 解析链。
# 保守：基础链 + 最坏命令（不考虑重叠）作为峰值上界
peak = base_finsh + cmds[0][0]
print('\nfinsh 峰值上界(基础+最坏命令): %d B' % peak)
print('当前分配                       : 3072 B')
print('结论: %s' % ('余量充足' if peak * 100.0 / 3072 < 80 else '需上调'))