import re, json, sys, os

import os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
WORK = os.path.join(ROOT, 'build_tmp')
path = os.path.join(WORK, 'full.dis')

funcs = {}
cur = None
line_re = re.compile(r'^([0-9a-f]+) <(.+)>:')
ins_re = re.compile(r'^\s+[0-9a-f]+:\s+(\S+)\s*(.*)$')

for line in open(path, encoding='utf-8', errors='replace'):
    m = line_re.match(line)
    if m:
        cur = m.group(2)
        funcs.setdefault(cur, {'frame': 0, 'calls': [], 'indirect': False})
        continue
    if cur is None:
        continue
    m = ins_re.match(line)
    if not m:
        continue
    op, args = m.group(1), m.group(2)
    base = op.split('.')[0]
    if base == 'sub' and re.match(r'^sp,\s*#', args):
        v = re.match(r'^sp,\s*#(\d+)', args)
        if v:
            funcs[cur]['frame'] += int(v.group(1))
    elif base == 'push':
        regs = re.findall(r'\b(r\d+|lr|sl|fp|ip|sb)\b', args)
        funcs[cur]['frame'] += 4 * len(regs)
    elif base in ('bl',):
        t = args.split('<')[-1].rstrip('>').strip() if '<' in args else None
        if t and re.match(r'^[A-Za-z_.$][\w.$]*$', t):
            funcs[cur]['calls'].append(t)
    elif base in ('blx', 'bx'):
        if '<' not in args:
            funcs[cur]['indirect'] = True

json.dump(funcs, open(os.path.join(WORK, 'cg.json'), 'w'))
print('functions parsed :', len(funcs))
print('total frame bytes:', sum(f['frame'] for f in funcs.values()))

# 最深的若干函数（静态帧）
top = sorted(funcs.items(), key=lambda kv: -kv[1]['frame'])[:25]
print('\n== 静态帧最大的 25 个函数 ==')
for n, f in top:
    print('  %-46s %6d B  calls=%d%s' % (n, f['frame'], len(f['calls']),
                                         '  INDIRECT' if f['indirect'] else ''))