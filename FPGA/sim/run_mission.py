#!/usr/bin/env python3
# ═══════════════════════════════════════════════════════════
#   run_mission.py - בונה ומריץ את tb_mission לכל קבוצת פרמטרים
#   (Verilator 5). כל קבוצה מקבלת עותק של mission_fsm.v עם ערכי ברירת
#   מחדל אחרים (רק כשצריך - למשל מגבלת זמן קצרה); הקובץ המקורי לא משתנה.
#   שימוש:  python3 run_mission.py <תיקיית FPGA-Drone> <תיקיית עבודה>
# ═══════════════════════════════════════════════════════════
import os, re, shutil, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

SRC  = sys.argv[1] if len(sys.argv) > 1 else '..'
WORK = sys.argv[2] if len(sys.argv) > 2 else '/tmp/tbm'
RTL  = ['drone_top.v','uart_rx.v','uart_tx.v','cmd_parser.v','msp_builder.v',
        'flight_fsm.v','mission_rx.v','mission_fsm.v','mission_tx.v']

TOP_PARAMS = ()

GROUPS = {
  # שם: (פרמטרים של mission_fsm.v, defines, תרחישים)
  # שלב הבדיקה, ההמראה, המרחקים והתקרה נקבעים בזמן ריצה בשורת MSE של ה-tb.
  'main': ({}, [], [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56]),
  'tmax': ({'MISSION_MAX_S':20}, [], [29]),
}

def build(name, params, defines):
    d = os.path.join(WORK, name)
    shutil.rmtree(d, ignore_errors=True); os.makedirs(d)
    for f in RTL: shutil.copy(os.path.join(SRC, f), d)
    shutil.copy(os.path.join(SRC, 'sim', 'tb_mission.sv'), d)
    # פרמטרי הפרופיל נקבעים ב-drone_top.v ועוברים משם במפורש; השאר ב-mission_fsm.v
    for k, v in params.items():
        f = 'drone_top.v' if k in TOP_PARAMS else 'mission_fsm.v'
        p = os.path.join(d, f); s = open(p, encoding='utf-8').read()
        s, n = re.subn(r'(parameter\s+%s\s*=\s*)\d+' % k, r'\g<1>%d' % v, s)
        assert n == 1, (k, f, n)
        open(p, 'w', encoding='utf-8').write(s)
    cmd = ['verilator','--binary','--timing','-j','2','-Wno-fatal','-Wno-lint','-Wno-style',
           '--default-language','1364-2005','+1800-2017ext+sv','--top-module','tb_mission',
           'tb_mission.sv'] + RTL + ['-o','tbm'] + ['-D%s' % x for x in defines]
    r = subprocess.run(cmd, cwd=d, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:]); raise SystemExit('build failed: ' + name)
    return d

def run(d, scen):
    r = subprocess.run([os.path.join(d,'obj_dir','tbm'), '+SCEN=%d' % scen], cwd=d,
                       capture_output=True, text=True, timeout=1500)
    return scen, r.stdout

if __name__ == '__main__':
    os.makedirs(WORK, exist_ok=True)
    with ThreadPoolExecutor(4) as ex:
        dirs = dict(zip(GROUPS, ex.map(lambda g: build(g, GROUPS[g][0], GROUPS[g][1]), GROUPS)))
    jobs = [(g, s) for g in GROUPS for s in GROUPS[g][2]]
    total_err = 0
    with ThreadPoolExecutor(8) as ex:
        for (g, s), (scen, out) in zip(jobs, ex.map(lambda j: run(dirs[j[0]], j[1]), jobs)):
            open(os.path.join(WORK, 'scen%02d_%s.log' % (scen, g)), 'w').write(out)
            res = [l for l in out.splitlines() if 'PASS' in l or 'FAIL' in l or 'DONE' in l]
            m = re.search(r'errors=(\d+)', out)
            e = int(m.group(1)) if m else 99
            total_err += e
            print('── scen %2d (%s): %s' % (scen, g, 'OK' if e == 0 else 'ERRORS=%d' % e))
            for l in res:
                if 'FAIL' in l or e: print('   ', l)
    print('=== TOTAL errors=%d ===' % total_err)
