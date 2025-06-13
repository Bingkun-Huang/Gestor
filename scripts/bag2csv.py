#!/usr/bin/env python3
import csv, re, sys, pathlib

RAW = 'joint_states_raw.csv'   # 先用 rostopic echo -p ... 生成的文件
OUT = 'joint_trajectory_2.csv'

ARM_JOINTS = [f'panda_joint{i}' for i in range(1,8)]

# ---- 1. 读取首行，建立 “关节名 ➜ position 列” 映射 ----
with open(RAW) as f:
    reader = csv.reader(f)
    header = next(reader)

# header 有形如 field.name0, field.name1, ...  以及 field.position0…
name_cols = [c for c in header if c.startswith('field.name')]
pos_cols  = [c for c in header if c.startswith('field.position')]

if len(name_cols) != len(pos_cols):
    sys.exit('❌ name 列与 position 列数量不一致，检查文件！')

# name 列内保存实际关节名，需先读取一行才能知道映射
with open(RAW) as f:
    reader = csv.DictReader(f)
    first_row = next(reader)

name_map = {first_row[nc]: pc for nc, pc in zip(name_cols, pos_cols)}
print('✔ 建立映射：', name_map)

missing = [j for j in ARM_JOINTS if j not in name_map]
if missing:
    sys.exit(f'❌ 找不到这些关节列: {missing}')

# ---- 2. 再次遍历文件，写出简洁 CSV ----
with open(RAW) as f_in, open(OUT, 'w', newline='') as f_out:
    reader = csv.DictReader(f_in)
    writer = csv.writer(f_out)
    writer.writerow(['time'] + ARM_JOINTS)

    t0 = None
    for row in reader:
        if t0 is None:
            t0 = float(row['%time'])
        trel = float(row['%time']) - t0
        q = [float(row[name_map[j]]) for j in ARM_JOINTS]
        writer.writerow([f'{trel:.6f}'] + [f'{v:.6f}' for v in q])

print('✅ 已生成', OUT, f'({pathlib.Path(OUT).stat().st_size/1024:.1f} KB)')
