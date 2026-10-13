#!/usr/bin/env python3
"""動画用のタイトル画面・まとめ画面・倍速表示を PNG で作る(ffmpeg に drawtext が無いため)。

実行: python3 make_cards.py OUT_DIR REPORT.json [MAP.png] [SPEED]
まとめ画面の数値は tests/evaluate_mission.py と同じ照合(本当の位置の記録)から計算する。
"""
import json
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
out = Path(sys.argv[1])
rep_path = Path(sys.argv[2])
rep = json.loads(rep_path.read_text())
map_png = Path(sys.argv[3]) if len(sys.argv) > 3 else None
speed = sys.argv[4] if len(sys.argv) > 4 else '2'
speed = speed.rstrip('0').rstrip('.') if '.' in speed else speed
out.mkdir(parents=True, exist_ok=True)
fp = subprocess.run(['fc-match', '-f', '%{file}', 'Noto Sans CJK JP:bold'],
                    capture_output=True, text=True).stdout.strip()
F = {s: ImageFont.truetype(fp, s) for s in (28, 36, 44, 64, 84)}
BG, TX, SUB, ACC = (16, 30, 48), (240, 240, 240), (170, 185, 200), (255, 140, 60)

# 評価(浮環の距離など)は evaluate_mission.py の出力をそのまま使う
ev = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / 'tests/evaluate_mission.py'),
                     str(rep_path)], capture_output=True, text=True).stdout
ev_lines = {k: line for line in ev.splitlines() for k in ('救命浮環と', '漂流補正', '位置誤差 平均')
            if line.startswith(k)}


def center(d, y, text, font, fill):
    w = d.textlength(text, font=font)
    d.text(((W - w) / 2, y), text, font=font, fill=fill)


# ---- タイトル --------------------------------------------------------------
im = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(im)
center(d, 220, 'AI 海難救助ドローン', F[84], TX)
center(d, 350, '転覆事故の漂流者を AI で見つけ、救命浮環を届ける', F[44], ACC)
items = ['想定: 名護市沖で釣り船が転覆、乗員 5 名。通報から 10 分後に砂浜から出動',
         '漂流予測(海流 × 経過時間)→ 拡大方形捜索(IAMSAR)',
         'YOLO11m のタイル分割推論で人を発見 → 降下して再確認 → 救命浮環を投下',
         'PX4 SITL + Gazebo Harmonic + ROS 2 Humble(オフボード制御)']
for i, t in enumerate(items):
    center(d, 490 + i * 70, t, F[36], SUB)
center(d, 900, f'本編は {speed} 倍速で再生しています', F[36], TX)
im.save(out / 'title.png')

# ---- まとめ ------------------------------------------------------------------
im = Image.new('RGB', (W, H), BG)
d = ImageDraw.Draw(im)
d.text((80, 60), '結果', font=F[64], fill=TX)
conf = [t for t in rep['targets'] if t['status'] == 'confirmed']
rej = [t for t in rep['targets'] if t['status'] == 'rejected']
lines = [
    (f'乗員 {rep["pob"]} 名中 {len(conf)} 名を発見し、救命浮環 {rep["n_drops"]} 個を投下', F[36], ACC),
    (f'誤検出の候補 {len(rej)} 件は、降下して再確認し棄却', F[36], TX),
    (f'飛行時間 {rep["elapsed"]:.0f} 秒 / 飛行距離 {rep["dist_flown"]:.0f} m', F[36], TX),
]
for k in ('救命浮環と', '位置誤差 平均', '漂流補正'):
    if k in ev_lines:
        t = ev_lines[k].replace('救命浮環と要救助者の距離', '浮環と要救助者の距離').split('(')[0]
        t = t.replace('漂流補正: 要救助者の重心まで 通報地点から', '漂流補正: 捜索の中心と要救助者の距離')
        t = t.replace(' → 補正した基点から ', ' → ')
        lines.append((t, F[28], SUB))
y = 180
for t, f, c in lines:
    d.text((80, y), t, font=f, fill=c)
    y += 70 if f is not F[28] else 50
d.text((80, 980), 'コード: GitHub(説明欄)  /  PAI 最終課題(海難救助版)', font=F[28], fill=SUB)
if map_png and map_png.exists():
    m = Image.open(map_png).convert('RGB')
    r = min(900 / m.height, 820 / m.width)
    m = m.resize((int(m.width * r), int(m.height * r)))
    im.paste(m, (W - m.width - 50, (H - m.height) // 2))
im.save(out / 'summary.png')

# ---- 倍速バッジ(透過 PNG)----------------------------------------------------
b = Image.new('RGBA', (200, 56), (0, 0, 0, 0))
d = ImageDraw.Draw(b)
d.rounded_rectangle((0, 0, 199, 55), 12, fill=(220, 60, 40, 230))
label = f'{speed}倍速'
d.text(((200 - d.textlength(label, font=F[36])) / 2, 6), label, font=F[36], fill=(255, 255, 255, 255))
b.save(out / 'speed_badge.png')
print('cards:', out)
