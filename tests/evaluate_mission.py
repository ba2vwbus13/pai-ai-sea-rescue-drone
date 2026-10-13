#!/usr/bin/env python3
"""海難救助ミッションの評価: 報告書 JSON を、Gazebo から記録した本当の位置(truth_*.jsonl)と照合する。

要救助者は海流で動き続けるので、確定・投下の時刻における本当の位置を記録から補間して比べる。
  - 発見数(再現率)と誤った確定(適合率)
  - 確定時点の位置誤差、発見までの時間
  - 救命浮環の投下精度(着水後の浮環と要救助者の距離)
  - 漂流補正の効果(通報地点と、補正した捜索基点の、それぞれ要救助者の重心までの距離)
出力: 標準出力の表と、結果地図 PNG(報告書と同じ名前)。

実行: python3 evaluate_mission.py results/mission_XXXX.json [--truth results/truth_XXXX.jsonl]
"""
import argparse
import bisect
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

for name in ('Noto Sans CJK JP', 'IPAexGothic', 'Hiragino Sans'):
    if any(name in f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams['font.family'] = name
        break


class Truth:
    def __init__(self, path):
        self.t, self.poses = [], []
        for line in Path(path).read_text().splitlines():
            d = json.loads(line)
            self.t.append(d['t'])
            self.poses.append(d['poses'])

    def at(self, name, t):
        """時刻 t における name の (north, east)。記録の間は線形補間する。"""
        i = bisect.bisect_left(self.t, t)
        i = min(max(i, 1), len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        p0, p1 = self.poses[i - 1].get(name), self.poses[i].get(name)
        if p0 is None or p1 is None:
            return p1 or p0
        a = 0.0 if t1 == t0 else min(1.0, max(0.0, (t - t0) / (t1 - t0)))
        return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]))

    def names(self, prefix):
        return sorted({n for p in self.poses for n in p if n.startswith(prefix)})


def find_truth(report_path, rep):
    cands = sorted(Path(report_path).parent.glob('truth_*.jsonl'))
    best = None
    for c in cands:
        first = json.loads(c.open().readline())['t']
        if first <= rep['t_start']:
            best = c
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('report')
    ap.add_argument('--truth')
    ap.add_argument('--match', type=float, default=5.0, help='正解と対応づける距離 [m]')
    a = ap.parse_args()

    rep = json.loads(Path(a.report).read_text())
    truth = Truth(a.truth or find_truth(a.report, rep))
    persons = truth.names('person_')
    confirmed = [t for t in rep['targets'] if t['status'] == 'confirmed']
    rejected = [t for t in rep['targets'] if t['status'] == 'rejected']

    # 確定時点の推定位置(est_*)と、同時刻の本当の位置を比べて対応づける
    cand = []
    for i, t in enumerate(confirmed):
        for p in persons:
            tn, te = truth.at(p, t['est_t'])
            cand.append((math.hypot(t['est_north'] - tn, t['est_east'] - te), i, p))
    pairs, used_t, used_p = [], set(), set()
    for d, i, p in sorted(cand):
        if d <= a.match and i not in used_t and p not in used_p:
            pairs.append((i, p, d))
            used_t.add(i)
            used_p.add(p)
    tp, fp, fn = len(pairs), len(confirmed) - len(pairs), len(persons) - len(pairs)

    print(f'要救助者(正解)       : {len(persons)} 名')
    print(f'確定 (うち正解 / 誤り) : {len(confirmed)} ({tp} / {fp})')
    print(f'見逃し                : {fn}  {[p for p in persons if p not in used_p]}')
    print(f'棄却した候補          : {len(rejected)}')
    print(f'再現率 {tp / len(persons):.0%}   適合率 {tp / max(1, len(confirmed)):.0%}')

    ring_err = []
    for i, p, d in sorted(pairs, key=lambda x: confirmed[x[0]]['confirm_t']):
        t = confirmed[i]
        found = t['confirm_t'] - rep['t_start']
        msg = f'  {p:9s} ← {t["id"]:4s} 確定 {found:5.0f} s  位置誤差 {d:4.2f} m'
        if t.get('drop'):
            dr = t['drop']
            tl = dr['t'] + 2.0                       # 着水後(落下 約 1 秒 + 余裕)
            rp = truth.at(dr['ring'], tl)
            pp = truth.at(p, tl)
            if rp and pp:
                e = math.hypot(rp[0] - pp[0], rp[1] - pp[1])
                ring_err.append(e)
                msg += f'  浮環との距離 {e:4.2f} m'
        print(msg)
    if pairs:
        errs = [d for _, _, d in pairs]
        print(f'位置誤差 平均 {sum(errs) / len(errs):.2f} m / 最大 {max(errs):.2f} m')
    if ring_err:
        print(f'救命浮環と要救助者の距離 平均 {sum(ring_err) / len(ring_err):.2f} m / '
              f'最大 {max(ring_err):.2f} m(浮環の届く範囲の目安 2 m 以内: '
              f'{sum(e <= 2.0 for e in ring_err)}/{len(ring_err)})')

    # 漂流補正の効果: 捜索開始時の要救助者の重心までの距離
    if rep.get('search_center'):
        ts = rep['t_start']
        pts = [truth.at(p, ts) for p in persons]
        cn, ce = sum(q[0] for q in pts) / len(pts), sum(q[1] for q in pts) / len(pts)
        d_datum = math.hypot(rep['datum']['n'] - cn, rep['datum']['e'] - ce)
        sc = rep['search_center']
        d_center = math.hypot(sc[0] - cn, sc[1] - ce)
        print(f'漂流補正: 要救助者の重心まで 通報地点から {d_datum:.1f} m → 補正した基点から {d_center:.1f} m')
    print(f'飛行時間 {rep["elapsed"]:.0f} s   飛行距離 {rep["dist_flown"]:.0f} m')

    # ---- 地図 --------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    ax.axhspan(-3, 30, color='#e8dcb5', zorder=0)
    ax.set_facecolor('#d6e8f5')
    r = rep['route']
    if r:
        ax.plot([p[1] for p in r], [p[0] for p in r], '--', c='gray', lw=0.8, label='拡大方形捜索(予定)')
    tr = rep['trail']
    ax.plot([p[1] for p in tr], [p[0] for p in tr], c='orange', lw=1.3, label='飛行軌跡')
    for k, p in enumerate(persons):
        seg = [truth.at(p, t) for t in truth.t[::4]]
        ax.plot([q[1] for q in seg], [q[0] for q in seg], c='blue', lw=1.0, alpha=0.6,
                label='要救助者の漂流(正解)' if k == 0 else None)
        end = seg[-1]
        ax.annotate(p.replace('person_', ''), (end[1], end[0]), xytext=(4, -10),
                    textcoords='offset points', color='blue', fontsize=8)
    ax.scatter([t['est_east'] for t in confirmed], [t['est_north'] for t in confirmed],
               c='red', s=40, zorder=5, label='AI が確定(確定時点)')
    rings = truth.names('lifering_')
    if rings:
        last = [truth.at(n, truth.t[-1]) for n in rings]
        ax.scatter([q[1] for q in last], [q[0] for q in last], s=70, facecolors='none',
                   edgecolors='#ff6a00', linewidths=2.5, zorder=6, label='救命浮環(最終位置)')
    if rejected:
        ax.scatter([t['est_east'] for t in rejected], [t['est_north'] for t in rejected],
                   marker='x', c='gray', s=40, label='誤検出として棄却')
    ax.plot(rep['datum']['e'], rep['datum']['n'], 'X', c='#d4a000', ms=11, label='通報地点')
    if rep.get('search_center'):
        sc = rep['search_center']
        ax.annotate('', (sc[1], sc[0]), (rep['datum']['e'], rep['datum']['n']),
                    arrowprops=dict(arrowstyle='->', color='#d4a000', lw=1.8))
        ax.plot(sc[1], sc[0], 'o', c='#d4a000', ms=6, label='漂流補正した捜索基点')
    ax.plot(0, 0, 's', c='k', ms=7, label='離陸地点(砂浜)')
    ax.set_xlabel('東 [m]')
    ax.set_ylabel('北 [m]')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_title(f'海難救助の結果: {len(persons)} 名中 {tp} 名を発見・救命浮環 {len(ring_err)} 個を投下')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=3, fontsize=8)
    fig.tight_layout()
    out = Path(a.report).with_suffix('.png')
    fig.savefig(out, dpi=150)
    print('地図:', out)


if __name__ == '__main__':
    main()
