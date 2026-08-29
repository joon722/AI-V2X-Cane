# -*- coding: utf-8 -*-
"""현장 결과 페이지(docs/field-results*.md)의 SVG 차트를 data/의 원본 CSV에서 재생성.

실행:  python scripts/figures/make_figures.py
출력:  docs/images/approach-timeline.svg   (실측 접근 한 건의 초 단위 판정)
       docs/images/distance-risk.svg       (전체 세션 거리별 위험등급 분포)

가공 없이 원본 CSV만 읽어 그린다. 히어로 이벤트는 재현 안정성을 위해 고정.
"""
import csv, glob, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "docs", "images")
os.makedirs(OUT, exist_ok=True)

# 고정 히어로 이벤트 (2026-08-17 12:22 세션의 접근 한 건)
HERO_FILE = os.path.join(DATA, "젯슨로그_20260817", "risk_tx_20260817_122247.csv")
HERO_SLICE = (343, 382)  # [start, end)

LVC = {0: '#16a34a', 1: '#d9a406', 2: '#ea580c', 3: '#dc2626'}
LVTINT = {0: '#e9f7ef', 1: '#fdf6d6', 2: '#fdecdd', 3: '#fbe0e0'}
LVNAME = {0: '안전', 1: '주의', 2: '경고', 3: '위험'}
FONT = '-apple-system,BlinkMacSystemFont,Segoe UI,Malgun Gothic,Apple SD Gothic Neo,sans-serif'


def fnum(x):
    try:
        return float(x)
    except Exception:
        return None


def load(f):
    with open(f, newline='', encoding='utf-8', errors='replace') as fh:
        return list(csv.DictReader(fh))


# ----------------------------------------------------------------------------
def make_hero():
    rows = load(HERO_FILE)[HERO_SLICE[0]:HERO_SLICE[1]]
    t0 = fnum(rows[0]['pc_time'])
    T = [fnum(r['pc_time']) - t0 for r in rows]
    D = [fnum(r['distance_m']) for r in rows]
    L = [int(float(r.get('effective_level') or 0)) for r in rows]
    TTC = [fnum(r.get('ttc_s')) for r in rows]

    W, H = 940, 470
    PL, PR, PT, PB = 92, 40, 92, 96
    px0, px1, py0, py1 = PL, W - PR, PT, H - PB
    tmax, dmax = max(T), 34.0

    def X(t):
        return px0 + (t / tmax) * (px1 - px0)

    def Y(d):
        return py0 + (1 - min(d, dmax) / dmax) * (py1 - py0)

    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="{FONT}">',
         f'<rect width="{W}" height="{H}" rx="14" fill="#ffffff"/>',
         f'<rect width="{W}" height="{H}" rx="14" fill="none" stroke="#e2e8f0"/>',
         f'<text x="{PL}" y="34" font-size="21" font-weight="700" fill="#0f172a">실제 접근 테스트 · 위험도 실시간 판정</text>',
         f'<text x="{PL}" y="58" font-size="13" fill="#64748b">2026-08-17 12:22 현장 로그 · 차량이 32 m에서 접근 → 등급 상승 → 정지 후 자동 해제 (실측, 가공 없음)</text>']
    # level bands
    seg = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or L[i] != L[seg]:
            lv = L[seg]
            xa = max(X(T[seg] - (0.5 if seg > 0 else 0)), px0)
            xb = min(X(T[i - 1] + (0.5 if i < len(rows) else 0)), px1)
            s.append(f'<rect x="{xa:.1f}" y="{py0}" width="{max(0,xb-xa):.1f}" height="{py1-py0}" fill="{LVTINT[lv]}"/>')
            seg = i
    s.append(f'<rect x="{px0}" y="{py0}" width="{px1-px0}" height="{py1-py0}" fill="none" stroke="#cbd5e1"/>')
    for dv in [0, 5, 10, 15, 20, 25, 30]:
        y = Y(dv)
        s.append(f'<line x1="{px0}" y1="{y:.1f}" x2="{px1}" y2="{y:.1f}" stroke="#eef2f6"/>')
        s.append(f'<text x="{px0-10}" y="{y+4:.1f}" font-size="12" fill="#94a3b8" text-anchor="end">{dv} m</text>')
    for tv in [0, 5, 10, 15, 20, 25, 30, 35]:
        if tv > tmax:
            continue
        x = X(tv)
        s.append(f'<line x1="{x:.1f}" y1="{py1}" x2="{x:.1f}" y2="{py1+5}" stroke="#cbd5e1"/>')
        s.append(f'<text x="{x:.1f}" y="{py1+22}" font-size="12" fill="#94a3b8" text-anchor="middle">{tv}s</text>')
    s.append(f'<text x="{(px0+px1)/2:.0f}" y="{H-52}" font-size="12.5" fill="#475569" text-anchor="middle">경과 시간</text>')
    cy = (py0 + py1) / 2
    s.append(f'<text x="26" y="{cy:.0f}" font-size="12.5" fill="#475569" text-anchor="middle" transform="rotate(-90 26 {cy:.0f})">차량–보행자 거리</text>')
    pts = ' '.join(f'{X(T[i]):.1f},{Y(D[i]):.1f}' for i in range(len(rows)))
    s.append(f'<polyline points="{pts}" fill="none" stroke="#0f172a" stroke-width="2.6" stroke-linejoin="round" stroke-linecap="round"/>')
    for i in range(len(rows)):
        s.append(f'<circle cx="{X(T[i]):.1f}" cy="{Y(D[i]):.1f}" r="3.1" fill="{LVC[L[i]]}"/>')
    first = {}
    for i, lv in enumerate(L):
        if lv >= 1 and lv not in first:
            first[lv] = i
    lv3i = first.get(3)
    clear_i = next((i for i in range(lv3i, len(rows)) if L[i] == 0), None) if lv3i is not None else None
    for i in [first.get(1), first.get(2), first.get(3), clear_i]:
        if i is None:
            continue
        col = '#0f766e' if i == clear_i else LVC[L[i]]
        s.append(f'<circle cx="{X(T[i]):.1f}" cy="{Y(D[i]):.1f}" r="6" fill="none" stroke="{col}" stroke-width="2.4"/>')
    panel = []
    for lv in (1, 2, 3):
        if lv in first:
            i = first[lv]
            ttc = TTC[i]
            tl = f' · TTC {ttc:.1f}s' if (ttc and ttc < 900) else ''
            panel.append((LVC[lv], f'LV{lv} {LVNAME[lv]}', f'{D[i]:.1f} m{tl} · {T[i]:.0f}s'))
    if clear_i is not None:
        panel.append(('#0f766e', '정지 → 자동 해제', f'{D[clear_i]:.1f} m · {T[clear_i]:.0f}s'))
    pnx, pny, pnw = 600, 96, 300
    s.append(f'<rect x="{pnx}" y="{pny}" width="{pnw}" height="{30+len(panel)*34}" rx="9" fill="#ffffff" stroke="#e2e8f0"/>')
    s.append(f'<text x="{pnx+16}" y="{pny+22}" font-size="12.5" font-weight="700" fill="#475569">이벤트 진행</text>')
    ry = pny + 44
    for col, name, det in panel:
        s.append(f'<circle cx="{pnx+22}" cy="{ry-4}" r="5.5" fill="{col}"/>')
        s.append(f'<text x="{pnx+38}" y="{ry}" font-size="13" font-weight="700" fill="#1f2937">{name}</text>')
        s.append(f'<text x="{pnx+pnw-14}" y="{ry}" font-size="12" fill="#64748b" text-anchor="end">{det}</text>')
        ry += 34
    lx, ly, ox = px0, H - 20, PL + 42
    s.append(f'<text x="{lx}" y="{ly}" font-size="12" fill="#64748b">등급:</text>')
    for lv in (0, 1, 2, 3):
        s.append(f'<rect x="{ox}" y="{ly-11}" width="13" height="13" rx="3" fill="{LVTINT[lv]}" stroke="{LVC[lv]}" stroke-width="1.4"/>')
        s.append(f'<text x="{ox+18}" y="{ly}" font-size="12" fill="#334155">{lv} {LVNAME[lv]}</text>')
        ox += 96
    s.append('</svg>')
    open(os.path.join(OUT, 'approach-timeline.svg'), 'w', encoding='utf-8').write('\n'.join(s))
    print('approach-timeline.svg  (%d행, %.1fs)' % (len(rows), T[-1]))


# ----------------------------------------------------------------------------
def make_summary():
    files = sorted(glob.glob(os.path.join(DATA, "**", "risk_tx_*.csv"), recursive=True))
    BINS = [(0, 2), (2, 4), (4, 6), (6, 10), (10, 15), (15, 30), (30, 1e9)]
    BLAB = ['0–2', '2–4', '4–6', '6–10', '10–15', '15–30', '30+']
    counts = {i: [0, 0, 0, 0] for i in range(len(BINS))}
    sessions, days, total = set(), set(), 0
    for f in files:
        try:
            r = load(f)
        except Exception:
            continue
        if len(r) < 8:
            continue
        sessions.add(os.path.basename(f))
        p = os.path.basename(f).split('_')
        if len(p) >= 3:
            days.add(p[2])
        for x in r:
            if not (str(x.get('cane_gps_valid')).startswith('1') and str(x.get('veh_gps_valid')).startswith('1')):
                continue
            d = fnum(x.get('distance_m'))
            if d is None:
                continue
            lv = max(0, min(3, int(float(x.get('effective_level') or 0))))
            total += 1
            for bi, (lo, hi) in enumerate(BINS):
                if lo <= d < hi:
                    counts[bi][lv] += 1
                    break

    W, H = 940, 452
    PL, PR, PT, bar_h, gap = 118, 150, 96, 34, 14
    x0, x1 = PL, W - PR
    barw = x1 - x0
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="{FONT}">',
         f'<rect width="{W}" height="{H}" rx="14" fill="#ffffff"/>',
         f'<rect width="{W}" height="{H}" rx="14" fill="none" stroke="#e2e8f0"/>',
         f'<text x="{PL}" y="34" font-size="21" font-weight="700" fill="#0f172a">거리별 위험등급 분포 · 실도로 {total:,}건</text>',
         f'<text x="{PL}" y="58" font-size="13" fill="#64748b">{len(sessions)}개 세션 · 7일(2026-08-12~25) · GPS 유효 판정만 · 가까울수록 높은 등급이 실제로 발생 (가공 없음)</text>']
    y = PT
    for bi in range(len(BINS)):
        c = counts[bi]
        tot = sum(c)
        if not tot:
            continue
        esc = (c[1] + c[2] + c[3]) / tot * 100
        s.append(f'<text x="{PL-14}" y="{y+bar_h/2+5:.0f}" font-size="13" font-weight="600" fill="#334155" text-anchor="end">{BLAB[bi]} m</text>')
        cx = x0
        for lv in (0, 1, 2, 3):
            w = c[lv] / tot * barw
            if w > 0:
                s.append(f'<rect x="{cx:.2f}" y="{y}" width="{w:.2f}" height="{bar_h}" fill="{LVC[lv]}"/>')
                if w > 30:
                    s.append(f'<text x="{cx+w/2:.1f}" y="{y+bar_h/2+4:.0f}" font-size="11" font-weight="600" fill="#ffffff" text-anchor="middle">{c[lv]/tot*100:.0f}%</text>')
            cx += w
        s.append(f'<text x="{x1+12}" y="{y+bar_h/2-2:.0f}" font-size="12" fill="#475569">{tot:,}건</text>')
        s.append(f'<text x="{x1+12}" y="{y+bar_h/2+13:.0f}" font-size="11.5" font-weight="700" fill="#b91c1c">경고↑ {esc:.0f}%</text>')
        y += bar_h + gap
    ly, ox = y + 18, PL + 42
    s.append(f'<text x="{PL}" y="{ly}" font-size="12" fill="#64748b">등급:</text>')
    for lv in (0, 1, 2, 3):
        s.append(f'<rect x="{ox}" y="{ly-11}" width="13" height="13" rx="3" fill="{LVC[lv]}"/>')
        s.append(f'<text x="{ox+18}" y="{ly}" font-size="12" fill="#334155">{lv} {LVNAME[lv]}</text>')
        ox += 100
    s.append('</svg>')
    open(os.path.join(OUT, 'distance-risk.svg'), 'w', encoding='utf-8').write('\n'.join(s))
    print('distance-risk.svg  (%d세션, %d건)' % (len(sessions), total))


if __name__ == '__main__':
    make_hero()
    make_summary()
    print('완료 ->', OUT)
