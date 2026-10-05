#!/usr/bin/env python3
"""다중 차량 추적 뷰어 (HTML 한 장) - SUMO 동시 접근 장면을 기존 엔진 vs 다중 엔진으로 재생.

AI_Model/transformer/make_prediction_viewer.py(예측 뷰어)와 같은 형식: 데이터를 HTML 안에
넣은 파일 하나라 더블클릭으로 열린다(채팅 미리보기에선 JS가 안 돈다).

보여 주는 것
    지도      보행자(지팡이)와 차 여러 대. 차 색 = 그 차의 다중 엔진 위험도(차선별)
    지팡이    지팡이로 나간 경보: 기존 엔진(차량 칸 하나) vs 다중 엔진, 빨간 음영 = SUMO 라벨 L2+
    차별 줄   차마다 다중 엔진 위험도(실선) vs 그 차의 SUMO 라벨(회색 점선)

장면 고르기: 차 2~4대, 라벨 L2+ 차가 2대 이상, 기존 엔진이 라벨 위험 구간을 하나 이상 놓쳤고
다중 엔진은 잡은 장면. 클라우드 불필요 - PC의 v5data로 전부 재생한다.

사용
    python make_multi_vehicle_viewer.py <v5data 폴더> [--scenes 8] [--out 다중차량추적_뷰어.html]
"""
import argparse
import bisect
import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import eval_multi_vehicle as ev  # noqa: E402
import gps_noise  # noqa: E402
import sim_multi_vehicle_stream as sm  # noqa: E402
from sim_to_rsu_stream import BASE_EPOCH  # noqa: E402

STEP_S = 0.2       # 화면 시간 간격(5 Hz)
VIEW_HALF_M = 30.0  # 지도: 지금 보행자 위치를 가운데 두고 사방으로 이만큼(보행자를 따라간다)


def sampler(timeline):
    """[(t, 등급)] 바뀔 때만 기록 → t에서의 등급(그 전이면 0)."""
    times = [t for t, _ in timeline]
    levels = [lv for _, lv in timeline]

    def at(t):
        i = bisect.bisect_right(times, t) - 1
        return levels[i] if i >= 0 else 0
    return at


def build_scene(scenario_dir, item):
    vids = [v for v, _, _ in item["vehicles"]]
    t_min = [t for _, t, _ in item["vehicles"]]
    ped, tracks = sm.load_episode(scenario_dir, item["person_id"], vids,
                                  t_focus=(min(t_min), max(t_min)))
    if not 2 <= len(tracks) <= 4:
        return None
    seed = gps_noise.scenario_seed(f"{scenario_dir.name}_p{item['person_id']}")
    per_node = sm.node_packets(ped, tracks, seed, False)
    full = sm.merge_packets(per_node)
    old = ev.replay_single(full)
    new, lanes = ev.replay_multi(full)
    labels = sm.sumo_labels(scenario_dir, item["person_id"], list(tracks))
    ids = sm.node_id_map(tracks)

    # 장면 조건: 라벨 L2+ 차 2대 이상, 기존이 놓친 라벨 구간 있고 다중은 그걸 잡음
    t_lo, t_hi = full[0]["pc_time"] - BASE_EPOCH, full[-1]["pc_time"] - BASE_EPOCH
    danger_cars, old_missed, new_caught = 0, 0, 0
    runs = {}
    for vid in tracks:
        lab = labels.get(vid)
        if lab is None:
            continue
        lab = lab[(lab["t"] >= t_lo) & (lab["t"] <= t_hi)]
        runs[vid] = ev.label_runs(lab, 2)
        if runs[vid]:
            danger_cars += 1
        for s, e in runs[vid]:
            o = ev.alarm_during(old, BASE_EPOCH + s - ev.LEAD_S, BASE_EPOCH + e)[0]
            n = ev.alarm_during(new, BASE_EPOCH + s - ev.LEAD_S, BASE_EPOCH + e)[0]
            old_missed += int(not o)
            new_caught += int(n and not o)
    if danger_cars < 2 or new_caught == 0:
        return None

    # 차 이름: 지팡이에 가장 가까이 온 순서대로 A, B, C ...
    closest = {}
    for vid, track in tracks.items():
        dmin = min(((s[1] - p[1]) ** 2 + (s[2] - p[2]) ** 2) ** 0.5
                   for s in track for p in [_nearest(ped, s[0])] if p is not None)
        tcpa = min(track, key=lambda s: _dist(s, _nearest(ped, s[0])))[0]
        closest[vid] = (tcpa, dmin)
    order = sorted(tracks, key=lambda v: closest[v][0])
    names = {vid: chr(ord("A") + i) for i, vid in enumerate(order)}

    grid = [round(t_lo + i * STEP_S, 2) for i in range(int((t_hi - t_lo) / STEP_S) + 1)]
    ped_at = {round(s[0], 2): s for s in ped}
    old_at, new_at = sampler(old), sampler(new)
    scene = {
        "title": f"{scenario_dir.name} · 보행자 {item['person_id']} · 차 {len(tracks)}대",
        "t": [round(t - t_lo, 1) for t in grid],
        "ped": [[round(ped_at[t][1], 1), round(ped_at[t][2], 1)] if t in ped_at else None for t in grid],
        "old": [old_at(BASE_EPOCH + t) for t in grid],
        "new": [new_at(BASE_EPOCH + t) for t in grid],
        "vehicles": [],
    }
    for vid in order:
        track = {round(s[0], 2): s for s in tracks[vid]}
        lane_at = sampler(lanes.get(str(ids[vid]), []))
        lab = labels.get(vid)
        lab_at = ({} if lab is None else
                  {int(t): int(lv) for t, lv in zip(lab["t"], lab["risk_level"])})
        pos, lane, label, dist = [], [], [], []
        for t in grid:
            s = track.get(t)
            p = ped_at.get(t)
            pos.append([round(s[1], 1), round(s[2], 1)] if s else None)
            lane.append(lane_at(BASE_EPOCH + t) if s else None)
            label.append(lab_at.get(int(t)) if s else None)
            dist.append(round(_dist(s, p), 1) if s and p else None)
        scene["vehicles"].append({
            "name": names[vid], "sumo_id": vid, "pos": pos, "lane": lane, "label": label,
            "dist": dist, "dmin": round(closest[vid][1], 1),
            "runs": [[round(s - t_lo, 1), round(e - t_lo, 1)] for s, e in runs.get(vid, [])],
        })
    scene["missed_old"] = old_missed
    scene["caught_new"] = new_caught
    scene["view_half"] = VIEW_HALF_M
    return scene


def _nearest(ped, t):
    i = bisect.bisect_left([p[0] for p in ped], t - 1e-6)
    return ped[i] if i < len(ped) and abs(ped[i][0] - t) < 0.11 else None


def _dist(a, b):
    if a is None or b is None:
        return float("inf")
    return ((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def global_stats(eval_dir):
    """eval_multi_vehicle.py 결과(300묶음)에서 상단 타일 숫자."""
    runs = list(csv.DictReader((eval_dir / "per_label_run.csv").open(encoding="utf-8-sig")))
    rows = list(csv.DictReader((eval_dir / "per_vehicle.csv").open(encoding="utf-8-sig")))

    def rate(sel, key):
        return round(sum(int(r[key]) for r in sel) / max(1, len(sel)) * 100, 1)
    l3 = [r for r in runs if r["label_level"] == "3"]
    l2 = [r for r in runs if r["label_level"] == "2"]
    ref_eps = sum(int(r["ref_alarm_episodes"]) for r in rows)
    return {
        "episodes": len({(r["scenario"], r["person_id"]) for r in rows}),
        "cars": len(rows),
        "l3": len(l3), "l3_old": rate(l3, "old_detected"), "l3_new": rate(l3, "new_detected"),
        "l2": len(l2), "l2_old": rate(l2, "old_detected"), "l2_new": rate(l2, "new_detected"),
        "kept_old": round(sum(int(r["old_kept"]) for r in rows) / max(1, ref_eps) * 100, 1),
        "kept_new": round(sum(int(r["new_kept"]) for r in rows) / max(1, ref_eps) * 100, 1),
    }


HTML = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<title>다중 차량 추적 뷰어</title>
<style>
body{font-family:'Malgun Gothic',sans-serif;margin:0;background:#f4f6fb;color:#1a2c56}
header{background:#1a2c56;color:#fff;padding:12px 20px}
header h1{font-size:17px;margin:0}
header p{font-size:12px;margin:4px 0 0;color:#cdd6ea;line-height:1.5}
.stats{display:flex;gap:10px;padding:12px 14px 0;flex-wrap:wrap}
.stat{background:#fff;border-radius:10px;box-shadow:0 1px 4px rgba(0,0,0,.12);
  padding:10px 16px;text-align:center;flex:1;min-width:170px}
.stat .v{font-size:22px;font-weight:bold}
.stat .v .o{color:#c05600}.stat .v .n{color:#1e7d46}
.stat .l{font-size:11.5px;color:#666;margin-top:2px;line-height:1.4}
.wrap{display:flex;gap:14px;padding:14px;flex-wrap:wrap}
.panel{background:#fff;border-radius:10px;box-shadow:0 1px 4px rgba(0,0,0,.12);padding:12px}
canvas{display:block;background:#fafbff;border-radius:6px}
.controls{margin-top:8px;display:flex;align-items:center;gap:10px}
button{background:#2b5bd7;color:#fff;border:0;border-radius:6px;padding:6px 16px;font-size:14px;cursor:pointer}
select{font-size:13px;padding:4px;border-radius:6px;border:1px solid #c8d0e4;max-width:100%}
input[type=range]{flex:1}
.legend{font-size:12px;margin-top:6px;color:#444;line-height:1.7}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 3px 0 8px}
h3{font-size:13.5px;margin:2px 0 8px}
.note{font-size:11.5px;color:#666;margin-top:6px;line-height:1.55;max-width:600px}
#now{font-size:12.5px;margin-top:8px;line-height:1.7}
#now .row{display:flex;gap:8px;align-items:center}
.lv{display:inline-block;min-width:44px;text-align:center;border-radius:10px;color:#fff;font-weight:bold;font-size:11.5px;padding:1px 6px}
.scene-badge{font-size:12px;padding:4px 10px;border-radius:12px;background:#fdf3ec;color:#c05600;font-weight:bold;display:inline-block;margin:4px 0}
</style></head><body>
<header><h1>다중 차량 추적 뷰어 — 차가 여러 대 올 때 차마다 따로 위험도를 계산해 경보</h1>
<p>같은 SUMO 장면(GPS 잡음 포함 패킷)을 두 엔진으로 재생: <b>기존 엔진</b>(차량 칸 하나 — 두 차가 서로 덮어씀) vs
<b>다중 엔진</b>(차마다 따로 추적, 지팡이엔 가장 높은 위험도). 빨간 음영 = SUMO 위험 라벨(채점표×DCPA 게이트) L2+ 구간.</p></header>
<div class="stats" id="statbar"></div>
<div class="wrap">
<div class="panel"><h3>🗺️ 장면 재생</h3>
<select id="scene"></select><br><span class="scene-badge" id="sbadge"></span>
<canvas id="map" width="520" height="440"></canvas>
<div class="controls"><button id="play">▶ 재생</button>
<input type="range" id="time" min="0" max="100" value="0" step="1"><span id="tlabel" style="font-size:13px;width:58px">0.0s</span></div>
<div class="legend">차 색 = 다중 엔진이 계산한 <b>그 차의</b> 위험도:
<span class="dot" style="background:#1e7d46"></span>안전 <span class="dot" style="background:#c9a800"></span>주의
<span class="dot" style="background:#e07000"></span>경고 <span class="dot" style="background:#d21f1f"></span>위험
· <span class="dot" style="background:#2b5bd7"></span>보행자(지팡이) · 회색 원 = 5/10/20 m</div>
<div id="now"></div></div>
<div class="panel"><h3>📈 지팡이로 나간 경보 — 기존 vs 다중</h3>
<canvas id="cane" width="600" height="150"></canvas>
<h3 style="margin-top:12px">🚗 차마다 따로 계산한 위험도 (다중 엔진) vs 그 차의 SUMO 라벨</h3>
<canvas id="cars" width="600" height="330"></canvas>
<div class="note"><b>읽는 법</b>: 빨간 음영(라벨상 위험)이 시작되기 전에 선이 경고(2) 이상으로 올라가면 제때 울린 것.
기존 엔진(주황 점선)은 다른 차 패킷이 칸을 덮어써 접근속도·TTC가 깨지면서 음영 구간에서도 조용한 경우가 많다.
다중 엔진(파랑)은 차별 위험도(아래 줄들) 중 가장 높은 값이라 어느 차든 위험하면 울린다.
라벨은 잡음 없는 좌표 1초 단위, 엔진은 GPS 잡음 패킷 5 Hz라 둘이 완전히 같지는 않다.</div></div>
</div>
<script>
const DATA = __DATA__;
const C = ["#1e7d46","#c9a800","#e07000","#d21f1f"], NAMES=["안전","주의","경고","위험"];
const G = DATA.stats;
document.getElementById('statbar').innerHTML =
 `<div class="stat"><div class="v"><span class="o">${G.l3_old}%</span> → <span class="n">${G.l3_new}%</span></div><div class="l">라벨 L3(위험) 구간 ${G.l3}개 검출<br>기존 → 다중</div></div>`+
 `<div class="stat"><div class="v"><span class="o">${G.l2_old}%</span> → <span class="n">${G.l2_new}%</span></div><div class="l">라벨 L2+(경고 이상) 구간 ${G.l2}개 검출<br>기존 → 다중</div></div>`+
 `<div class="stat"><div class="v"><span class="o">${G.kept_old}%</span> → <span class="n">${G.kept_new}%</span></div><div class="l">"그 차만 있었다면 울렸을" 경보 유지<br>기존 → 다중</div></div>`+
 `<div class="stat"><div class="v" style="color:#2b5bd7">${G.episodes}개 · ${G.cars}대</div><div class="l">SUMO 동시 접근 장면 · 차<br>(위 숫자의 평가 범위)</div></div>`;
const sel=document.getElementById('scene');
DATA.scenes.forEach((s,i)=>{const o=document.createElement('option');o.value=i;o.textContent=(i+1)+". "+s.title;sel.appendChild(o);});
let S=null, k=0, playing=false;
const slider=document.getElementById('time'), map=document.getElementById('map'), mx=map.getContext('2d');
function load(i){S=DATA.scenes[i]; k=0; center=null; slider.max=S.t.length-1; slider.value=0;
  document.getElementById('sbadge').textContent=`이 장면: 기존 엔진이 놓친 라벨 위험 구간 ${S.missed_old}개 → 다중 엔진은 그중 ${S.caught_new}개를 잡음`;
  draw();}
let center=null;
function view(){if(S.ped[k])center=S.ped[k]; const c=center||[0,0], half=S.view_half, s=Math.min(500,420)/(2*half);
  return {X:x=>260+(x-c[0])*s, Y:y=>220-(y-c[1])*s, s};}
function drawMap(){const V=view(); mx.clearRect(0,0,520,440);
  const p=S.ped[k];
  if(p){[5,10,20].forEach(r=>{mx.strokeStyle="#e3e7f0";mx.beginPath();mx.arc(V.X(p[0]),V.Y(p[1]),r*V.s,0,7);mx.stroke();});}
  mx.strokeStyle="#c7d3f0";mx.beginPath();let st=false;
  S.ped.forEach(q=>{if(!q)return;st?mx.lineTo(V.X(q[0]),V.Y(q[1])):mx.moveTo(V.X(q[0]),V.Y(q[1]));st=true;});mx.stroke();
  S.vehicles.forEach(v=>{mx.strokeStyle="#e4e4e4";mx.beginPath();let s2=false;
    v.pos.forEach(q=>{if(!q)return;s2?mx.lineTo(V.X(q[0]),V.Y(q[1])):mx.moveTo(V.X(q[0]),V.Y(q[1]));s2=true;});mx.stroke();});
  if(p){mx.fillStyle="#2b5bd7";mx.beginPath();mx.arc(V.X(p[0]),V.Y(p[1]),7,0,7);mx.fill();}
  S.vehicles.forEach(v=>{const q=v.pos[k]; if(!q)return; const lv=v.lane[k]||0;
    mx.fillStyle=C[lv];mx.beginPath();mx.arc(V.X(q[0]),V.Y(q[1]),8,0,7);mx.fill();
    mx.fillStyle="#1a2c56";mx.font="bold 13px sans-serif";mx.fillText(v.name,V.X(q[0])+10,V.Y(q[1])-8);});
  document.getElementById('tlabel').textContent=S.t[k].toFixed(1)+"s";
  const badge=lv=>`<span class="lv" style="background:${C[lv]}">${NAMES[lv]}</span>`;
  let h=`<div class="row"><b style="width:120px">지팡이 경보</b> 기존 ${badge(S.old[k])} · 다중 ${badge(S.new[k])}</div>`;
  S.vehicles.forEach(v=>{ if(v.pos[k]===null){h+=`<div class="row"><span style="width:120px">차 ${v.name}</span><span style="color:#aaa">화면 밖/없음</span></div>`;return;}
    h+=`<div class="row"><span style="width:120px">차 ${v.name} (${v.dist[k]===null?"-":v.dist[k]+" m"})</span> 다중 엔진 ${badge(v.lane[k]||0)} · 라벨 ${v.label[k]===null||v.label[k]===undefined?"-":badge(v.label[k])}</div>`;});
  document.getElementById('now').innerHTML=h;}
function axis(ctx,x0,w,top,hgt,title){ctx.fillStyle="#888";ctx.font="10px sans-serif";
  for(let l=0;l<4;l++){const y=top+hgt-l*hgt/3;ctx.strokeStyle="#f0f0f0";ctx.beginPath();ctx.moveTo(x0,y);ctx.lineTo(x0+w,y);ctx.stroke();}
  ctx.fillText("0",x0-10,top+hgt+3);ctx.fillText("3",x0-10,top+4);
  ctx.fillStyle="#1a2c56";ctx.font="bold 11px sans-serif";ctx.fillText(title,2,top+hgt/2+4);}
function step(ctx,arr,px,py,color,w,dash){ctx.strokeStyle=color;ctx.lineWidth=w;ctx.setLineDash(dash||[]);ctx.beginPath();let pv=null;
  arr.forEach((l,i)=>{if(l===null||l===undefined){pv=null;return;}const X=px(i),Y=py(l);
    if(pv===null)ctx.moveTo(X,Y);else{ctx.lineTo(X,py(pv));ctx.lineTo(X,Y);}pv=l;});
  ctx.stroke();ctx.setLineDash([]);ctx.lineWidth=1;}
function shade(ctx,runs,px,top,hgt){ctx.fillStyle="rgba(210,31,31,.13)";
  runs.forEach(([s,e])=>{const i0=S.t.findIndex(t=>t>=s), i1=S.t.findIndex(t=>t>=e+1);
    const a=px(Math.max(0,i0)), b=px(i1<0?S.t.length-1:i1); ctx.fillRect(a,top,Math.max(2,b-a),hgt);});}
function drawCharts(){const n=S.t.length, x0=62, w=530, px=i=>x0+i/(n-1)*w;
  const c=document.getElementById('cane'), cx=c.getContext('2d'); cx.clearRect(0,0,600,150);
  const allRuns=[].concat(...S.vehicles.map(v=>v.runs)); shade(cx,allRuns,px,10,110);
  axis(cx,x0,w,10,110,"지팡이"); const py=l=>120-l*110/3;
  step(cx,S.old,px,py,"#e07000",2,[6,4]); step(cx,S.new,px,py,"#2b5bd7",2.6);
  cx.fillStyle="#e07000";cx.font="11px sans-serif";cx.fillText("— — 기존 엔진",x0+5,145);
  cx.fillStyle="#2b5bd7";cx.fillText("—— 다중 엔진",x0+110,145);
  cx.fillStyle="#d21f1f";cx.fillText("■ 라벨 L2+ (어느 차든)",x0+215,145);
  cx.fillStyle="#888";cx.fillText(S.t[n-1].toFixed(0)+"s",x0+w-20,145);
  const d=document.getElementById('cars'), dx=d.getContext('2d'); dx.clearRect(0,0,600,330);
  const rowH=Math.min(100,(320)/S.vehicles.length);
  S.vehicles.forEach((v,r)=>{const top=6+r*rowH, hgt=rowH-28;
    shade(dx,v.runs,px,top,hgt); axis(dx,x0,w,top,hgt,"차 "+v.name);
    const pyy=l=>top+hgt-l*hgt/3;
    step(dx,v.label,px,pyy,"#9a9a9a",1.8,[4,3]); step(dx,v.lane,px,pyy,"#2b5bd7",2.4);
    dx.fillStyle="#666";dx.font="10px sans-serif";
    dx.fillText(`SUMO id ${v.sumo_id} · 최근접 ${v.dmin} m · 실선=다중 엔진(이 차) · 회색 점선=이 차 라벨`,x0,top+hgt+12);});
  [cx,dx].forEach((ctx,j)=>{const H=j?330:125;ctx.strokeStyle="#d21f1f";ctx.setLineDash([4,3]);ctx.beginPath();
    ctx.moveTo(px(k),4);ctx.lineTo(px(k),H);ctx.stroke();ctx.setLineDash([]);});}
function draw(){drawMap();drawCharts();}
sel.onchange=()=>load(parseInt(sel.value));
slider.oninput=()=>{k=parseInt(slider.value);draw();};
document.getElementById('play').onclick=function(){playing=!playing;this.textContent=playing?"⏸ 정지":"▶ 재생";if(playing)tick();};
function tick(){if(!playing)return;k=k>=S.t.length-1?0:k+1;slider.value=k;draw();setTimeout(tick,60);}
load(0);
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description="다중 차량 추적 뷰어 HTML 생성")
    ap.add_argument("data_dir")
    ap.add_argument("--scenes", type=int, default=8)
    ap.add_argument("--eval-dir", default=str(HERE / "multi_vehicle_eval"))
    ap.add_argument("--out", default=str(HERE / "다중차량추적_뷰어.html"))
    args = ap.parse_args()

    dirs = sorted(d for d in Path(args.data_dir).glob("scenario_*") if (d / "DONE").exists())
    # 후보를 넉넉히(3배) 모은 뒤 "기존이 놓친 걸 다중이 다 잡은" 장면부터 보여 준다.
    candidates = []
    for d in dirs:
        if len(candidates) >= args.scenes * 3:
            break
        for item in sm.find_multi_approaches(d):
            if len(candidates) >= args.scenes * 3:
                break
            scene = build_scene(d, item)
            if scene is not None:
                candidates.append(scene)
    candidates.sort(key=lambda sc: (sc["caught_new"] == sc["missed_old"], sc["caught_new"]),
                    reverse=True)
    scenes = candidates[:args.scenes]
    for i, scene in enumerate(scenes, 1):
        print(f"[장면 {i}] {scene['title']} - 기존이 놓친 라벨 구간 "
              f"{scene['missed_old']}개, 다중이 잡은 것 {scene['caught_new']}개")
    data = {"stats": global_stats(Path(args.eval_dir)), "scenes": scenes}
    out = Path(args.out)
    out.write_text(HTML.replace("__DATA__", json.dumps(data, ensure_ascii=False)), encoding="utf-8")
    print(f"saved: {out} ({out.stat().st_size / 1024:.0f}KB)")


if __name__ == "__main__":
    main()
