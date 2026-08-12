# -*- coding: utf-8 -*-
"""예측 결과 시각화 뷰어 v2 — 정확도·기법 기여도 강조판"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).parent
SCENARIO = "scenario_7552"
SRC = HERE / f"{SCENARIO}_result.csv"
PED_SRC = HERE / f"{SCENARIO}_pedestrian.csv"


def gate(d):
    if d <= 2.5: return 1.0
    if d >= 7.5: return 0.2
    return 1.0 + (d - 2.5) / 5.0 * (0.2 - 1.0)


def classify(s):
    return 3 if s >= 70 else 2 if s >= 45 else 1 if s >= 20 else 0


df = pd.read_csv(SRC)
df["actual_level"] = [classify(s * gate(d))
                      for s, d in zip(df["risk_score"], df["dcpa_m"])]
# 물리 단독의 3초 예측 (phys_score_3s에 방향 억제 적용)
df["phys_level"] = [classify(s * gate(d))
                    for s, d in zip(df["phys_score_3s"], df["dcpa_m"])]

danger_ids, safe_ids = [], []
for vid, g in df.groupby("vehicle_id"):
    (danger_ids if g["onnx_risk_level"].max() >= 2 else safe_ids).append(vid)
picked = danger_ids[:12] + safe_ids[:8]
sub = df[df["vehicle_id"].isin(picked)]

vehicles = []
tot_rows = tot_match = tot_phys_same = 0
fut_danger_rows = fut_danger_hit = 0
leads = []
for vid, g in sub.groupby("vehicle_id"):
    g = g.sort_values("timestep_time").reset_index(drop=True)
    actual = g["actual_level"].astype(int).to_numpy()
    pred = g["onnx_risk_level"].astype(int).to_numpy()
    phys = g["phys_level"].astype(int).to_numpy()
    t = g["timestep_time"].round(0).astype(int).to_numpy()

    # 미래 정답: 각 시점의 "향후 3초 실제 최대 위험"
    fut = np.full(len(actual), -1)
    for i in range(len(actual) - 1):
        fut[i] = actual[i + 1:i + 4].max()
    valid = fut >= 0

    match = int((pred[valid] == fut[valid]).sum())
    n_valid = int(valid.sum())
    phys_same = int((pred == phys).sum())
    tot_rows += n_valid; tot_match += match; tot_phys_same += phys_same

    fd = valid & (fut >= 2)
    fut_danger_rows += int(fd.sum())
    fut_danger_hit += int((pred[fd] >= 2).sum())

    # 선행 시간: AI 첫 L2+ 경고 vs 실제 첫 L2+ 발생
    lead = None
    a_idx = np.where(actual >= 2)[0]
    p_idx = np.where(pred >= 2)[0]
    if len(a_idx) and len(p_idx) and p_idx[0] <= a_idx[0]:
        lead = int(t[a_idx[0]] - t[p_idx[0]])
        leads.append(lead)

    vid_label = str(int(vid)) if float(vid) == int(float(vid)) else str(vid)
    vehicles.append({
        "id": vid_label,
        "danger": int(pred.max()),
        "t": t.tolist(),
        "x": g["veh_x"].round(1).tolist(),
        "y": g["veh_y"].round(1).tolist(),
        "pred": pred.tolist(),
        "actual": actual.tolist(),
        "phys": phys.tolist(),
        "dist": g["distance_m"].round(1).tolist(),
        "acc": round(match / n_valid * 100, 1) if n_valid else None,
        "aiCorr": round((1 - phys_same / len(pred)) * 100, 1),
        "lead": lead,
        "tDanger": int(t[a_idx[0]]) if len(a_idx) else None,
        "tAlarm": int(t[p_idx[0]]) if len(p_idx) else None,
    })
vehicles.sort(key=lambda v: -v["danger"])

# 보행자 전체 궤적 (v2: 여러 명)
pedf = pd.read_csv(PED_SRC, sep=";")
peds = []
for pid, g in pedf.groupby("person_id"):
    g = g.sort_values("timestep_time")
    peds.append({"id": str(pid),
                 "t": g["timestep_time"].round(0).astype(int).tolist(),
                 "x": g["person_x"].round(1).tolist(),
                 "y": g["person_y"].round(1).tolist()})
stats = {
    "acc": round(tot_match / tot_rows * 100, 1),
    "detect": round(fut_danger_hit / fut_danger_rows * 100, 1) if fut_danger_rows else None,
    "leadMean": round(float(np.mean(leads)), 1) if leads else None,
    "physSame": round(tot_phys_same / max(1, len(sub)) * 100, 1),
    "aiCorr": round((1 - tot_phys_same / max(1, len(sub))) * 100, 1),
}
data = {"scenario": f"{SCENARIO} (현실성 v2 — 보행자 {len(peds)}명)",
        "peds": peds,
        "vehicles": vehicles, "stats": stats}
data_js = json.dumps(data, ensure_ascii=False)

html = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<title>AI-V2X 3초 선행 예측 뷰어</title>
<style>
body{font-family:'Malgun Gothic',sans-serif;margin:0;background:#f4f6fb;color:#1a2c56}
header{background:#1a2c56;color:#fff;padding:12px 20px}
header h1{font-size:17px;margin:0}
header p{font-size:12px;margin:4px 0 0;color:#cdd6ea}
.stats{display:flex;gap:10px;padding:12px 14px 0;flex-wrap:wrap}
.stat{background:#fff;border-radius:10px;box-shadow:0 1px 4px rgba(0,0,0,.12);
  padding:10px 18px;text-align:center;flex:1;min-width:150px}
.stat .v{font-size:26px;font-weight:bold;color:#2b5bd7}
.stat .l{font-size:11.5px;color:#666;margin-top:2px;line-height:1.4}
.stat.green .v{color:#1e7d46}.stat.orange .v{color:#c05600}
.wrap{display:flex;gap:14px;padding:14px;flex-wrap:wrap}
.panel{background:#fff;border-radius:10px;box-shadow:0 1px 4px rgba(0,0,0,.12);padding:12px}
canvas{display:block;background:#fafbff;border-radius:6px}
.controls{margin-top:8px;display:flex;align-items:center;gap:10px}
button{background:#2b5bd7;color:#fff;border:0;border-radius:6px;padding:6px 16px;font-size:14px;cursor:pointer}
input[type=range]{flex:1}
.legend{font-size:12px;margin-top:6px;color:#444}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin:0 3px 0 10px}
#vlist{max-height:110px;overflow-y:auto;margin-top:8px;font-size:12.5px}
.vitem{padding:3px 8px;border-radius:5px;cursor:pointer;display:inline-block;margin:2px}
.vitem:hover{background:#eef2fb}.vitem.sel{background:#2b5bd7;color:#fff}
h3{font-size:13.5px;margin:2px 0 8px}
.note{font-size:11.5px;color:#666;margin-top:6px;line-height:1.55}
#vbadges{display:flex;gap:8px;margin:4px 0 8px;flex-wrap:wrap}
.badge{font-size:12px;padding:4px 10px;border-radius:12px;background:#eef2fb;color:#1a2c56;font-weight:bold}
.badge.lead{background:#e8f4ec;color:#1e7d46}
.badge.corr{background:#fdf3ec;color:#c05600}
</style></head><body>
<header><h1>AI-V2X 3초 선행 예측 뷰어 — __SCENARIO__ (TTC 수정 v4 모델 · 캠퍼스 차량 포함)</h1>
<p>세 개의 선: <b>파랑=AI 최종 예측</b> · <b>주황 점선=물리 계산만</b> · <b>회색=실제 위험</b> — 파랑이 회색보다 먼저 오르면 선행 예측 성공, 파랑이 주황과 다르면 AI가 물리를 보정한 것</p></header>
<div class="stats" id="statbar"></div>
<div class="wrap">
<div class="panel"><h3>🗺️ 시뮬레이션 재생</h3>
<canvas id="map" width="560" height="440"></canvas>
<div class="controls"><button id="play">▶ 재생</button>
<input type="range" id="time" min="0" max="100" value="0"><span id="tlabel" style="font-size:13px;width:52px">t=0s</span></div>
<div class="legend">차량 색 = AI 예측: <span class="dot" style="background:#1e7d46"></span>안전
<span class="dot" style="background:#c9a800"></span>주의 <span class="dot" style="background:#e07000"></span>경고
<span class="dot" style="background:#d21f1f"></span>위험 · <span class="dot" style="background:#2b5bd7"></span>보행자</div>
<div id="vlist"></div></div>
<div class="panel"><h3 id="ctitle">📈 차량을 선택하세요</h3>
<div id="vbadges"></div>
<canvas id="chart" width="560" height="290"></canvas>
<canvas id="dchart" width="560" height="130" style="margin-top:8px"></canvas>
<div class="note"><b>정확도 읽는 법</b>: 이 차량의 "일치율"은 매초 AI 예측을 <b>3초 뒤 실제 결과</b>와 대조한 값.
<b>기법 비율</b>: 혼합 비율은 고정이 아니라 상황별로 달라짐 — "AI 보정 N%"는 AI 최종 판정이
물리 계산과 다르게 (대부분 더 일찍·더 정확하게) 판단한 시점의 비율.<br>
초록 음영 = AI가 실제 위험 발생보다 먼저 경고한 구간(선행 시간).</div></div>
</div>
<script>
const DATA = __DATA__;
const COLORS = ["#1e7d46","#c9a800","#e07000","#d21f1f"];
let xs=[], ys=[], ts=[];
DATA.vehicles.forEach(v=>{xs.push(...v.x); ys.push(...v.y); ts.push(...v.t)});
DATA.peds.forEach(p=>{xs.push(...p.x); ys.push(...p.y); ts.push(...p.t)});
const xmin=Math.min(...xs), xmax=Math.max(...xs), ymin=Math.min(...ys), ymax=Math.max(...ys);
const tmin=Math.min(...ts), tmax=Math.max(...ts);
const map=document.getElementById('map'), mctx=map.getContext('2d');
const sx=x=>(x-xmin)/(xmax-xmin)*520+20, sy=y=>420-(y-ymin)/(ymax-ymin)*400;
let curT=tmin, playing=false, selected=null;
const slider=document.getElementById('time');
slider.min=tmin; slider.max=tmax; slider.value=tmin;

// 상단 통계 타일
const S=DATA.stats;
document.getElementById('statbar').innerHTML=
 `<div class="stat"><div class="v">${S.acc}%</div><div class="l">예측 ↔ 3초 뒤 실제<br>일치율 (이 시나리오)</div></div>`+
 (S.detect!==null?`<div class="stat green"><div class="v">${S.detect}%</div><div class="l">미래 위험(L2+)<br>사전 검출률</div></div>`:"")+
 (S.leadMean!==null?`<div class="stat green"><div class="v">${S.leadMean}초</div><div class="l">위험 발생보다<br>먼저 경고 (평균)</div></div>`:"")+
 `<div class="stat orange"><div class="v">${S.physSame}% : ${S.aiCorr}%</div><div class="l">물리 계산과 동일 : AI가 보정<br>(기법 기여 비율)</div></div>`;

function at(v,t){const i=v.t.indexOf(t); return i<0?null:i;}
function drawMap(){
  mctx.clearRect(0,0,560,440);
  DATA.vehicles.forEach(v=>{mctx.strokeStyle="#dde4f2"; mctx.beginPath();
    v.x.forEach((x,i)=>{i?mctx.lineTo(sx(x),sy(v.y[i])):mctx.moveTo(sx(x),sy(v.y[i]))});
    mctx.stroke();});
  DATA.peds.forEach(p=>{
    const pi=p.t.indexOf(curT);
    if(pi>=0){mctx.fillStyle="#2b5bd7"; mctx.beginPath();
      mctx.arc(sx(p.x[pi]),sy(p.y[pi]),6,0,7); mctx.fill();}});
  DATA.vehicles.forEach(v=>{
    const i=at(v,curT); if(i===null)return;
    mctx.fillStyle=COLORS[v.pred[i]];
    mctx.beginPath(); mctx.arc(sx(v.x[i]),sy(v.y[i]),v===selected?9:6,0,7); mctx.fill();
    if(v===selected){mctx.strokeStyle="#1a2c56"; mctx.lineWidth=2.5; mctx.stroke(); mctx.lineWidth=1;
      mctx.fillStyle="#1a2c56"; mctx.font="12px sans-serif";
      mctx.fillText("차량 "+v.id, sx(v.x[i])+12, sy(v.y[i])-8);}});
  document.getElementById('tlabel').textContent="t="+curT+"s";
}
function drawChart(){
  const c=document.getElementById('chart'), ctx=c.getContext('2d');
  const d=document.getElementById('dchart'), dtx=d.getContext('2d');
  ctx.clearRect(0,0,560,290); dtx.clearRect(0,0,560,130);
  if(!selected)return;
  const v=selected;
  document.getElementById('ctitle').textContent="📈 차량 "+v.id;
  let badges="";
  if(v.acc!==null)badges+=`<span class="badge">예측↔실제 일치 ${v.acc}%</span>`;
  if(v.lead!==null)badges+=`<span class="badge lead">실제 위험보다 ${v.lead}초 먼저 경고</span>`;
  badges+=`<span class="badge corr">AI 보정 ${v.aiCorr}% · 물리 동일 ${(100-v.aiCorr).toFixed(1)}%</span>`;
  document.getElementById('vbadges').innerHTML=badges;

  const t0=v.t[0], t1=v.t[v.t.length-1];
  const px=t=>(t-t0)/Math.max(1,(t1-t0))*505+42, py=l=>240-l*62;
  // 선행 구간 음영 (AI 첫 경고 ~ 실제 위험 발생)
  if(v.tAlarm!==null&&v.tDanger!==null&&v.tAlarm<v.tDanger){
    ctx.fillStyle="rgba(30,125,70,.14)";
    ctx.fillRect(px(v.tAlarm),20,px(v.tDanger)-px(v.tAlarm),240);
    ctx.fillStyle="#1e7d46"; ctx.font="bold 12px sans-serif";
    ctx.fillText((v.tDanger-v.tAlarm)+"초 선행",px(v.tAlarm)+4,34);}
  ctx.strokeStyle="#eee"; ctx.fillStyle="#888"; ctx.font="11px sans-serif";
  for(let l=0;l<4;l++){ctx.beginPath();ctx.moveTo(42,py(l));ctx.lineTo(548,py(l));ctx.stroke();
    ctx.fillText(["안전0","주의1","경고2","위험3"][l],2,py(l)+4);}
  function line(arr,color,w,dash){ctx.strokeStyle=color;ctx.lineWidth=w;
    if(dash)ctx.setLineDash(dash);ctx.beginPath();
    arr.forEach((l,i)=>{const X=px(v.t[i]),Y=py(l);
      i?(ctx.lineTo(X,py(arr[i-1])),ctx.lineTo(X,Y)):ctx.moveTo(X,Y);});
    ctx.stroke();ctx.setLineDash([]);ctx.lineWidth=1;}
  line(v.actual,"#999",2);
  line(v.phys,"#e07000",1.8,[5,3]);
  line(v.pred,"#2b5bd7",2.6);
  const ti=px(Math.min(Math.max(curT,t0),t1));
  ctx.strokeStyle="#d21f1f"; ctx.setLineDash([4,3]);
  ctx.beginPath();ctx.moveTo(ti,18);ctx.lineTo(ti,262);ctx.stroke();ctx.setLineDash([]);
  const dmax=Math.max(...v.dist);
  const dy=val=>112-val/dmax*95;
  dtx.strokeStyle="#eee"; [0,dmax/2,dmax].forEach(val=>{dtx.beginPath();
    dtx.moveTo(42,dy(val));dtx.lineTo(548,dy(val));dtx.stroke();
    dtx.fillStyle="#888";dtx.font="10px sans-serif";dtx.fillText(Math.round(val)+"m",4,dy(val)+3);});
  dtx.strokeStyle="#e07000";dtx.lineWidth=2;dtx.beginPath();
  v.dist.forEach((val,i)=>{const X=px(v.t[i]);i?dtx.lineTo(X,dy(val)):dtx.moveTo(X,dy(val));});
  dtx.stroke();dtx.lineWidth=1;
  dtx.strokeStyle="#d21f1f";dtx.setLineDash([4,3]);dtx.beginPath();
  dtx.moveTo(ti,5);dtx.lineTo(ti,125);dtx.stroke();dtx.setLineDash([]);
  dtx.fillStyle="#666";dtx.font="10px sans-serif";dtx.fillText("보행자와의 거리",460,14);
}
const vlist=document.getElementById('vlist');
DATA.vehicles.forEach(v=>{
  const el=document.createElement('span');
  el.className='vitem'; el.style.borderLeft="4px solid "+COLORS[v.danger];
  el.textContent="차량 "+v.id+(v.danger>=2?" ⚠":"");
  el.onclick=()=>{selected=v;
    document.querySelectorAll('.vitem').forEach(e=>e.classList.remove('sel'));
    el.classList.add('sel'); curT=v.t[0]; slider.value=curT; drawMap(); drawChart();};
  vlist.appendChild(el);});
slider.oninput=()=>{curT=parseInt(slider.value); drawMap(); drawChart();};
document.getElementById('play').onclick=function(){
  playing=!playing; this.textContent=playing?"⏸ 정지":"▶ 재생";
  if(playing)tick();};
function tick(){if(!playing)return;
  curT=curT>=tmax?tmin:curT+1; slider.value=curT; drawMap(); drawChart();
  setTimeout(tick,180);}
if(DATA.vehicles.length){document.querySelector('.vitem').click();}
drawMap();
</script></body></html>"""

html = html.replace("__SCENARIO__", data["scenario"]).replace("__DATA__", data_js)
out = HERE / "예측결과_뷰어.html"
out.write_text(html, encoding="utf-8")
print("saved:", out, f"({out.stat().st_size/1024:.0f}KB)")
print("시나리오 통계:", stats)
