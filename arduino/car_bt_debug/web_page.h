// build_web_viewer.py 가 web_viewer.html 로 만든 파일. 직접 고치지 말 것.
#pragma once
static const char WEB_PAGE[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>V2X 듀얼 뷰어</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--line:#d8dbe0;--ink:#1a1c1f;--dim:#6b7280;
      --ok:#1a7f37;--warn:#b8860b;--bad:#c62828;--accent:#1a56db;--tile:#eef0f3}
*{box-sizing:border-box;-webkit-text-size-adjust:100%}
body{margin:0;padding:10px;background:var(--bg);color:var(--ink);
     font:15px/1.45 -apple-system,"Apple SD Gothic Neo",sans-serif}
h1{font-size:17px;margin:0 0 8px}
h2{font-size:15px;margin:0 0 6px;display:flex;align-items:center;gap:8px;
   flex-wrap:wrap}
.bar{background:var(--card);border:1px solid var(--line);border-radius:10px;
     padding:8px 10px;margin-bottom:10px;display:flex;flex-wrap:wrap;
     gap:8px;align-items:center}
.grid{display:grid;grid-template-columns:1fr;gap:10px}
@media(min-width:820px){.grid{grid-template-columns:1fr 1fr}}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;
        padding:10px;min-width:0}
table{width:100%;border-collapse:collapse;font-size:14px}
td{padding:3px 4px;border-bottom:1px solid #eef0f3;
   font-variant-numeric:tabular-nums}
td.k{color:var(--dim);width:44%;word-break:keep-all}
td.v{font-family:ui-monospace,Menlo,monospace;word-break:break-all}
tr.chg td.v{background:#fff6d5}
input,select,button{font:inherit;border-radius:8px;border:1px solid var(--line);
                    padding:7px 10px;background:#fff;color:var(--ink)}
input{flex:1;min-width:80px}
button{background:var(--accent);color:#fff;border-color:transparent;
       cursor:pointer;-webkit-appearance:none}
button.sub{background:#eef1f6;color:var(--ink);border-color:var(--line)}
button:active{opacity:.7}
.row{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}
.dot{width:9px;height:9px;border-radius:50%;background:#bbb;display:inline-block}
.dot.on{background:var(--ok)}.dot.old{background:var(--warn)}
.age{font-size:12px;color:var(--dim);font-weight:400}
.warn{font-size:13px;color:var(--bad);font-weight:700}
pre{background:#0f1115;color:#e6e8eb;border-radius:8px;padding:8px;
    font:12px/1.4 ui-monospace,Menlo,monospace;height:160px;overflow:auto;
    margin:0;white-space:pre-wrap;word-break:break-all}
.rec{color:var(--bad);font-weight:600}
a.dl{display:inline-block;background:var(--ok);color:#fff;text-decoration:none;
     padding:7px 10px;border-radius:8px;margin:4px 4px 0 0;font-size:14px}
label.chk{font-size:13px;font-weight:400;color:var(--dim);display:flex;
          align-items:center;gap:4px}
label.chk input{flex:none;min-width:0;padding:0;margin:0}
/* 요약: 큰 카드 + 상태 줄 */
.cards{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:6px;
       margin:2px 0 6px}
@media(max-width:520px){.cards{grid-template-columns:repeat(2,minmax(0,1fr))}}
.card{background:var(--tile);border-radius:8px;padding:5px 8px;min-width:0}
.card .ct{font-size:12px;color:var(--dim)}
.card .cv{font-size:21px;font-weight:700;line-height:1.25;white-space:nowrap;
          overflow:hidden;text-overflow:ellipsis;font-variant-numeric:tabular-nums}
.card .cs{font-size:12px;color:var(--dim);min-height:17px;white-space:nowrap;
          overflow:hidden;text-overflow:ellipsis}
.card.risk .ct,.card.risk .cv,.card.risk .cs{color:#fff}
.card.dim .cv{color:var(--dim)}
.line{display:flex;gap:6px;align-items:center;font-size:14px;padding:2px 0;
      font-variant-numeric:tabular-nums}
.line b{min-width:42px;flex:none}
.line.dim span{color:var(--dim)}
.ldot{width:8px;height:8px;border-radius:50%;flex:none;background:transparent}
.ldot.on{background:var(--ok)}.ldot.off{background:#bbb}
/* 전체 값: 묶음별로 접힘 */
.grp{margin-top:8px;border-top:1px solid var(--line);padding-top:4px}
details{border-top:1px solid #eef0f3}
details:first-child{border-top:0}
summary{cursor:pointer;font-size:14px;font-weight:600;padding:4px 0}
</style>
</head>
<body>

<h1>V2X 듀얼 뷰어 <span class="age" id="conn"></span></h1>

<div class="bar">
  <input id="recName" placeholder="기록 이름 (예: 직선접근1)">
  <button id="recBtn" onclick="toggleRecord()">기록 시작</button>
  <span id="recInfo" class="age">기록 안 함</span>
  <span style="flex:1"></span>
  <span id="modeBtns" style="display:flex;gap:6px;flex-wrap:wrap"></span>
  <select id="interval" onchange="restartTimer()">
    <option value="200">0.2초</option>
    <option value="500" selected>0.5초</option>
    <option value="1000">1초</option>
  </select>
</div>
<div id="dlBox"></div>

<div class="grid">
  <section>
    <h2><span class="dot" id="dotCar"></span>차량<span class="age" id="ageCar"></span>
        <span class="warn" id="warnCar"></span></h2>
    <div id="sumCar"></div>
    <div class="grp"><div class="age">전체 값 (누르면 펼쳐짐)</div>
      <div id="grpCar"></div></div>
    <div class="row">
      <input id="cmdCar" placeholder="명령 (예: alpha 0.25)"
             autocapitalize="off" autocorrect="off" spellcheck="false">
      <button onclick="send('car')">전송</button>
    </div>
    <div class="row">
      <button class="sub" onclick="quick('car','get')">get</button>
      <button class="sub" onclick="quick('car','help')">help</button>
      <button class="sub" onclick="quick('car','save')">save</button>
      <button class="sub" onclick="quick('car','reset')">reset</button>
      <button class="sub" onclick="quick('car','play 3')">음성</button>
    </div>
    <div class="row" id="quickCar"></div>
  </section>

  <section>
    <h2><span class="dot" id="dotCane"></span>지팡이<span class="age" id="ageCane"></span>
        <span class="warn" id="warnCane"></span></h2>
    <div id="sumCane"></div>
    <div class="grp"><div class="age">전체 값 (누르면 펼쳐짐)</div>
      <div id="grpCane"></div></div>
    <div class="row">
      <input id="cmdCane" placeholder="명령 (예: alpha 0.25)"
             autocapitalize="off" autocorrect="off" spellcheck="false">
      <button onclick="send('cane')">전송</button>
    </div>
    <div class="row">
      <button class="sub" onclick="quick('cane','get')">get</button>
      <button class="sub" onclick="quick('cane','help')">help</button>
      <button class="sub" onclick="quick('cane','save')">save</button>
      <button class="sub" onclick="quick('cane','reset')">reset</button>
      <button class="sub" onclick="quick('cane','test 3')">진동</button>
    </div>
    <div class="row" id="quickCane"></div>
  </section>
</div>

<section style="margin-top:10px">
  <h2>로그 <span class="age">명령 응답·알림</span>
    <label class="chk"><input type="checkbox" id="showVals">값 줄도 보기</label>
    <button class="sub" onclick="paused=!paused" id="pauseBtn"
            style="margin-left:auto;padding:4px 9px">일시정지</button>
    <button class="sub" onclick="logLines=[];draw()"
            style="padding:4px 9px">지우기</button>
  </h2>
  <pre id="log"></pre>
</section>

<!-- 요약 설정: dual_serial_viewer.py 의 SUMMARY 등을 build_web_viewer.py 가 복사해 넣는다.
     여기서 직접 고치지 말고 dual_serial_viewer.py 를 고친 뒤 build_web_viewer.py 실행. -->
<script id="viewer-spec" type="application/json">{"RISK_NAMES":{"0":"안전","1":"주의","2":"경고","3":"위험"},"RISK_COLORS":{"0":"#2e9e44","1":"#b8860b","2":"#e8710a","3":"#d0342c"},"FRESH_MS":1500,"STALE_SEC":2.0,"BOARD_MS_KEY":"시각ms","BOOT_COUNT_KEY":"부팅횟수","SUMMARY":{"차량":{"cards":[{"title":"위험 단계","key":"위험","risk":true,"sub":{"name":"원시","key":"원시위험"}},{"title":"TTC","key":"TTC","unit":"s","digits":1,"min":0,"max":900,"sub":{"name":"전방","key":"전방여부","map":{"1":"예","0":"아니오"}}},{"title":"계산 거리","key":"계산거리","unit":"m","digits":2,"min":0,"sub":{"name":"접근","key":"접근속도","unit":"m/s","digits":2}},{"title":"UWB 거리","key":"UWB보정거리","unit":"m","digits":2,"ok":"UWB유효","sub":[{"name":"각도","key":"UWB각도","unit":"°","digits":0,"when":"UWB각도유효"},{"name":"원시","key":"UWB원시거리","unit":"m","digits":2}]}],"lines":[{"label":"GPS","ok":"GPS유효","bad":"무효","parts":[{"name":"위성","key":"GPS위성"},{"name":"HDOP","key":"GPS_HDOP","digits":2},{"name":"속도","key":"속도","unit":"m/s","digits":2},{"name":"방향","key":"방향","unit":"°","digits":0},{"key":"상대보정","map":{"1":"영점 맞춤","0":"영점 전"}},{"key":"상대보정중","map":{"1":"영점 잡는 중"}}]},{"label":"UWB","ok":"UWB유효","bad":"무효","parts":[{"key":"UWB보정","map":{"1":"보정됨","0":"보정 전"}},{"key":"UWB보정중","map":{"1":"보정 중"}},{"name":"각도","key":"UWB각도","unit":"°","digits":0,"when":"UWB각도유효"},{"key":"UWB각도유효","map":{"0":"각도 모름"}},{"name":"신뢰","key":"UWB각도신뢰","min":1},{"key":"UWB가림","map":{"1":"가림","0":"트임"}},{"name":"접근","key":"UWB접근속도","unit":"m/s","digits":2},{"name":"경과","key":"UWB경과ms","unit":"ms","digits":0,"min":0},{"name":"전송","key":"UWB전송","rate":true}]},{"label":"IMU","ok_eq":{"IMU보정":"3/3/3/3","IMU정렬":"1"},"bad":"보정 필요","parts":[{"name":"S/G/A/M","key":"IMU보정"},{"key":"IMU정렬","map":{"1":"정렬됨","0":"정렬 전"}},{"name":"차체","key":"차체방향","unit":"°","digits":0}]},{"label":"RSSI","fresh":"RSSI경과ms","bad":"끊김","parts":[{"key":["RSSI평활","RSSI원시"],"unit":"dBm","digits":0},{"name":"추정","key":"RSSI거리","unit":"m","digits":1,"min":0}]},{"label":"통신","parts":[{"name":"송신","key":"송신","rate":true},{"name":"지팡이 수신","key":"지팡이수신","rate":true},{"name":"RSU","key":"RSU위험경과ms","scale":0.001,"unit":"s 전","digits":1,"min":0}]}]},"지팡이":{"cards":[{"title":"위험 단계","key":"위험","risk":true},{"title":"차량 경고","key":"차량위험","risk":true,"age":"차량위험경과ms","sub":{"name":"경과","key":"차량위험경과ms","scale":0.001,"unit":"s","digits":1,"min":0}},{"title":"RSU 경고","key":"RSU위험","risk":true,"age":"RSU위험경과ms","sub":{"name":"경과","key":"RSU위험경과ms","scale":0.001,"unit":"s","digits":1,"min":0}},{"title":"UWB 거리","key":"UWB보정거리","unit":"m","digits":2,"ok":"UWB유효","sub":{"name":"원시","key":"UWB원시거리","unit":"m","digits":2}}],"lines":[{"label":"GPS","ok":"GPS유효","bad":"무효","parts":[{"name":"위성","key":"GPS위성"},{"name":"HDOP","key":"GPS_HDOP","digits":2},{"name":"속도","key":"속도","unit":"m/s","digits":2},{"name":"GPS 복구","key":"GPS복구횟수","unit":"회","min":1}]},{"label":"방향","ok":"IMU방향유효","bad":"무효","parts":[{"key":"IMU방향","unit":"°","digits":0}]},{"label":"IMU","ok_eq":{"IMU보정":"3/3/3/3","IMU정렬":"1"},"bad":"보정 필요","parts":[{"name":"S/G/A/M","key":"IMU보정"},{"key":"IMU정렬","map":{"1":"정렬됨","0":"정렬 전"}}]},{"label":"UWB","ok":"UWB유효","bad":"무효","parts":[{"key":"UWB보정","map":{"1":"보정됨","0":"보정 전"}},{"name":"접근","key":"UWB접근속도","unit":"m/s","digits":2},{"name":"경과","key":"UWB경과ms","unit":"ms","digits":0,"min":0},{"name":"수신","key":"UWB수신","rate":true}]},{"label":"RSSI","fresh":"RSSI경과ms","bad":"끊김","parts":[{"key":["RSSI평활","RSSI원시"],"unit":"dBm","digits":0}]},{"label":"통신","parts":[{"name":"송신","key":"송신","rate":true},{"name":"차량 수신","key":"차량수신","rate":true}]}]}},"QUICK_COMMANDS":{"차량":[["IMU 상태","imu"],["IMU 저장","imusave"],["UWB 상태","uwb status"]],"지팡이":[["IMU 상태","imu"],["IMU 저장","imusave"]]},"MODE_COMMANDS":[["운영 모드",["logmode 0","rate 500"]],["상세 기록 모드",["logmode 1","rate 100"]]],"EXACT_GROUPS":{"속도":"GPS","위도":"GPS","경도":"GPS","방향":"방향·IMU","방향오차":"위험·경보","송신":"통신"},"VALUE_GROUPS":[["UWB",["UWB","uwb"],[]],["RSSI",["RSSI","rssi"],[]],["GPS",["GPS","gps","원시위도","원시경도"],["GPS","위성","HDOP","상대보정","보정동쪽","보정북쪽","lat","lng"]],["방향·IMU",["IMU","BNO"],["방향","가속도","자이로","자력계","자기모델","요회전","후진","스윙","heading","yaw"]],["충격",[],["충격","impact"]],["위험·경보",[],["위험","TTC","거리","접근속도","전방","음성","risk","ttc","dist"]],["통신",[],["송신","수신","전송","seq","lost"]],["시스템",[],["시각","부팅","리셋","상태","heap","메모리"]]],"GROUP_ORDER":["위험·경보","UWB","GPS","방향·IMU","RSSI","통신","충격","시스템","기타"],"NO_SPACE_UNITS":["°","%","회"]}</script>

<script>
var SPEC = JSON.parse(document.getElementById("viewer-spec").textContent);
var NODE_TITLE = {car: "차량", cane: "지팡이"};
var logLines = [], paused = false, timer = null;
var prev = {car:{}, cane:{}};
var cur = {car:{}, cane:{}};            // 최신 값 (요약 계산용)
var hist = {car:{}, cane:{}};           // 누적 횟수 기록 → 초당 증가량
var ages = {car:-1, cane:-1};
var lastMs = {car:null, cane:null}, reboots = {car:0, cane:0};
var lastBoots = {car:null, cane:null}, lastRebootAt = {car:0, cane:0};
var recording = false, recStart = 0, recName = "";
var recRows = {car:[], cane:[]}, recKeys = {car:[], cane:[]}, recLog = [];

function esc(s){return String(s).replace(/[&<>]/g,function(c){
  return {'&':'&amp;','<':'&lt;','>':'&gt;'}[c];});}

function stamp(){
  var d = new Date();
  return ("0"+d.getHours()).slice(-2)+":"+("0"+d.getMinutes()).slice(-2)+
         ":"+("0"+d.getSeconds()).slice(-2);
}

function pushLog(line){
  logLines.push(stamp()+"  "+line);
  if (logLines.length > 600) logLines.splice(0, 200);
  if (recording) recLog.push(stamp()+"  "+line);
}

function parseSections(text){
  var out = {}, cur = null, lines = text.split("\n");
  for (var i=0;i<lines.length;i++){
    var l = lines[i];
    if (l.indexOf("###") === 0){ cur = l.slice(3).trim(); out[cur] = []; continue; }
    if (cur) out[cur].push(l);
  }
  return out;
}

function toPairs(lines){
  var pairs = [];
  for (var i=0;i<lines.length;i++){
    var p = lines[i].indexOf(":");
    if (p > 0) pairs.push([lines[i].slice(0,p), lines[i].slice(p+1)]);
  }
  return pairs;
}

// ── 요약 계산 (dual_serial_viewer.py 의 format_field 등과 같은 규칙) ──
function lookup(values, key){
  var keys = (typeof key === "string") ? [key] : key;
  for (var i=0;i<keys.length;i++)
    if (values[keys[i]] !== undefined) return [keys[i], values[keys[i]]];
  return [null, null];
}

function cardSubs(card){
  if (!card.sub) return [];
  return Array.isArray(card.sub) ? card.sub : [card.sub];
}

function rateKeys(node){
  var cfg = SPEC.SUMMARY[NODE_TITLE[node]], out = [];
  if (!cfg) return out;
  var add = function(s){ if (s && s.rate) out = out.concat(
    typeof s.key === "string" ? [s.key] : s.key); };
  cfg.cards.forEach(function(c){ add(c); cardSubs(c).forEach(add); });
  cfg.lines.forEach(function(l){ l.parts.forEach(add); });
  return out;
}

function addCounter(node, key, raw, t){
  var v = parseFloat(raw);
  if (isNaN(v)) return;
  var h = hist[node][key] || (hist[node][key] = []);
  if (h.length && (v < h[h.length-1][1] || t < h[h.length-1][0])) h.length = 0;
  h.push([t, v]);
  while (h.length > 2 && t - h[0][0] > 3000) h.shift();
}

function rateOf(node, key){
  var h = hist[node][key];
  if (!h || h.length < 2) return null;
  if (ages[node] < 0 || ages[node] > SPEC.STALE_SEC*1000) return 0;
  var span = (h[h.length-1][0] - h[0][0]) / 1000;
  if (span < 0.8) return null;
  return (h[h.length-1][1] - h[0][1]) / span;
}

function fmt(spec, node){
  if (spec.when !== undefined){
    var cond = lookup(cur[node], spec.when)[1];
    if (cond === null) return null;
    cond = String(cond).trim();
    if (cond !== "1" && cond !== "true" && cond !== "True") return null;
  }
  var kv = lookup(cur[node], spec.key), raw = kv[1];
  if (raw === null) return null;
  raw = String(raw).trim();
  if (spec.rate){ var r = rateOf(node, kv[0]); return r === null ? null : r.toFixed(1)+"/s"; }
  if (spec.map) return spec.map.hasOwnProperty(raw) ? spec.map[raw] : null;
  var text = raw;
  if (spec.digits !== undefined || spec.min !== undefined ||
      spec.max !== undefined || spec.scale !== undefined){
    var num = parseFloat(raw);
    if (isNaN(num)) return raw || null;
    if (spec.min !== undefined && num < spec.min) return null;
    if (spec.max !== undefined && num >= spec.max) return null;
    num *= (spec.scale === undefined ? 1 : spec.scale);
    text = spec.digits !== undefined ? num.toFixed(spec.digits) : String(num);
  }
  if (!text) return null;
  if (spec.unit)
    text += (SPEC.NO_SPACE_UNITS.indexOf(spec.unit) >= 0 ? "" : " ") + spec.unit;
  return text;
}

function specOk(spec, node){
  if (spec.ok_eq !== undefined){
    var result = null;
    for (var key in spec.ok_eq){
      if (!spec.ok_eq.hasOwnProperty(key)) continue;
      var got = lookup(cur[node], key)[1];
      if (got === null) continue;
      if (String(got).trim() !== spec.ok_eq[key]) return false;
      result = true;
    }
    return result;
  }
  if (spec.ok !== undefined){
    var raw = lookup(cur[node], spec.ok)[1];
    if (raw === null) return null;
    raw = String(raw).trim();
    return raw === "1" || raw === "true" || raw === "True";
  }
  if (spec.fresh !== undefined){
    var ms = parseFloat(lookup(cur[node], spec.fresh)[1]);
    if (isNaN(ms)) return null;
    return ms >= 0 && ms <= SPEC.FRESH_MS;
  }
  return null;
}

function riskLevel(spec, node){
  var raw = lookup(cur[node], spec.key)[1];
  if (raw === null) return null;
  if (spec.age !== undefined){
    var age = parseFloat(lookup(cur[node], spec.age)[1]);
    if (!isNaN(age) && age < 0) return null;   // 경과 -1 = 아직 못 받음
  }
  var lv = parseInt(raw, 10);
  return isNaN(lv) ? null : lv;
}

function renderSummary(node, stale){
  var cfg = SPEC.SUMMARY[NODE_TITLE[node]] || {cards:[], lines:[]};
  var html = "<div class='cards'>";
  cfg.cards.forEach(function(spec){
    var sub = "", cls = "card", style = "", text;
    var subs = cardSubs(spec);
    for (var i = 0; i < subs.length; i++){   // 보여 줄 수 있는 첫 칸
      var s = fmt(subs[i], node);
      if (s !== null){ sub = ((subs[i].name || "") + " " + s).trim(); break; }
    }
    if (spec.risk){
      var lv = riskLevel(spec, node);
      if (lv === null){ text = "—"; cls += " dim"; }
      else {
        text = lv + " " + (SPEC.RISK_NAMES[lv] || "");
        cls += " risk";
        style = " style='background:" +
          (stale ? "#999999" : (SPEC.RISK_COLORS[lv] || "#999999")) + "'";
      }
    } else {
      text = fmt(spec, node);
      if (text === null || stale || specOk(spec, node) === false) cls += " dim";
      if (text === null) text = "—";
    }
    html += "<div class='" + cls + "'" + style + "><div class='ct'>" +
            esc(spec.title) + "</div><div class='cv'>" + esc(text) +
            "</div><div class='cs'>" + esc(sub) + "</div></div>";
  });
  html += "</div>";
  cfg.lines.forEach(function(spec){
    var ok = specOk(spec, node), parts = [];
    if (ok === false) parts.push(spec.bad || "무효");
    spec.parts.forEach(function(p){
      var s = fmt(p, node);
      if (s !== null) parts.push(p.name ? p.name + " " + s : s);
    });
    var dot = ok === null ? "" : (ok && !stale ? " on" : " off");
    var dim = stale || ok === false || !parts.length;
    html += "<div class='line" + (dim ? " dim" : "") + "'><span class='ldot" +
            dot + "'></span><b>" + esc(spec.label) + "</b><span>" +
            esc(parts.length ? parts.join(" · ") : "—") + "</span></div>";
  });
  document.getElementById(node === "car" ? "sumCar" : "sumCane").innerHTML = html;

  var w = document.getElementById(node === "car" ? "warnCar" : "warnCane");
  if (reboots[node]){
    var reason = lookup(cur[node], "리셋원인")[1];
    w.textContent = "⚠ 재부팅 " + reboots[node] + "회" + (reason ? " (" + reason + ")" : "");
  } else w.textContent = "";
}

// 새로 받은 값 묶음을 요약 계산용으로 반영 (재부팅 감지·초당 증가량 포함)
function absorb(node, pairs){
  if (!pairs.length) return;
  var vals = {};
  pairs.forEach(function(p){ vals[p[0]] = p[1].trim(); });
  var ms = parseFloat(vals[SPEC.BOARD_MS_KEY]), fresh = true;
  var boots = parseInt(vals[SPEC.BOOT_COUNT_KEY], 10), note = null;
  if (!isNaN(ms)){
    if (lastMs[node] !== null && ms + 1000 < lastMs[node])
      note = SPEC.BOARD_MS_KEY + " " + lastMs[node] + " → " + ms;
    fresh = lastMs[node] === null || ms !== lastMs[node];
    lastMs[node] = ms;
  }
  if (!isNaN(boots)){
    if (lastBoots[node] !== null && boots > lastBoots[node] && !note)
      note = SPEC.BOOT_COUNT_KEY + " " + lastBoots[node] + " → " + boots;
    lastBoots[node] = boots;
  }
  // 한 번의 재부팅을 두 값이 같이 알려도 5초 안이면 한 번만 센다.
  if (note && Date.now() - lastRebootAt[node] > 5000){
    lastRebootAt[node] = Date.now();
    reboots[node]++;
    hist[node] = {};
    pushLog("[뷰어] " + NODE_TITLE[node] + " 재부팅 감지: " + note);
  }
  cur[node] = vals;
  if (!fresh) return;   // 같은 묶음을 한 번 더 받은 것
  var t = isNaN(ms) ? Date.now() : ms;
  rateKeys(node).forEach(function(k){
    if (vals[k] !== undefined) addCounter(node, k, vals[k], t);
  });
}

// ── 전체 값: 값 이름으로 묶음을 정한다 ──
function classify(key){
  if (SPEC.EXACT_GROUPS.hasOwnProperty(key)) return SPEC.EXACT_GROUPS[key];
  var g, i, j;
  for (i=0;i<SPEC.VALUE_GROUPS.length;i++){
    g = SPEC.VALUE_GROUPS[i];
    for (j=0;j<g[1].length;j++) if (key.indexOf(g[1][j]) === 0) return g[0];
  }
  for (i=0;i<SPEC.VALUE_GROUPS.length;i++){
    g = SPEC.VALUE_GROUPS[i];
    for (j=0;j<g[2].length;j++) if (key.indexOf(g[2][j]) >= 0) return g[0];
  }
  return "기타";
}

function groupBox(node, group){
  var order = SPEC.GROUP_ORDER.indexOf(group);
  if (order < 0) order = SPEC.GROUP_ORDER.length;
  var id = "g_" + node + "_" + order;
  var el = document.getElementById(id);
  if (el) return el;
  el = document.createElement("details");
  el.id = id;
  el.setAttribute("data-order", order);
  el.innerHTML = "<summary></summary><table></table>";
  var box = document.getElementById(node === "car" ? "grpCar" : "grpCane");
  var before = null;
  for (var i=0;i<box.children.length;i++){
    if (parseInt(box.children[i].getAttribute("data-order"), 10) > order){
      before = box.children[i]; break;
    }
  }
  box.insertBefore(el, before);
  return el;
}

function fillGroups(node, pairs){
  var byGroup = {};
  pairs.forEach(function(p){
    var g = classify(p[0]);
    (byGroup[g] = byGroup[g] || []).push(p);
  });
  Object.keys(byGroup).forEach(function(g){
    var el = groupBox(node, g), rows = "";
    byGroup[g].forEach(function(p){
      var k = p[0], v = p[1];
      var chg = prev[node][k] !== undefined && prev[node][k] !== v;
      prev[node][k] = v;
      rows += "<tr" + (chg ? " class='chg'" : "") + "><td class='k'>" + esc(k) +
              "</td><td class='v'>" + esc(v) + "</td></tr>";
    });
    el.querySelector("summary").textContent = g + " (" + byGroup[g].length + ")";
    el.querySelector("table").innerHTML = rows;
  });
}

function markAge(dotId, ageId, ms){
  var dot = document.getElementById(dotId), age = document.getElementById(ageId);
  if (ms < 0 || ms > 60000){ dot.className = "dot"; age.textContent = "수신 없음"; return false; }
  dot.className = ms < 3000 ? "dot on" : "dot old";
  age.textContent = (ms/1000).toFixed(1)+"초 전";
  return true;
}

function recordRow(node, pairs){
  if (!recording || !pairs.length) return;
  var row = {"시각": stamp(),
             "경과초": ((Date.now()-recStart)/1000).toFixed(3)};
  for (var i=0;i<pairs.length;i++){
    row[pairs[i][0]] = pairs[i][1];
    if (recKeys[node].indexOf(pairs[i][0]) < 0) recKeys[node].push(pairs[i][0]);
  }
  recRows[node].push(row);
}

function refresh(){
  fetch("/data", {cache:"no-store"}).then(function(r){return r.text();})
  .then(function(text){
    var s = parseSections(text);
    var meta = {};
    (s.META||[]).forEach(function(l){
      var p = l.indexOf(":"); if (p>0) meta[l.slice(0,p)] = parseInt(l.slice(p+1),10);
    });

    ages.car = meta.carAge === undefined ? -1 : meta.carAge;
    ages.cane = meta.caneAge === undefined ? -1 : meta.caneAge;
    var carOk = markAge("dotCar","ageCar", ages.car);
    var caneOk = markAge("dotCane","ageCane", ages.cane);
    document.getElementById("conn").textContent =
      (carOk?"차량 O":"차량 X") + " / " + (caneOk?"지팡이 O":"지팡이 X");

    var carPairs = toPairs(s.CAR||[]), canePairs = toPairs(s.CANE||[]);
    absorb("car", carPairs);
    absorb("cane", canePairs);
    if (!paused){
      fillGroups("car", carPairs);
      fillGroups("cane", canePairs);
    }
    var staleMs = SPEC.STALE_SEC * 1000;
    renderSummary("car", ages.car < 0 || ages.car > staleMs);
    renderSummary("cane", ages.cane < 0 || ages.cane > staleMs);
    recordRow("car", carPairs);
    recordRow("cane", canePairs);

    // 값 줄은 위 요약·전체 값에 나오므로 로그에는 '값 줄도 보기'일 때만.
    if (!paused && document.getElementById("showVals").checked){
      if (carPairs.length) pushLog("[차량] " + carPairs.map(function(p){
        return p[0]+":"+p[1];}).join(" "));
      if (canePairs.length) pushLog("[지팡이] " + canePairs.map(function(p){
        return p[0]+":"+p[1];}).join(" "));
    }
    (s.REPLY||[]).forEach(function(l){ if (l.trim()) pushLog(l); });
    draw();
  })
  .catch(function(){
    document.getElementById("conn").textContent = "보드 연결 끊김";
  });
}

function draw(){
  var el = document.getElementById("log");
  el.textContent = logLines.slice(-300).join("\n");
  if (!paused) el.scrollTop = el.scrollHeight;
  document.getElementById("pauseBtn").textContent = paused ? "재개" : "일시정지";
  if (recording){
    var sec = Math.floor((Date.now()-recStart)/1000);
    document.getElementById("recInfo").innerHTML =
      "<span class='rec'>기록 중 " + ("0"+Math.floor(sec/60)).slice(-2) + ":" +
      ("0"+(sec%60)).slice(-2) + " (" +
      (recRows.car.length + recRows.cane.length) + "행)</span>";
  }
}

function send(target, text){
  var el = document.getElementById(target === "car" ? "cmdCar" : "cmdCane");
  if (text === undefined) text = el.value.trim();
  if (!text) return;
  pushLog("> [" + (target==="car"?"차량":"지팡이") + "] " + text);
  if (recording) recLog.push(stamp()+"  > ["+target+"] "+text);
  fetch("/cmd?target="+target+"&text="+encodeURIComponent(text))
    .then(function(){ setTimeout(refresh, 250); });
  if (el.value) el.value = "";
  draw();
}
function quick(target, text){ send(target, text); }

function toggleRecord(){
  if (!recording){
    recName = (document.getElementById("recName").value || "기록").trim();
    recording = true; recStart = Date.now();
    recRows = {car:[], cane:[]}; recKeys = {car:[], cane:[]}; recLog = [];
    document.getElementById("recBtn").textContent = "기록 중지";
    document.getElementById("dlBox").innerHTML = "";
  } else {
    recording = false;
    document.getElementById("recBtn").textContent = "기록 시작";
    document.getElementById("recInfo").textContent =
      "기록 완료 — 아래 버튼으로 저장";
    showDownloads();
  }
}

function toCsv(node){
  var keys = ["시각","경과초"].concat(recKeys[node]);
  var out = "﻿" + keys.join(",") + "\n";
  for (var i=0;i<recRows[node].length;i++){
    var r = recRows[node][i], line = [];
    for (var j=0;j<keys.length;j++){
      var v = r[keys[j]] === undefined ? "" : String(r[keys[j]]);
      line.push(v.indexOf(",") >= 0 ? '"'+v+'"' : v);
    }
    out += line.join(",") + "\n";
  }
  return out;
}

function fileName(suffix){
  var d = new Date(recStart), p = function(n){return ("0"+n).slice(-2);};
  return recName + "_" + d.getFullYear() + "-" + p(d.getMonth()+1) + "-" +
         p(d.getDate()) + "_" + p(d.getHours()) + "-" + p(d.getMinutes()) +
         "-" + p(d.getSeconds()) + "_" + suffix;
}

function makeLink(text, content, name, type){
  var blob = new Blob([content], {type:type||"text/csv;charset=utf-8"});
  var a = document.createElement("a");
  a.className = "dl"; a.textContent = text;
  a.href = URL.createObjectURL(blob); a.download = name;
  return a;
}

function showDownloads(){
  var box = document.getElementById("dlBox");
  box.innerHTML = "";
  if (recRows.car.length)
    box.appendChild(makeLink("차량 CSV 저장", toCsv("car"), fileName("차량.csv")));
  if (recRows.cane.length)
    box.appendChild(makeLink("지팡이 CSV 저장", toCsv("cane"), fileName("지팡이.csv")));
  if (recLog.length)
    box.appendChild(makeLink("로그 저장", recLog.join("\n"),
                             fileName("로그.txt"), "text/plain;charset=utf-8"));
}

function restartTimer(){
  if (timer) clearInterval(timer);
  timer = setInterval(refresh, parseInt(document.getElementById("interval").value,10));
}

document.getElementById("cmdCar").addEventListener("keydown", function(e){
  if (e.key === "Enter") send("car"); });
document.getElementById("cmdCane").addEventListener("keydown", function(e){
  if (e.key === "Enter") send("cane"); });

// 빠른 명령·모드 버튼은 dual_serial_viewer.py 의 QUICK_COMMANDS·MODE_COMMANDS 와 같다.
function makeButton(label, onClick){
  var b = document.createElement("button");
  b.className = "sub"; b.textContent = label; b.onclick = onClick;
  return b;
}
["car", "cane"].forEach(function(node){
  var box = document.getElementById(node === "car" ? "quickCar" : "quickCane");
  (SPEC.QUICK_COMMANDS[NODE_TITLE[node]] || []).forEach(function(q){
    box.appendChild(makeButton(q[0], function(){ send(node, q[1]); }));
  });
});
(SPEC.MODE_COMMANDS || []).forEach(function(m){
  document.getElementById("modeBtns").appendChild(makeButton(m[0], function(){
    ["car", "cane"].forEach(function(node){
      m[1].forEach(function(command){ send(node, command); });
    });
  }));
});

refresh();
restartTimer();
</script>
</body>
</html>
)HTMLPAGE";
