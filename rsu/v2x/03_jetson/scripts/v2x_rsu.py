#!/usr/bin/env python3
import serial, json, time, math, csv, sys

PORT = "/dev/ttyUSB0"
BAUD = 115200

PED_LAT = 37.000000
PED_LNG = 127.000150
PED_HEADING = 0.0

TTC_CAUTION = 5.5
TTC_WARNING = 3.0
TTC_DANGER  = 1.5
STALE_SEC = 2.0
SEND_ON_CHANGE_ONLY = True
LOG_FILE = "v2x_risk_log.csv"

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360) % 360

def compute_risk(veh, ped_lat, ped_lng):
    dist = haversine_m(veh["lat"], veh["lng"], ped_lat, ped_lng)
    speed = max(0.0, veh.get("speed_mps", 0.0))
    veh_hdg = veh.get("heading_deg", 0.0)
    to_ped = bearing_deg(veh["lat"], veh["lng"], ped_lat, ped_lng)
    diff = abs((veh_hdg - to_ped + 180) % 360 - 180)
    closing = speed * math.cos(math.radians(diff))
    if closing <= 0.1:
        return 0, float("inf"), dist, closing, "receding_or_stopped"
    ttc = dist / closing
    if ttc < TTC_DANGER:    risk, reason = 3, "ttc<1.5s"
    elif ttc < TTC_WARNING: risk, reason = 2, "ttc<3.0s"
    elif ttc < TTC_CAUTION: risk, reason = 1, "ttc<5.5s"
    else:                   risk, reason = 0, "ttc>=5.5s"
    return risk, ttc, dist, closing, reason

def main():
    sim_mode  = "--sim" in sys.argv
    reply_off = "--reply-off" in sys.argv
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"[연결] {PORT} @ {BAUD}")
    print(f"[모드] {'SIM' if sim_mode else '실측'}{' / 회신OFF' if reply_off else ''}")
    print(f"[보행자] lat={PED_LAT}, lng={PED_LNG}")
    print("-" * 78)
    vehicles = {}
    last_sent_risk = None
    sim_dist = 60.0
    logf = open(LOG_FILE, "w", newline="")
    writer = csv.writer(logf)
    writer.writerow(["rx_wall","node_id","distance_m","closing_mps","ttc_s","risk_level","reason","sent"])
    try:
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw.startswith("{"): continue
            try: d = json.loads(raw)
            except: continue
            if d.get("type") != "vehicle": continue
            nid = d.get("node_id", 0)
            vehicles[nid] = {"lat": d.get("lat",0.0), "lng": d.get("lng",0.0),
                             "speed_mps": d.get("speed_mps",0.0),
                             "heading_deg": d.get("heading_deg",0.0), "last_rx": time.time()}
            now = time.time()
            best = (0, float("inf"), 0.0, 0.0, "no_vehicle", None)
            for vid, v in vehicles.items():
                if now - v["last_rx"] > STALE_SEC: continue
                if sim_mode:
                    closing = 8.0
                    ttc = sim_dist / closing
                    if   ttc < TTC_DANGER:  r, rs = 3, "sim ttc<1.5s"
                    elif ttc < TTC_WARNING: r, rs = 2, "sim ttc<3.0s"
                    elif ttc < TTC_CAUTION: r, rs = 1, "sim ttc<5.5s"
                    else:                   r, rs = 0, "sim ttc>=5.5s"
                    cand = (r, ttc, sim_dist, closing, rs, vid)
                else:
                    r, ttc, dist, closing, rs = compute_risk(v, PED_LAT, PED_LNG)
                    cand = (r, ttc, dist, closing, rs, vid)
                if cand[0] > best[0]: best = cand
            risk, ttc, dist, closing, reason, src_nid = best
            if sim_mode: sim_dist = max(2.0, sim_dist - 0.5)
            ttc_str = f"{ttc:5.1f}s" if ttc != float("inf") else "  inf"
            print(f"RISK={risk}  dist={dist:6.1f}m  closing={closing:5.2f}m/s  TTC={ttc_str}  ({reason})  veh={src_nid}")
            sent = ""
            if not reply_off:
                if (not SEND_ON_CHANGE_ONLY) or (risk != last_sent_risk):
                    msg = json.dumps({"target_id": 0, "risk": risk})
                    ser.write((msg + "\n").encode())
                    last_sent_risk = risk
                    sent = msg
                    print(f"   -> 회신: {msg}")
            writer.writerow([f"{time.time():.3f}", src_nid, f"{dist:.2f}", f"{closing:.2f}", f"{ttc:.2f}", risk, reason, sent])
            logf.flush()
    except KeyboardInterrupt:
        print("\n[종료] RISK 0 회신")
        if not reply_off: ser.write(b'{"target_id":0,"risk":0}\n')
        logf.close()
        print(f"[로그] {LOG_FILE}")

if __name__ == "__main__":
    try: main()
    except serial.SerialException as e:
        print(f"[포트 에러] {e}\n→ sudo fuser -k /dev/ttyUSB0")
