# Field Results

> 🇰🇷 [한국어 버전](field-results.ko.md) · back to [← README.md](../README.md)

Not lab numbers — **logs recorded on a real road.** Every figure below is generated **without hand-editing** from the raw CSVs in [`data/`](../data/); the generation scripts are included too.

- Period: **2026-08-12 – 08-25 (7 days, 47 sessions)**
- Basis: **104,511** GPS-valid risk decisions, **95** extracted approach events
- Site: campus sidewalk–road boundary (the vehicle node rides an RC car as a speed-scaled stand-in)

---

## 1. Real-time risk decision — one real approach, second by second

![Real approach test timeline](images/approach-timeline.svg)

**One approach from the 2026-08-17 12:22 session.** A vehicle closes from 32 m until it brushes past the cane, and the risk level climbs and clears on its own.

| Time | Distance | TTC | Level | Why |
| ---: | ---: | ---: | :--- | :--- |
| 0.0 s | 32.6 m | 10.8 s | **0 No alert** | far, approaching |
| ~8–12 s | ~16 m | — | **0 No alert** | brief stop (closing ≈ 0) → DCPA gate suppresses |
| 14.6 s | 13.5 m | 7.9 s | **1 Caution** | closing again, score reaches 20 |
| 17.3 s | 6.6 m | 4.3 s | **2 Warning** | score reaches 45 |
| 21.0 s | 2.2 m | **1.97 s** | **3 Danger** | **safety floor**: TTC drops below 2 s → top level regardless of score |
| 24.4 s | 1.3 m | — | **0 No alert** | vehicle stops (closing ≈ 0) → auto-clears |

The key moment is **21.0 s**. The rule score was only 63 (still the Warning band), but **as time-to-collision fell to 1.97 s the "safety-floor" rule forced the level straight to Danger.** This records the safety floor operating under the inputs and decision conditions of that example; it does not guarantee an alert under every sensor state or scenario.

---

## 2. "Closer means more dangerous," verified on 100k decisions

![Risk level by distance](images/distance-risk.svg)

A single approach could be luck, so we binned **all 104,511 GPS-valid decisions across 7 days / 47 sessions** by distance.

| Distance | Decisions | Warning-or-higher (LV1+) |
| ---: | ---: | ---: |
| 0–2 m | 1,183 | **73.9 %** |
| 2–4 m | 6,322 | 28.7 % |
| 4–6 m | 10,875 | 18.6 % |
| 6–10 m | 22,287 | 13.8 % |
| 10–15 m | 21,171 | 7.7 % |
| 15–30 m | 25,893 | 5.2 % |
| 30+ m | 16,780 | 1.4 % |

The warning rate rises **monotonically** as the vehicle gets closer — 100k real decisions show the intended behavior on their own.

> **Honestly:** a small fraction of warnings remain beyond 15 m (5.2 %, 1.4 %), from GPS noise and TTC-based early warnings. We found these in the field and tightened the gates (§3) to reduce them — and left them in the aggregate rather than hiding them.

---

## 3. What the field taught us

- **Tightened the DCPA gate (7.5 m → 4.5 m).** Backed by measured sidewalk–road gaps (3–10 m, 3 m at the narrowest). Replay held real-approach detection (−1) while cutting idle false alarms ~20 %.
- **UWB as a GPS-gap safety net (first outdoor success, 8/25).** In a session where the cane GPS never locked, UWB ranging carried 100 % of the decisions (LV1–LV3). A GPS-only system would have been blind.
- **AI operating status updated (reviewed September 28, 2026).** AI-off settings in some earlier trials do not describe all later operation. In the September 19 public TX log, 39 of 11,951 valid records have `level_source=model` (9 changes, 30 heartbeats). The field step8 runner uses a tree ensemble, distinct from the separate Transformer test metrics. [Code and log evidence](current-ai-status.md)
- **Rules plus AI upgrades.** The model cannot lower rule-generated alerts; model errors fall back to rules. This is not independent triple redundancy against shared position-input faults. The field step8 runner leaves static zone risk at its default of 0.



---

## 4. Data & reproduction

- Raw logs: [`data/`](../data/) — per-day session folders (see [data/README.md](../data/README.md))
- Decision CSVs: each session's `risk_tx_*.csv` (distance, TTC, closing speed, level, GPS, model probability)
- Figure scripts: [`scripts/figures/`](../scripts/figures/) — regenerate the SVGs from the same CSVs.

> **Method note:** Sections 1–2 summarize the historical August 12–25 logs; the September 19 AI evidence in §3 is a separate session. The 104,511 rows are not independent trials or labeled hazard outcomes. Alarm frequency by distance does not establish detection or false-alarm rates.
