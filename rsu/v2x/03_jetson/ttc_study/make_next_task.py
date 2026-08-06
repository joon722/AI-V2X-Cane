#!/usr/bin/env python3
"""다음 작업 지시서를 PDF로 만든다.

시뮬레이션이 알아낸 것이 실시간 판단에 전달되지 않는 문제를 다룬다. 다음 세션이
이 문서 하나만 보고 착수할 수 있도록, 왜 필요한지·어디를 고치는지·어떻게
검증하는지를 순서대로 적는다.
"""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (PageBreak, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)

HERE = Path(__file__).parent
OUT = HERE / "다음작업_시뮬레이션과_실시간_연결.pdf"

pdfmetrics.registerFont(TTFont("Malgun", "C:/Windows/Fonts/malgun.ttf"))
pdfmetrics.registerFont(TTFont("MalgunBd", "C:/Windows/Fonts/malgunbd.ttf"))

B = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("t", parent=B["Title"], fontName="MalgunBd", fontSize=17,
                            leading=23, spaceAfter=3),
    "sub": ParagraphStyle("s", parent=B["Normal"], fontName="Malgun", fontSize=10,
                          leading=14, alignment=TA_CENTER,
                          textColor=colors.HexColor("#555"), spaceAfter=14),
    "h1": ParagraphStyle("h1", parent=B["Heading1"], fontName="MalgunBd", fontSize=13,
                         leading=18, spaceBefore=13, spaceAfter=6,
                         textColor=colors.HexColor("#1f4e79")),
    "h2": ParagraphStyle("h2", parent=B["Heading2"], fontName="MalgunBd", fontSize=11,
                         leading=15, spaceBefore=9, spaceAfter=4,
                         textColor=colors.HexColor("#2f6ea5")),
    "body": ParagraphStyle("b", parent=B["Normal"], fontName="Malgun", fontSize=9.6,
                           leading=15, alignment=TA_JUSTIFY, spaceAfter=6),
    "box": ParagraphStyle("bx", parent=B["Normal"], fontName="Malgun", fontSize=9.4,
                          leading=14.5, alignment=TA_JUSTIFY, leftIndent=9,
                          rightIndent=9, spaceAfter=6, borderPadding=8,
                          backColor=colors.HexColor("#f4f7fa")),
    "warn": ParagraphStyle("w", parent=B["Normal"], fontName="Malgun", fontSize=9.4,
                           leading=14.5, alignment=TA_JUSTIFY, leftIndent=9,
                           rightIndent=9, spaceAfter=6, borderPadding=8,
                           backColor=colors.HexColor("#fdeaea")),
    "code": ParagraphStyle("c", parent=B["Normal"], fontName="Malgun", fontSize=8.4,
                           leading=12.5, leftIndent=10, spaceBefore=3, spaceAfter=7,
                           backColor=colors.HexColor("#f2f4f6"), borderPadding=6),
    "cap": ParagraphStyle("cp", parent=B["Normal"], fontName="Malgun", fontSize=8.4,
                          leading=12, textColor=colors.HexColor("#666"), spaceAfter=8),
}


def P(t, s="body"):
    return Paragraph(t, S[s])


def T(rows, widths, hi=None):
    data = [[Paragraph(f"<b>{c}</b>", ParagraphStyle(
        "th", fontName="MalgunBd", fontSize=8.6, leading=11.5,
        textColor=colors.white, alignment=TA_CENTER)) for c in rows[0]]]
    for r in rows[1:]:
        data.append([Paragraph(str(c), ParagraphStyle(
            "td", fontName="Malgun", fontSize=8.6, leading=12)) for c in r])
    st = [
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2f6ea5")),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#c8d4e0")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f7f9fb")]),
    ]
    for i in (hi or []):
        st.append(("BACKGROUND", (0, i + 1), (-1, i + 1), colors.HexColor("#fdeaea")))
    t = Table(data, colWidths=widths)
    t.setStyle(TableStyle(st))
    return t


def build():
    s = []
    s += [
        P("다음 작업 — 시뮬레이션이 알아낸 것을 실시간 판단에 먹이기", "title"),
        P("zone_base_risk 연결 · 2026-08-06 작성 · 착수 시점 8/9 이후", "sub"),
    ]

    # ── 1. 문제 ────────────────────────────────────────────────────
    s += [
        P("1. 지금 무엇이 문제인가", "h1"),
        P("젯슨에서 두 흐름이 돌고 있는데 <b>서로 연결되어 있지 않다.</b> 시뮬레이션이 "
          "3초 앞을 예측해도 그 결과가 지팡이 판단에 아무 영향을 주지 않는다. 지도에만 "
          "그려지고 끝난다.", "body"),
        P("[시뮬레이션 흐름]  GCP 서버 SUMO 생성 → 젯슨 process_scenarios.py (cron 1분)<br/>"
          "                  → v3 트랜스포머 3초 예측 → 서버 업로드 → 위험지도<br/><br/>"
          "[실시간 흐름]      노드 GPS 5Hz → 브리지 → 젯슨 step8_send_risk.py (systemd)<br/>"
          "                  → 규칙 + 안전하한 + GBM → 지팡이 진동 (0.1초)", "code"),
        P("<b>간접 경로는 하나 있다.</b> 시뮬레이션 데이터로 모델을 학습해 "
          "risk_model.json으로 젯슨에 넣었으므로, 시뮬레이션이 \"판단하는 법\"은 이미 "
          "전달되고 있다. 없는 것은 <b>운영 중에 계속 갱신되는 지식</b>이다.", "box"),
    ]

    # ── 2. 자리 ────────────────────────────────────────────────────
    s += [
        P("2. 받을 자리는 이미 있다", "h1"),
        P("step7_risk.py의 팀 점수표에 구간 위험도를 받는 인자가 설계되어 있다.", "body"),
        P("def calculate_risk_score(distance_m, relative_speed_mps,<br/>"
          "                         vehicle_speed_mps, ttc, <b>zone_base_risk=0</b>):<br/>"
          "    ...<br/>"
          "    # 5. Hazard-zone correction: up to 5<br/>"
          "    score += min(max(zone_base_risk, 0), 5)", "code"),
        P("그런데 <b>step8이 이 값을 넘기지 않아 항상 0이다.</b> 100점 중 5점을 쓸 수 "
          "있는데 비워두고 있는 셈이다. auto_pipeline/process_scenarios.py도 "
          "\"SUMO에 구역 정보가 없으므로 0을 쓴다\"고 주석에 적어두었다.", "body"),
    ]

    # ── 3. 목표 구조 ───────────────────────────────────────────────
    s += [
        P("3. 만들 구조", "h1"),
        P("SUMO 시뮬 + 실측 이벤트<br/>"
          "        ↓<br/>"
          "   위험지도 API 구간별 집계   ← 이미 가동 중<br/>"
          "        ↓ 젯슨이 주기적으로 내려받아 캐시<br/>"
          "   현재 GPS → 구간 매칭<br/>"
          "        ↓<br/>"
          "   zone_base_risk → step7 점수표 → 위험 구간에서 더 일찍 경보", "code"),
        P("이렇게 되면 <b>\"데이터가 쌓일수록 지팡이가 똑똑해진다\"</b>가 성립한다. "
          "서버가 넓게 보고 엣지가 빠르게 반응하는 협력 구조이고, 발표에서도 시스템이 "
          "하나로 도는 그림이 된다.", "box"),
        Spacer(1, 2),
        P("이미 있는 것: 젯슨 → 서버 이벤트 업로드(deploy/upload_events.py, 1분 타이머), "
          "위험지도 API(Cloud Run), 구간 집계. <b>없는 것은 서버 → 젯슨 방향뿐이다.</b>", "cap"),
    ]

    s.append(PageBreak())

    # ── 4. 주의 ────────────────────────────────────────────────────
    s += [
        P("4. 지금 바로 넣으면 안 되는 이유", "h1"),
        P("<b>근거 없는 가중치가 되기 때문이다.</b> \"교차로니까 +5점\"을 주려면 "
          "교차로에서 실제로 더 자주 위험하다는 증거가 있어야 한다. 지금 가진 구간 "
          "위험도는 SUMO가 만든 값이고, 그 SUMO는 12,621스텝에서 위험 사례가 0건이었던 "
          "바로 그 시뮬레이터다.", "warn"),
        P("이번 연구가 zone을 의도적으로 뺀 것도 같은 이유다. 오라클 라벨은 물리적 "
          "충돌만 보므로, 교차로에서 실제로 더 자주 충돌하도록 모델링하지 않는 한 "
          "zone 가중치는 정당화되지 않는다(SPEC.md 12절).", "body"),
        P("<b>따라서 구조는 먼저 만들되 값은 데이터가 채우게 한다.</b> 실측 이벤트가 "
          "쌓이면 \"이 구간에서 위험이 N번 발생했다\"는 통계적 근거가 생기고, 그때 "
          "값을 넣으면 된다.", "box"),
    ]

    # ── 5. 단계 ────────────────────────────────────────────────────
    s += [
        P("5. 실행 단계", "h1"),
        T([
            ["단계", "할 일", "판정 기준", "담당"],
            ["1", "위험지도 API가 구간별 위험도를 <b>내려주는</b> 엔드포인트가 있는지 확인. "
                  "없으면 추가 요청 (GET /api/zones 형태)",
             "구간ID → 위험도 JSON을 받을 수 있다", "위험지도 담당"],
            ["2", "젯슨용 다운로더 작성. 주기적 갱신(10~30분), 로컬 캐시, "
                  "오프라인이면 마지막 캐시 사용",
             "네트워크가 끊겨도 엔진이 멈추지 않는다", "현준"],
            ["3", "현재 GPS → 구간 매칭. 구간 ID 형식은 1381099007_3 계열",
             "실측 로그의 좌표가 올바른 구간으로 매칭된다", "현준"],
            ["4", "step8 → step7에 zone_base_risk 전달. gate_params에 추가",
             "기존 테스트 255개 통과 유지", "현준"],
            ["5", "<b>효과 검증</b> (아래 6절). 검증 전에는 값을 0으로 두고 로깅만",
             "오라클 기준으로 성능이 오르는가", "현준·민서"],
        ], [10 * mm, 62 * mm, 50 * mm, 34 * mm], hi=[4]),
        Spacer(1, 3),
        P("<b>1~4단계는 배관 공사이고 위험이 없다.</b> zone_base_risk를 0으로 두면 "
          "지금과 똑같이 동작하기 때문이다. 실제로 값을 넣는 것은 5단계 검증 이후다.", "body"),
    ]

    # ── 6. 검증 ────────────────────────────────────────────────────
    s += [
        P("6. 어떻게 검증할 것인가", "h1"),
        P("두 가지 방법이 있고, 둘 다 하는 것이 좋다.", "body"),

        P("6.1 시뮬레이션 검증 — 구조가 옳은지", "h2"),
        P("scenario_sim.py에 구간 개념을 넣는다. 시나리오마다 \"위험 구간\" 표식을 "
          "부여하되, <b>표식이 있는 구간에서 실제로 위험이 더 자주 발생하도록</b> "
          "생성 파라미터를 다르게 준다(예: miss_offset 분포를 좁힌다). 그러면 "
          "zone 정보가 진짜 정보가 되고, 그것을 쓴 판정이 나아지는지 evaluate.py로 "
          "잴 수 있다.", "body"),
        P("이 실험은 <b>\"구간 정보가 있으면 도움이 되는가\"</b>라는 질문에 답한다. "
          "도움이 되지 않는다면 배관만 만들어두고 값은 넣지 않으면 된다.", "cap"),

        P("6.2 실측 검증 — 값이 옳은지", "h2"),
        P("실측 이벤트가 쌓이면 구간별 발생 빈도를 집계한다. 특정 구간에 이벤트가 "
          "몰린다면 그 구간에 가중치를 주는 것이 정당해진다. 몰리지 않는다면 "
          "zone_base_risk는 0으로 두는 것이 맞다.", "body"),
        P("빈도 → 점수 환산은 단순하게 시작한다. 예를 들어 상위 10% 구간에 5점, "
          "상위 30%에 3점, 나머지 0점. 복잡한 공식은 근거가 생긴 뒤에 만든다.", "cap"),
    ]

    s.append(PageBreak())

    # ── 7. 참고 ────────────────────────────────────────────────────
    s += [
        P("7. 착수에 필요한 정보", "h1"),
        T([
            ["항목", "내용"],
            ["위험지도 API",
             "https://riskmap-api-193571596396.asia-northeast3.run.app (Cloud Run, "
             "GCP 프로젝트 hanium-ssu-kpc). 팀원 관리 영역"],
            ["기존 업로드 경로",
             "deploy/upload_events.py — 레벨 1+ 이벤트를 POST /api/events, 1분 타이머. "
             "인증키는 /etc/default/v2x-riskmap"],
            ["집계 갱신",
             "POST /api/admin/refresh-stats (빈 바디 -d \"{}\" 필요, 없으면 411). "
             "지도는 집계 테이블 기준이라 이 호출 전에는 새 이벤트가 반영되지 않는다"],
            ["구간 ID 형식", "1381099007_3 계열"],
            ["점수표 상한", "zone_base_risk는 0~5로 클램프됨 (100점 중 5점)"],
            ["관련 파일",
             "step7_risk.py(계산), step8_send_risk.py(전달 지점), "
             "auto_pipeline/process_scenarios.py(시뮬레이션 쪽 동일 인자)"],
        ], [30 * mm, 126 * mm]),

        P("8. 이 작업을 8/9에 함께 논의해야 하는 이유", "h1"),
        P("8/9에 \"두 AI 모델을 어떻게 합칠 것인가\"를 결정하기로 되어 있는데, "
          "이 작업이 <b>같은 질문의 다른 얼굴</b>이다. 둘 다 \"시뮬레이션 쪽과 실시간 "
          "쪽을 어떻게 연결할 것인가\"를 묻고 있다.", "body"),
        T([
            ["질문", "무엇을 연결하는가"],
            ["두 모델 중 무엇을 쓸 것인가", "시뮬레이션에서 <b>학습한 것</b> → 실시간 판단"],
            ["zone_base_risk를 어떻게 채울 것인가", "시뮬레이션·실측에서 <b>관측한 것</b> → 실시간 판단"],
        ], [56 * mm, 100 * mm]),
        Spacer(1, 3),
        P("따로 결정하면 서로 어긋난 구조가 나올 수 있다. 한자리에서 정하는 것이 좋다.", "body"),

        P("9. 예상 작업량", "h1"),
        T([
            ["단계", "작업량"],
            ["1~4 (배관)", "반나절. 위험이 없고 기존 동작 불변"],
            ["6.1 시뮬레이션 검증", "반나절. scenario_sim에 구간 개념 추가 + 재측정"],
            ["6.2 실측 검증", "데이터 축적 후. 집계 스크립트 자체는 1시간"],
        ], [40 * mm, 116 * mm]),
        Spacer(1, 4),
        P("<b>정리하면</b> — 배관을 먼저 놓고(반나절), 구조가 옳은지 시뮬레이션으로 "
          "확인하고(반나절), 값은 실측이 쌓이는 대로 채운다. 급하지 않지만 "
          "<b>연결하지 않으면 시뮬레이션 파이프라인이 지도 그리는 용도로 끝난다.</b>", "box"),
    ]

    doc = SimpleDocTemplate(
        str(OUT), pagesize=A4,
        leftMargin=22 * mm, rightMargin=22 * mm,
        topMargin=18 * mm, bottomMargin=18 * mm,
        title="다음 작업 — 시뮬레이션과 실시간 연결",
    )

    def footer(canvas, d):
        canvas.saveState()
        canvas.setFont("Malgun", 8)
        canvas.setFillColor(colors.HexColor("#888"))
        canvas.drawCentredString(A4[0] / 2, 10 * mm, str(d.page))
        canvas.restoreState()

    doc.build(s, onFirstPage=footer, onLaterPages=footer)
    return OUT


if __name__ == "__main__":
    print("저장:", build())
