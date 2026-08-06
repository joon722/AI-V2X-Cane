#!/usr/bin/env python3
"""공모전 전략 문서를 PDF로 만든다.

인수인계 문서가 "무엇을 만들었고 어떻게 돌아가는가"를 다룬다면, 이 문서는
"그것을 어떻게 보여줄 것인가"만 다룬다. 기술 지표는 이미 경쟁권이고 남은 것은
전달이므로, 읽고 바로 실행할 수 있는 형태로 쓴다.
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
OUT = HERE / "공모전_전략_2026-08-06.pdf"

pdfmetrics.registerFont(TTFont("Malgun", "C:/Windows/Fonts/malgun.ttf"))
pdfmetrics.registerFont(TTFont("MalgunBd", "C:/Windows/Fonts/malgunbd.ttf"))

B = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("t", parent=B["Title"], fontName="MalgunBd", fontSize=18,
                            leading=24, spaceAfter=3),
    "sub": ParagraphStyle("s", parent=B["Normal"], fontName="Malgun", fontSize=10,
                          leading=14, alignment=TA_CENTER,
                          textColor=colors.HexColor("#555"), spaceAfter=14),
    "h1": ParagraphStyle("h1", parent=B["Heading1"], fontName="MalgunBd", fontSize=13,
                         leading=18, spaceBefore=14, spaceAfter=6,
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
    "quote": ParagraphStyle("q", parent=B["Normal"], fontName="MalgunBd", fontSize=11,
                            leading=17, alignment=TA_CENTER, leftIndent=12,
                            rightIndent=12, spaceBefore=6, spaceAfter=10,
                            textColor=colors.HexColor("#1f4e79"),
                            backColor=colors.HexColor("#eef4fa"), borderPadding=10),
    "cap": ParagraphStyle("c", parent=B["Normal"], fontName="Malgun", fontSize=8.4,
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
        P("공모전 전략 — 무엇이 부족하고 무엇을 보여줄 것인가", "title"),
        P("한이음 드림업 · 창의도전형 · 2026-08-06 기준", "sub"),
    ]

    s += [
        P("기술 지표는 이미 경쟁권이다. 남은 것은 전달이다. "
          "<b>지금부터는 코드를 더 짜는 것으로 상이 올라가지 않는다.</b>", "box"),
    ]

    # ── 1. 점수 ────────────────────────────────────────────────────
    s += [
        P("1. 평가항목별 자체 평가", "h1"),
        P("배점이 공개되지 않았으므로 각 항목 100점 환산으로 매겼다.", "cap"),
        T([
            ["항목", "세부", "점수", "근거"],
            ["기획력", "차별성", "90",
             "카메라→통신 패러다임 전환. <b>후방 위험의 75%가 TTC 무한대</b>라는 정량 근거 보유"],
            ["", "필요성", "85",
             "카메라가 실패하는 조건(가림·야간·급출현)에 통신은 무관. 단 당사자 검증 없음"],
            ["", "활용가능성", "85", "고령자·어린이·산업현장 확장, 단일 칩 하드웨어"],
            ["기술력", "기능구체성", "90", "ESP32×3 + 젯슨 + 서버 + 지도 전 구간 동작"],
            ["", "난이도", "95", "V2X 프로토콜 자작, 칼만 필터, AI, 실시간 임베디드"],
            ["", "완성도", "85", "테스트 344개·실기 가동. <b>실측 검증이 얇다</b>"],
            ["수행능력", "문서완성도", "92", "논문·설계문서·인수인계·실험계획·미팅기록"],
            ["", "문제해결능력", "95",
             "함정 7개를 발견→규명→수정→재측정까지 데이터로 기록"],
            ["", "수행충실성", "88", "4월부터 GitHub 이력 연속"],
        ], [18 * mm, 24 * mm, 13 * mm, 101 * mm], hi=[5]),
        Spacer(1, 3),
        P("<b>종합 약 90점.</b> 기술력·수행능력은 상위권으로 보인다. 특히 "
          "문제해결능력은 최상위일 가능성이 높다 — 대부분의 팀은 이 항목에 "
          "\"라이브러리 버전 충돌을 해결했다\" 수준을 쓴다.", "body"),
    ]

    # ── 2. 약점 ────────────────────────────────────────────────────
    s += [
        P("2. 치명적 약점 셋", "h1"),
        P("① <b>실증이 사실상 없다.</b> 실외 데이터가 8/5 로그 하나뿐이고, 그 안에서 "
          "2m 이내 접근 1회, AI가 반응한 구간 9스텝이다. 나머지는 전부 시뮬레이션이다. "
          "\"실제로 테스트하셨나요\"에 지금은 답이 궁색하다. <b>8/8 실험이 유일한 "
          "기회다.</b><br/><br/>"
          "② <b>당사자 검증이 없다.</b> 시각장애인을 위한 시스템인데 진동 세기가 "
          "적절한지, 인지되는지 확인되지 않았다. 안전 하한 2.0초의 '정지 1.2초' 항도 "
          "일반 보행자 문헌에서 빌린 값이다.<br/><br/>"
          "③ <b>하드웨어가 언제 죽을지 모른다.</b> 브리지 브라운아웃 이력, 차량 GPS "
          "51.3% 무효, 지팡이 IMU 28% 무효, 차량 AP 없으면 지팡이 무한 재부팅. "
          "<b>시연 중 하나만 터지면 끝이다.</b>", "warn"),

        P("2.1 심사에서 답하기 어려운 질문", "h2"),
        T([
            ["예상 질문", "준비해야 할 답"],
            ["규칙만으로 19/19 다 잡는데 AI가 왜 필요한가",
             "\"다 잡긴 잡는데 늦게 잡습니다. 반응할 수 있게 미리 울린 비율이 "
             "66.6%에서 85.5%로 올랐습니다.\" — 한 문장으로 끝낼 것"],
            ["TTC 무한대에서 울린 건 오경보 아닌가",
             "\"실제로 2m 안까지 접근한 경우만 정답으로 세었습니다. 규칙과 무관하게 "
             "궤적으로만 판정했습니다.\""],
            ["SUMO 놔두고 왜 직접 만들었나",
             "\"SUMO로 12,621스텝을 돌렸는데 최소거리가 9.59m였습니다. 위험이 0건이라 "
             "검증이 불가능했습니다.\" — 측정 결과이지 변명이 아님"],
            ["AI가 두 개인가",
             "8/9까지 하나로 정리할 것. 정리 전에는 답할 수 없다"],
        ], [50 * mm, 106 * mm]),
    ]

    s.append(PageBreak())

    # ── 3. 대안 ────────────────────────────────────────────────────
    s += [
        P("3. 당사자 검증의 대체 — 눈가리개 측정", "h1"),
        P("시각장애인 섭외가 현실적으로 어렵다. 그렇다면 <b>팀원이 시각을 차단하고 "
          "진동 인지 시간을 직접 재는 것</b>으로 대체한다. 완벽한 대체는 아니지만 "
          "문헌 대용값보다는 우리 시스템에 가깝다.", "body"),
        T([
            ["지금", "눈가리개 측정 후"],
            ["안전 하한의 정지 1.2초 = 일반 보행자 문헌 대용", "우리가 직접 측정한 값"],
            ["\"당사자 검증 없음\"", "\"시각을 차단하고 측정함\""],
            ["진동 세기가 적절한지 모름", "인지 여부·인지 시간 확인"],
        ], [78 * mm, 78 * mm]),
        Spacer(1, 3),
        P("<b>측정 대상은 하나다 — 진동이 온 순간부터 완전히 멈출 때까지의 시간.</b> "
          "안대와 스톱워치면 되고, 10회 반복에 20분이면 충분하다. 절차는 "
          "EXPERIMENT_PLAN.md에 넣어두었다.", "box"),
        Spacer(1, 2),
        P("발표에서의 효과가 더 크다. \"못 했다\"가 아니라 <b>\"이렇게 대신했다\"</b>가 "
          "되기 때문이다. 심사위원은 한계를 인정하고 대안을 찾은 팀을 좋게 본다.", "body"),

        P("4. 빠져 있는 무기 — 비용", "h1"),
        P("활용가능성 주장에 가격이 없다. 이것이 가장 아깝다.", "body"),
        T([
            ["", "비용", "한계"],
            ["카메라 기반 보행 보조", "수십만 원 + 연산 장치 + 배터리",
             "가려지면·어두우면·갑자기 나타나면 못 본다"],
            ["<b>V2X 노드 (ESP32)</b>", "<b>칩 하나, 부품가 몇천 원</b>",
             "상대도 노드를 달아야 한다"],
        ], [40 * mm, 56 * mm, 60 * mm], hi=[1]),
        Spacer(1, 3),
        P("카메라는 비싸고, 안 보이면 못 봅니다.<br/>저희는 싸고, 안 보여도 압니다.", "quote"),
        P("이 한 문장에 차별성·필요성·활용가능성이 모두 들어간다. 그리고 "
          "\"칩이 싸니까 고령자·어린이·산업현장으로 확장할 수 있다\"로 자연스럽게 "
          "이어진다.", "body"),
    ]

    s.append(PageBreak())

    # ── 5. 발표 구성 ───────────────────────────────────────────────
    s += [
        P("5. 발표 5분 구성안", "h1"),
        P("심사위원은 코드를 읽지 않는다. 테스트 344개도 논문 10쪽도 5분 안에 전달되지 "
          "않으면 0점과 같다.", "cap"),
        T([
            ["시간", "내용", "핵심"],
            ["0:00~0:30", "문제 제기",
             "\"기존 시각장애인 보조기기는 대부분 카메라입니다. 그런데 카메라는 "
             "가려지면, 어두우면, 갑자기 나타나면 못 봅니다.\""],
            ["0:30~1:30", "우리 접근",
             "V2X 통신. 시야와 무관하게 위치를 주고받는다. 실물 보여주기 "
             "(지팡이·차량·RSU)"],
            ["1:30~2:30", "<b>발견</b>",
             "\"저희가 측정해보니 <b>후방 위험의 75%</b>는 TTC라는 기존 지표로 "
             "잡히지 않았습니다. 임계값을 몇 초로 두든 안 잡힙니다.\""],
            ["2:30~3:30", "<b>시연 영상</b>",
             "TTC가 무한대인데 지팡이가 진동하는 5초. <b>이 장면이 발표 전체보다 강하다</b>"],
            ["3:30~4:30", "결과",
             "적시 경보 66.6% → 85.5% (같은 오경보). 안전 하한 2.0초는 AI와 무관하게 발동"],
            ["4:30~5:00", "확장",
             "칩 하나, 몇천 원. 고령자·어린이·산업현장"],
        ], [22 * mm, 26 * mm, 108 * mm], hi=[2, 3]),
        Spacer(1, 3),
        P("<b>기억시킬 것은 숫자 하나와 장면 하나뿐이다.</b> 숫자는 \"후방 75%\", "
          "장면은 \"TTC 무한대인데 울리는 지팡이\". 나머지는 질문받으면 답하면 된다.", "box"),

        P("6. 상 사다리", "h1"),
        T([
            ["단계", "필요 조건", "왜"],
            ["동상", "지금 상태", "기술은 되는데 실증이 없다"],
            ["<b>은상</b>", "+ 8/8 실험 성공 (고속 위험 30건 + 측면 시연 영상)",
             "\"실제로 된다\"가 증명된다"],
            ["<b>금상</b>", "+ 눈가리개 측정 + AI 단일화 + 비용·확장 서사",
             "한계를 대안으로 메운 팀으로 보인다"],
            ["대상", "+ 완성품처럼 보이는 실물 + 실도로 영상 + 10월까지 추가 성과",
             "\"실험실 프로젝트\"가 아니라 \"제품\"으로 보인다"],
        ], [18 * mm, 74 * mm, 64 * mm], hi=[1, 2]),
        Spacer(1, 3),
        P("창의도전형 은상 이상은 대상 1 + 금상 3 + 은상 10 = <b>상위 14팀</b>이다. "
          "\"최소 은상\"은 8/8 실험이 성공해야 성립한다.", "cap"),

        P("7. 지금 해야 할 것 — 셋만", "h1"),
        T([
            ["", "할 일", "언제", "효과"],
            ["1", "<b>8/8 실험 성공.</b> 브리지 전원 먼저 확인(전원공급형 허브), "
                  "차량 먼저 켜기, 측면 통과 10회 사수, <b>영상 촬영</b>",
             "8/8 (토)", "동상 → 은상"],
            ["2", "<b>눈가리개 진동 인지 측정 10회.</b> 안대와 스톱워치, 20분",
             "8/8 같은 날", "필요성 항목 +"],
            ["3", "<b>AI를 하나로 정리.</b> 두 모델이 공존하는 채로는 발표할 수 없다",
             "8/9", "기술력 +"],
        ], [10 * mm, 84 * mm, 24 * mm, 38 * mm], hi=[0, 1]),
        Spacer(1, 3),
        P("실험이 실패하면 다시 잡을 주말은 8/15뿐이다. <b>브리지 전원부터 확인할 것.</b> "
          "8/6 젯슨 로그에서도 브리지 대기 55초가 관측됐다.", "warn"),
        P("10월 30일까지 3개월이 남아 있다. 8/17 통합테스트 이후가 비어 있으므로, "
          "실물 완성도(케이스·배선)와 실도로 영상, 어르신 시나리오 한 번을 그 기간에 "
          "채우면 대상권 조건이 갖춰진다.", "body"),
    ]

    doc = SimpleDocTemplate(
        str(OUT), pagesize=A4,
        leftMargin=22 * mm, rightMargin=22 * mm,
        topMargin=18 * mm, bottomMargin=18 * mm,
        title="공모전 전략 2026-08-06",
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
