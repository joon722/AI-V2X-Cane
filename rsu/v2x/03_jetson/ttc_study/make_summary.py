#!/usr/bin/env python3
"""8/7 작업 정리 — 팀 누구나 읽을 수 있게.

make_findings.py가 "무엇을 확인했는가"를 근거와 함께 담는다면, 이 문서는
"오늘 무엇을 했고 그래서 뭐가 달라졌는가"를 짧게 답한다. 용어를 풀어 쓰고
표를 앞세운다.
"""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

HERE = Path(__file__).parent
OUT = HERE / "오늘한일_2026-08-07.pdf"

pdfmetrics.registerFont(TTFont("Malgun", "C:/Windows/Fonts/malgun.ttf"))
pdfmetrics.registerFont(TTFont("MalgunBd", "C:/Windows/Fonts/malgunbd.ttf"))

B = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("t", parent=B["Title"], fontName="MalgunBd", fontSize=18,
                            leading=24, spaceAfter=3),
    "sub": ParagraphStyle("s", parent=B["Normal"], fontName="Malgun", fontSize=10,
                          leading=14, alignment=TA_CENTER,
                          textColor=colors.HexColor("#555"), spaceAfter=16),
    "h1": ParagraphStyle("h1", parent=B["Heading1"], fontName="MalgunBd", fontSize=14,
                         leading=19, spaceBefore=15, spaceAfter=7,
                         textColor=colors.HexColor("#1f4e79")),
    "body": ParagraphStyle("b", parent=B["Normal"], fontName="Malgun", fontSize=10,
                           leading=16, alignment=TA_JUSTIFY, spaceAfter=7),
    "box": ParagraphStyle("bx", parent=B["Normal"], fontName="Malgun", fontSize=9.8,
                          leading=15, alignment=TA_JUSTIFY, leftIndent=9,
                          rightIndent=9, spaceAfter=7, borderPadding=8,
                          backColor=colors.HexColor("#f4f7fa")),
    "warn": ParagraphStyle("w", parent=B["Normal"], fontName="Malgun", fontSize=9.8,
                           leading=15, alignment=TA_JUSTIFY, leftIndent=9,
                           rightIndent=9, spaceAfter=7, borderPadding=8,
                           backColor=colors.HexColor("#fdeaea")),
    "cap": ParagraphStyle("cp", parent=B["Normal"], fontName="Malgun", fontSize=8.6,
                          leading=12.5, alignment=TA_CENTER,
                          textColor=colors.HexColor("#666"), spaceAfter=10),
}


def P(t, s="body"):
    return Paragraph(t, S[s])


def T(rows, widths, hi=None):
    data = [[Paragraph(f"<b>{c}</b>", ParagraphStyle(
        "th", fontName="MalgunBd", fontSize=9, leading=12,
        textColor=colors.white, alignment=TA_CENTER)) for c in rows[0]]]
    for r in rows[1:]:
        data.append([Paragraph(str(c), ParagraphStyle(
            "td", fontName="Malgun", fontSize=9, leading=13)) for c in r])
    st = [
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2f6ea5")),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#c8d4e0")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f7f9fb")]),
    ]
    for i in (hi or []):
        st.append(("BACKGROUND", (0, i + 1), (-1, i + 1), colors.HexColor("#fdeaea")))
    t = Table(data, colWidths=widths)
    t.setStyle(TableStyle(st))
    return t


W = 156 * mm


def figure(name, width_mm, caption):
    path = HERE / name
    img = Image(str(path))
    ratio = img.imageHeight / img.imageWidth
    img.drawWidth = width_mm * mm
    img.drawHeight = width_mm * mm * ratio
    img.hAlign = "CENTER"
    return [img, Spacer(1, 4), P(caption, "cap")]


def build():
    s = []

    s += [
        P("8월 7일에 한 일", "title"),
        P("팀 대화의 질문 하나에서 시작해 일곱 가지를 확인했다", "sub"),

        P("시작은 이 질문이었다 — <b>\"인도를 걷는데 옆 차도로 차가 지나가면 "
          "울리나?\"</b> 코드를 돌려보니 울렸고, 왜 그런지 따라가다 시스템 전반을 "
          "점검하게 되었다.", "body"),

        P("오늘 한 일을 한 줄로 말하면", "h1"),
        P("<b>거의 모두 '재보는 일'이었다. 코드는 세 줄 고쳤다.</b><br/>"
          "고치면 좋아 보이는 것을 셋 시도했는데 재보니 셋 다 손해였다. "
          "그래서 안 고치기로 했고, 그 판단에 근거가 생겼다.", "box"),

        T([
            ["한 일", "결과"],
            ["인도에서 울리는지 계산", "<b>7m 떨어져도 울린다</b>"],
            ["GPS 오차를 바꿔가며 성능 측정", "성능이 60%~88%로 갈린다"],
            ["점수표 항목 중복 검사", "다섯 중 셋이 같은 것을 재고 있었다"],
            ["AI 이득이 GPS에 따라 변하는지", "+18.9 → +9.5%p로 줄어든다"],
            ["위험지도 문제 진단", "초록이 없는 게 문제가 아니었다"],
            ["시뮬레이션 연결 상태 확인", "이미 다 뚫려 있다"],
            ["모델 파일 두 개 비교", "하나가 특정 상황에서 오작동"],
            ["젯슨에 뭐가 들어 있는지 확인", "안전 하한 정상, 모델은 결함 있는 쪽"],
        ], [72 * mm, W - 72 * mm]),
    ]

    s.append(PageBreak())

    s += [P("지금 시스템은 이렇게 생겼다", "h1")]
    s += figure("fig_overview.png", 152,
                "빨강은 계산으로 나온 값이라 AI가 못 뒤집는다. "
                "주황이 AI다. 아래 청록 점선이 아직 없는 연결이다.")

    s.append(PageBreak())

    s += [
        P("알아낸 것 일곱 가지", "h1"),

        P("<b>1. 인도를 걸을 때 울린다</b><br/>"
          "차가 뒤에서 다가오는 2초 동안 최고 경보가 나간다. 횡방향으로 7m "
          "떨어져 있어도 그렇다. 안전장치(TTC 2초 규칙)가 다른 판단을 전부 "
          "덮어쓰기 때문이다.", "body"),

        P("<b>2. GPS 정밀도가 성능을 좌우한다</b><br/>"
          "위치 오차에 따라 제때 울린 비율이 60%에서 88%까지 벌어졌다. AI가 "
          "올린 것이 +18.9%p였는데, GPS만 좋아져도 +15%p다.", "body"),

        P("<b>3. 놓침과 헛경보는 원인이 다르다</b> — 오늘 가장 크게 바뀐 생각", "body"),
        T([
            ["", "범인", "고치는 방법"],
            ["위험을 못 잡음", "<b>GPS</b>", "안테나 · 부품"],
            ["인도에서 울림", "<b>판정 구조</b>", "코드"],
        ], [46 * mm, 40 * mm, W - 86 * mm]),
        P("GPS를 아무리 고쳐도 헛경보는 안 줄었다(3.42% → 3.41%). 반대로 코드를 "
          "고쳐도 놓침은 안 줄었다. 지금까지 한 덩어리로 다뤘는데 별개였다.", "cap"),

        P("<b>4. AI 이득은 GPS에 따라 달라진다</b><br/>"
          "\"AI가 +18.9%p 낫다\"는 GPS 오차 2.5m 조건의 값이다. GPS가 좋아지면 "
          "+9.5%p로 준다. 발표에서 조건을 안 밝히면 질문 한 번에 무너진다. "
          "다만 뒤집어 보면 <b>모델이 싼 부품을 보완하고 있다</b>는 뜻이라, "
          "보급형 기기에는 오히려 강점이다.", "body"),

        P("<b>5. 위험지도는 '상충 지도'다</b><br/>"
          "동사무소가 보고 CCTV 위치를 정하는 지도라면, 차량용도 보행자용도 "
          "아니다. 진짜 문제는 초록이 없는 게 아니라 <b>259개가 거의 다 같은 "
          "등급이라 순위를 못 매기는 것</b>이다. 예산이 한정적인데 "
          "\"259곳이 위험합니다\"로는 어디부터 손댈지 정할 수 없다.", "body"),

        P("<b>6. 시뮬레이션 연결은 이미 다 되어 있다</b><br/>"
          "서버에서 젯슨으로, 젯슨에서 서버로 양방향이 1분마다 돌고 재시도까지 "
          "있다. <b>문제는 배관이 아니라 물이다</b> — SUMO 데이터에 위험 사례가 "
          "0건이라 흐를 내용이 비어 있다.", "body"),

        P("<b>7. 모델 파일이 두 개인데 하나가 오작동한다</b><br/>"
          "30m 밖에 정지한 차량에 경보를 낸다. 그런데 1,600개 시나리오로 채점하면 "
          "두 모델 성적이 거의 같다. 결함이 국소적이라 평균에 묻힌다. 다시 학습해도 "
          "결과가 같았고, 원인은 <b>학습 데이터에 '가까운데 둘 다 정지'라는 상황이 "
          "없어서</b>다.", "body"),
    ]

    s.append(PageBreak())

    s += [
        P("고치려다 안 고친 것 셋", "h1"),
        P("셋 다 \"이렇게 하면 좋아지겠다\" 싶었는데 재보니 손해였다. "
          "<b>재보지 않고 넣었으면 조용히 성능이 깎였을 것이다.</b>", "body"),

        T([
            ["시도", "기대", "실제"],
            ["안전장치를 조건부로",
             "인도 헛경보 감소",
             "위험 시나리오 <b>3.8개 더 놓침</b>"],
            ["점수표 항목 줄이기",
             "중복 제거, 설명 쉬워짐",
             "성능 같음. 판정도 <b>98.7% 동일</b> — 바꿀 이유 없음"],
            ["말도 안 되면 막는 장치",
             "30m 주차 차량 헛경보 차단",
             "제때 울린 비율 <b>2.6%p 하락</b>"],
        ], [40 * mm, 42 * mm, W - 82 * mm]),

        P("세 번째가 특히 아쉬웠다. 막으려던 상황이 전체의 <b>0.12%</b>인데, "
          "그것 때문에 <b>\"곧 출발할 차\"를 미리 알리는 기능이 죽었다.</b> "
          "물리 규칙은 '지금'만 보고 AI는 '곧 일어날 일'을 보는데, 규칙으로 AI를 "
          "막으면 그 차이만큼 잃는다.", "warn"),

        P("실제로 고친 것", "h1"),
        T([
            ["파일", "무엇을", "위험"],
            ["run_v2x_risk_engine.sh", "규칙만으로 돌리는 옵션 (3줄)", "없음 — 기본 동작 그대로"],
            ["test_model_gate.py", "실제 모델을 검사하는 테스트 2개", "없음"],
            ["EXPERIMENT_PLAN.md", "실험 절차 보완", "없음"],
        ], [50 * mm, 60 * mm, W - 110 * mm]),
        P("<b>step1~9 코드는 하나도 건드리지 않았다.</b> 테스트는 256개 통과 "
          "(이전 255개). 실패 1개는 위의 30m 문제를 \"알려진 한계\"로 표시해 둔 것이다.", "cap"),
    ]

    s.append(PageBreak())

    s += [
        P("내일 실험에서 할 일", "h1"),
        T([
            ["", "할 일", "얼마나", "왜"],
            ["0", "지금 무엇이 도는지 기록", "5분",
             "안 하면 이후 데이터가 어느 버전인지 모른다"],
            ["1", "<b>GPS 오차 측정</b>", "10분",
             "<b>가장 싸고 가장 결정적.</b> 이후 결정이 이 값에 달렸다"],
            ["2", "규칙만으로 한 바퀴", "—", "AI 없이 기준선을 잡는다"],
            ["3", "인도 나란히 걷기 (3/5/7m)", "40분", "몇 미터부터 조용해지는지"],
            ["4", "벽 뒤 전파 세기 측정", "30분",
             "\"운전자가 못 보는 곳\"을 감지할 수 있는지"],
        ], [8 * mm, 52 * mm, 18 * mm, W - 78 * mm]),
        Spacer(1, 5),
        P("1번이 왜 중요하냐면 — GPS 오차가 1.5m 이하로 나오면 헛경보를 줄이는 "
          "변경이 <b>공짜</b>가 되고, 2.5m면 손해, 4m면 아예 논외가 된다. "
          "값 하나로 이후 작업이 갈린다. 그런데 지금까지 한 번도 재본 적이 없다.", "box"),

        P("8월 9일에 정할 것", "h1"),
        T([
            ["항목", "정할 것"],
            ["안전장치 조건화", "GPS 실측값이 1.5m 이하면 채택, 아니면 보류"],
            ["지도", "상위 몇 개만 색칠하기 · 오래된 기록 버리기 · 통행량으로 나누기"],
            ["<b>SUMO 위험 생성</b>", "<b>보행자 양보 설정. 나머지가 전부 여기 달렸다</b>"],
            ["모델 파일", "하나로 정하고 어느 것인지 기록"],
            ["점수표", "바꾸지 않기로 이미 정했다 — 참고 사항"],
        ], [40 * mm, W - 40 * mm], hi=[2]),

        P("만든 문서와 그림", "h1"),
        T([
            ["파일", "내용"],
            ["오늘확인한것_2026-08-07.pdf", "일곱 가지 확인 사항과 근거 숫자"],
            ["ROADSIDE_ALARM.md", "계산 과정 전부와 재현 방법"],
            ["EXPERIMENT_PLAN.md", "내일 실험 절차 (0단계 · GPS 측정 · 차폐 추가)"],
            ["fig_overview 외 3장", "구조 그림 — PNG와 SVG 둘 다"],
        ], [58 * mm, W - 58 * mm]),

        Spacer(1, 6),
        P("오늘 세 번 \"그럴듯한 개선이 재보니 손해\"가 나왔다. 이것은 나쁜 신호가 "
          "아니라 <b>지금 구조가 생각보다 잘 잡혀 있다</b>는 뜻이다. 남은 문제는 "
          "구조가 아니라 <b>센서 정밀도와 데이터</b>에 있고, 둘 다 내일과 8/9에 "
          "답이 나온다.", "box"),
    ]
    return s


def _page_number(canvas, doc):
    canvas.saveState()
    canvas.setFont("Malgun", 8)
    canvas.setFillColor(colors.HexColor("#888"))
    canvas.drawCentredString(A4[0] / 2, 11 * mm, str(doc.page))
    canvas.restoreState()


def main():
    doc = SimpleDocTemplate(
        str(OUT), pagesize=A4,
        leftMargin=27 * mm, rightMargin=27 * mm,
        topMargin=20 * mm, bottomMargin=18 * mm,
        title="8월 7일에 한 일", author="V2X 팀",
    )
    doc.build(build(), onFirstPage=_page_number, onLaterPages=_page_number)
    print(f"저장: {OUT}")


if __name__ == "__main__":
    main()
