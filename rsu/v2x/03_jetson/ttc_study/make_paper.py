#!/usr/bin/env python3
"""연구 결과를 논문 형식 PDF로 만든다.

reportlab 기본 폰트에는 한글 글리프가 없으므로 Malgun Gothic을 등록해서 쓴다.
수치는 evaluate.py / alarm_profile.py / field_check.py 출력을 그대로 옮긴 것이고,
재현 명령은 부록에 적어 둔다.
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
OUT = HERE / "TTC_연구_논문.pdf"

pdfmetrics.registerFont(TTFont("Malgun", "C:/Windows/Fonts/malgun.ttf"))
pdfmetrics.registerFont(TTFont("MalgunBd", "C:/Windows/Fonts/malgunbd.ttf"))

BASE = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("t", parent=BASE["Title"], fontName="MalgunBd",
                            fontSize=19, leading=25, spaceAfter=4),
    "subtitle": ParagraphStyle("st", parent=BASE["Normal"], fontName="Malgun",
                               fontSize=10.5, leading=15, alignment=TA_CENTER,
                               textColor=colors.HexColor("#555555"), spaceAfter=14),
    "h1": ParagraphStyle("h1", parent=BASE["Heading1"], fontName="MalgunBd",
                         fontSize=13.5, leading=19, spaceBefore=15, spaceAfter=7,
                         textColor=colors.HexColor("#1f4e79")),
    "h2": ParagraphStyle("h2", parent=BASE["Heading2"], fontName="MalgunBd",
                         fontSize=11.5, leading=16, spaceBefore=10, spaceAfter=5,
                         textColor=colors.HexColor("#2f6ea5")),
    "body": ParagraphStyle("b", parent=BASE["Normal"], fontName="Malgun",
                           fontSize=9.8, leading=15.5, alignment=TA_JUSTIFY,
                           spaceAfter=7),
    "abstract": ParagraphStyle("ab", parent=BASE["Normal"], fontName="Malgun",
                               fontSize=9.4, leading=14.5, alignment=TA_JUSTIFY,
                               leftIndent=10, rightIndent=10, spaceAfter=5,
                               backColor=colors.HexColor("#f4f7fa"),
                               borderPadding=9),
    "caption": ParagraphStyle("c", parent=BASE["Normal"], fontName="Malgun",
                              fontSize=8.6, leading=12.5, alignment=TA_CENTER,
                              textColor=colors.HexColor("#555555"), spaceBefore=4,
                              spaceAfter=11),
    "quote": ParagraphStyle("q", parent=BASE["Normal"], fontName="MalgunBd",
                            fontSize=10.2, leading=16, leftIndent=16, rightIndent=16,
                            spaceBefore=6, spaceAfter=10,
                            textColor=colors.HexColor("#1f4e79")),
    "ref": ParagraphStyle("r", parent=BASE["Normal"], fontName="Malgun",
                          fontSize=8.6, leading=13, leftIndent=16, firstLineIndent=-16,
                          spaceAfter=4),
    "code": ParagraphStyle("cd", parent=BASE["Normal"], fontName="Courier",
                           fontSize=8.2, leading=11.5, leftIndent=12,
                           backColor=colors.HexColor("#f6f6f6"), borderPadding=7,
                           spaceBefore=4, spaceAfter=9),
}


def P(text, style="body"):
    return Paragraph(text, S[style])


def table(rows, widths, highlight=None, align=None):
    """헤더 1행 + 본문. highlight는 강조할 본문 행 인덱스(0-based, 헤더 제외)."""
    data = [[Paragraph(f"<b>{c}</b>", ParagraphStyle(
        "th", fontName="MalgunBd", fontSize=8.8, leading=12,
        textColor=colors.white, alignment=TA_CENTER)) for c in rows[0]]]
    for r in rows[1:]:
        data.append([Paragraph(str(c), ParagraphStyle(
            "td", fontName="Malgun", fontSize=8.8, leading=12.5,
            alignment=TA_CENTER)) for c in r])

    style = [
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#2f6ea5")),
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#c8d4e0")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 4.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4.5),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1),
         [colors.white, colors.HexColor("#f7f9fb")]),
    ]
    for i in (highlight or []):
        style.append(("BACKGROUND", (0, i + 1), (-1, i + 1), colors.HexColor("#fdeaea")))
    t = Table(data, colWidths=widths, hAlign=align or "CENTER")
    t.setStyle(TableStyle(style))
    return t


def figure(name, caption, width=155 * mm):
    path = HERE / name
    from PIL import Image as PILImage
    with PILImage.open(path) as im:
        w, h = im.size
    img = Image(str(path), width=width, height=width * h / w)
    return [img, P(caption, "caption")]


def build():
    story = []

    # ── 표제 ────────────────────────────────────────────────────────────
    story += [
        P("최적의 TTC는 존재하지 않는다", "title"),
        P("시각장애인 보행 지원 V2X 시스템에서 충돌 예상 시간 임계값의 한계와<br/>"
          "상황 조건부 위험 판단의 실증적 비교", "subtitle"),
    ]

    story += [
        P("<b>초록</b> — 차량-보행자 충돌 위험 판단에 널리 쓰이는 충돌 예상 시간"
          "(TTC, Time-To-Collision) 임계값 방식의 한계를 실증적으로 규명하고, 상황 "
          "조건부 판단 모델과 비교했다. 핵심 난점은 정답의 정의에 있었다. 기존 시스템은 "
          "규칙 기반 점수표로 학습 라벨을 생성했기 때문에, 어떤 모델을 학습시켜도 그 "
          "규칙을 복제할 뿐 규칙 자체의 타당성은 검증할 수 없는 순환 구조에 있었다. "
          "본 연구는 시뮬레이션의 미래 정보만을 사용하는 규칙 독립 정답(오라클 라벨)을 "
          "설계하고, 이를 기준으로 세 방법을 동일한 오경보 예산에서 비교했다. "
          "시나리오 1,200개 × 5회 반복 실험에서 조건부 모델은 현행 점수표 대비 적시 "
          "경보율을 66.6%에서 83.4%로 개선했다(+16.9%p, 대응 t-검정 t=3.15, p=0.034). "
          "더 중요한 발견은 모델이 탐지한 위험의 34%가 TTC 값이 무한대인 상태에서 "
          "발생했다는 점이다. 측면 접근 53%, 후방 접근 75%가 이에 해당하며, 이는 "
          "임계값을 어떤 값으로 설정하더라도 원리적으로 탐지 불가능한 위험이 존재함을 "
          "의미한다. 따라서 최적화 대상은 임계값이 아니라 판단 구조여야 한다. "
          "본 연구는 물리적 근거에 기반한 안전 하한 2.0초를 함께 제시하고 실기에 "
          "반영했다.", "abstract"),
        Spacer(1, 4),
        P("<b>주제어</b> — TTC, 충돌 위험 예측, 보행자 안전, V2X, 라벨 누수, "
          "안전 하한, 시각장애인 보행 지원", "caption"),
    ]

    # ── 1. 서론 ─────────────────────────────────────────────────────────
    story += [
        P("1. 서론", "h1"),
        P("충돌 예상 시간(TTC)은 차량-보행자 충돌 위험을 나타내는 대표적 지표로, "
          "두 물체 간 거리를 접근 속도로 나눈 값이다. 국제 표준과 안전 평가 규격은 "
          "TTC에 기반한 경보 시점을 정의하고 있으며[1][2], 다수의 상용 전방 충돌 "
          "경보 시스템이 이 방식을 채택한다. 본 연구가 대상으로 하는 시각장애인 "
          "보행 지원 시스템 역시 TTC를 포함한 다변수 점수표로 위험 수준을 산출한다.", "body"),
        P("연구의 출발점은 단순한 질문이었다. <b>우리 시스템의 최적 TTC 임계값은 "
          "몇 초인가?</b> 그러나 이 질문에 답하려는 과정에서 세 가지 문제가 순차적으로 "
          "드러났으며, 결과적으로 질문 자체가 잘못 설정되었음이 밝혀졌다.", "body"),
        P("첫째, 기존 학습 데이터의 정답이 검증 대상인 규칙으로부터 생성되어 있었다"
          "(2절). 둘째, 평가 지표가 경보의 시의성을 반영하지 못했다(3.4절). 셋째, "
          "그리고 가장 근본적으로, TTC라는 척도 자체로는 표현할 수 없는 위험이 상당 "
          "비율 존재했다(4.2절).", "body"),
        P("본 논문의 기여는 다음과 같다. (1) 규칙과 독립된 정답을 시뮬레이션의 미래 "
          "정보로 구성하는 방법을 제시하고, 교통공학의 표준 상충 지표와 교차검증했다. "
          "(2) 동일 오경보 예산 조건에서 규칙과 모델을 비교하는 평가 프로토콜을 "
          "설계했다. (3) TTC 임계값 방식이 원리적으로 탐지할 수 없는 위험의 존재와 "
          "그 비율을 정량화했다. (4) 학습으로 결정하지 않고 물리로 결정하는 안전 하한을 "
          "제시하고 실기에 반영했다.", "body"),
    ]

    # ── 2. 문제 진단 ────────────────────────────────────────────────────
    story += [
        P("2. 문제 진단: 정답이 규칙에서 나오는 순환 구조", "h1"),
        P("기존 시스템의 학습 데이터 생성 과정을 분석한 결과, 라벨이 다음과 같이 "
          "구성되어 있었다.", "body"),
        P("정답 = classify_risk_level( 점수표(거리, TTC, 상대속도, 차량속도) &times; DCPA 게이트 )",
          "code"),
        P("즉 검증하려는 규칙이 정답을 생성하고, 그 정답으로 모델을 학습시키는 구조다. "
          "이 경우 모델의 예측 정확도가 아무리 높아도 그것은 규칙을 얼마나 잘 모사했는지를 "
          "의미할 뿐, 규칙의 타당성에 대해서는 아무런 정보를 제공하지 않는다. 모델에게 "
          "\"TTC 2초라는 기준이 타당한가\"를 물을 수 없는 것이다.", "body"),
        P("추가로 입력 특징 집합에 규칙 점수(risk_score)가 포함되어 있었다. 라벨이 그 "
          "점수로부터 결정되므로 이는 명백한 라벨 누수(label leakage)이며, 보고된 정확도 "
          "수치의 해석을 무의미하게 만든다. 또한 비접근 상황의 TTC를 9999라는 관례값으로 "
          "채운 결과, 정규화 통계에서 TTC 평균이 6198로 산출되어 해당 특징이 사실상 "
          "무력화되어 있었다.", "body"),
        P("주목할 점은 이 시스템이 v1에서 v3까지 세 차례 개선되는 동안 모델 구조는 "
          "지속적으로 정교해졌으나(다변수 벡터 특징 추가, 3초 선행 예측 도입) "
          "<b>정답의 정의는 한 번도 변경되지 않았다</b>는 것이다. 모델의 개선이 규칙 "
          "모사 정확도의 개선으로만 귀결된 셈이다.", "body"),
    ]

    # ── 3. 방법 ─────────────────────────────────────────────────────────
    story += [
        P("3. 방법", "h1"),
        P("3.1 규칙 독립 정답(오라클 라벨)", "h2"),
        P("시뮬레이션 환경은 미래 궤적을 완전히 관측할 수 있다는 특성을 갖는다. 이를 "
          "이용해 사후적으로 \"해당 시점에 경보했어야 하는가\"를 판정한다.", "body"),
        P("d_min(t) = min{ 거리(&tau;) : &tau; &isin; (t, t+H] }<br/>"
          "y(t) = 1 if d_min(t) &le; d_crit else 0", "code"),
        P("이 정의에는 TTC도, 점수표도, DCPA 게이트도 개입하지 않는다. 좌표 기하학과 "
          "미래 정보만을 사용하므로 규칙의 타당성을 이 정답에 대해 질의할 수 있다. "
          "구현에서는 정답 생성 모듈이 규칙 모듈을 참조하지 않음을 추상 구문 트리(AST) "
          "검사로 강제하여 순환을 구조적으로 차단했다.", "body"),
        P("접촉 기준 거리 d_crit은 2.0m로 설정했다(차폭 절반 약 0.9m + 보행자 신체폭 "
          "+ 지팡이 궤적). 이 값의 민감도는 1.5m와 2.5m에서 확인했으며 결론은 유지되었다.", "body"),
        P("<b>교차검증.</b> 정의가 특정 수식에 의존하지 않음을 확인하기 위해 교통공학의 "
          "표준 상충 지표인 PET(Post-Encroachment Time)[3]를 병행 계산했다. 두 방법의 "
          "판정은 96.7% 일치했으며, 불일치 사례는 모두 PET가 더 넓게 판정하는 방향이었다. "
          "즉 본 연구의 정답은 PET의 부분집합으로, 보수적인 쪽에 위치한다.", "body"),

        P("3.2 예측 창 길이의 결정", "h2"),
        P("예측 창 H는 세 조건의 교집합으로 결정된다. 첫째, 반응 시간 하한(3.3절)이 "
          "2.0초이므로 창이 2초이면 경보 후 대응 시간이 0이 된다. 둘째, 창이 길수록 "
          "예측 자체가 불가능해진다. 셋째, 비교 대상인 물리 외삽이 3초 창을 사용한다. "
          "실측한 창 길이별 판별 성능은 그림 2와 같으며, 최종적으로 3초를 채택했다.", "body"),

        P("3.3 안전 하한의 물리적 산출", "h2"),
        P("안전 하한 T_floor는 학습으로 결정하지 않는다. 모델은 학습 분포 밖에서 "
          "예고 없이 실패할 수 있으므로, 자율주행 분야의 안전 감시자 개념[4]과 같이 "
          "물리적으로 결정된 하한을 모델 아래에 배치한다.", "body"),
        table([
            ["구성 요소", "값", "근거"],
            ["GPS 갱신 주기", "0.2 s", "수신기 5Hz 설정"],
            ["전송 지연", "0.1 s", "실측: 위험도 변경 29건 전수 (연산 1.0ms + 무선 왕복 102ms)"],
            ["인지 · 판단", "0.3 s", "촉각 단순 반응 0.12~0.18s[5]의 1.5~2배 (선택 반응)"],
            ["정지 동작", "1.2 s", "보행 급정지 제동시간 0.84~1.21s[6]의 보수적 끝"],
            ["안전 여유", "0.2 s", "—"],
            ["합계 T_floor", "2.0 s", "GB/T 33577 최소 2초[2], NHTSA NCAP FCW 2.0~2.4초와 일치"],
        ], [32 * mm, 18 * mm, 105 * mm], highlight=[5]),
        Spacer(1, 3),
        P("세 계열의 독립적 근거 — 자체 시스템 실측, 인간 반응 문헌, 국제 규격 — 이 "
          "동일한 값을 지시한다는 점이 이 수치의 신뢰성을 뒷받침한다.", "body"),
    ]

    story.append(PageBreak())

    story += [
        P("3.4 평가 지표: 적시 경보율", "h2"),
        P("경보 시스템의 평가에는 재현율만으로 불충분하다. 위험 발생 직전에만 경보해도 "
          "재현율은 상승하지만, 그 경보는 대응이 불가능하므로 실효성이 없다. 본 연구는 "
          "시나리오 단위로 다음을 측정했다.", "body"),
        P("적시 경보 = (실제 접촉 시각 &minus; 최초 경보 시각) &ge; T_floor", "code"),
        P("접촉 시각은 라벨 창과 무관하게 실제 궤적에서 산출하여, 창 설정 변경이 지표를 "
          "자동으로 개선시키는 것을 방지했다.", "body"),
        P("<b>비교의 공정성.</b> 각 방법이 임의의 동작점에서 비교되면 어느 쪽이든 "
          "우위를 주장할 수 있다. 이를 방지하기 위해 모델의 판정 임계값을 현행 점수표의 "
          "오경보율에 정확히 일치시킨 뒤 적시 경보율을 비교했다.", "body"),

        P("3.5 데이터 생성", "h2"),
        P("교통 시뮬레이터(SUMO)로 생성된 기존 데이터 12,621행을 분석한 결과, 차량-보행자 "
          "최소 거리가 9.59m로 위험 사례가 전무했다. 원인은 차량 유형 정의에서 보행자 "
          "양보 관련 파라미터(jmIgnoreFoeProb, jmDriveAfterRedTime, impatience)가 "
          "설정되지 않아, 차량이 기본 로직에 따라 보행자에게 항상 양보했기 때문이다. "
          "임계값 검증에는 위험이 실현된 궤적이 필수적이므로, 2체 운동학 기반 시뮬레이터를 "
          "직접 구현했다.", "body"),
        table([
            ["항목", "기존 SUMO 데이터", "본 연구 시뮬레이터"],
            ["보행자", "1명, 고정 경로", "속도·방향·가감속·선회 무작위"],
            ["차량", "63대 동시, 속도 중앙값 0.0", "1대, 정지 상태 16% 포함"],
            ["최소 거리", "9.59 m (위험 0건)", "0.20 m (위험 6.5%)"],
            ["시간 해상도", "1.0 초", "0.1 초"],
        ], [30 * mm, 60 * mm, 65 * mm], highlight=[2]),
        Spacer(1, 3),
        P("<b>관측 오차 모델.</b> 정답은 오차 없는 참값 궤적에서 생성하고, 모델이 관측하는 "
          "특징은 GPS 오차(1차 Gauss-Markov 과정, &sigma;=2.5m, 야외 실측 근거)가 부가된 "
          "궤적에서 산출했다. 이는 실제 시스템이 처한 조건 — 흔들리는 관측으로부터 실제 "
          "위험을 판정해야 하는 상황 — 을 재현하기 위함이다.", "body"),

        P("3.6 물리 외삽의 한계", "h2"),
        P("등속 가정 하의 3초 외삽 오차를 측정한 결과는 다음과 같다.", "body"),
        table([
            ["조건", "예측 오차 (중앙값)"],
            ["오차 없는 좌표", "0.00 m"],
            ["GPS 오차 + 단순 미분", "3.22 m"],
            ["GPS 오차 + 칼만 필터 (실제 파이프라인)", "2.92 m"],
        ], [95 * mm, 45 * mm], highlight=[2]),
        Spacer(1, 3),
        P("오차 없는 좌표에서는 물리 계산만으로 충분하다(오차 0). 그러나 실제 관측 조건에서 "
          "오차는 2.92m로, 위험 판정 기준인 2.0m를 상회한다. 칼만 필터가 이를 크게 개선하지 "
          "못하는 이유는 필터 역시 등속 모델에 기반하여 외삽과 동일한 약점을 공유하기 "
          "때문이다. 이 간극이 학습 모델이 기여할 수 있는 영역이다.", "body"),
    ]

    story.append(PageBreak())

    # ── 4. 결과 ─────────────────────────────────────────────────────────
    story += [
        P("4. 실험 결과", "h1"),
        P("4.1 세 방법의 비교", "h2"),
        P("시나리오 1,200개를 생성하고 시나리오 단위 층화 분할로 학습/평가를 구성했다. "
          "5개 시드로 반복한 결과는 다음과 같다.", "body"),
        table([
            ["방법", "적시 경보율", "오경보율"],
            ["A. 현행 점수표", "66.6% ± 5.2", "1.99%"],
            ["B. TTC &le; 2.0s", "30.8% ± 8.4", "2.15%"],
            ["B. TTC &le; 2.5s", "70.0% ± 10.9", "3.20%"],
            ["B. TTC &le; 3.0s", "82.7% ± 6.7", "4.29%"],
            ["C. 조건부 모델", "<b>83.4% ± 8.3</b>", "<b>1.99%</b>"],
        ], [58 * mm, 45 * mm, 37 * mm], highlight=[4]),
        Spacer(1, 4),
    ]
    story += figure("fig_tradeoff.png",
                    "그림 1. 오경보율 대비 적시 경보율. 회색은 TTC 단독 규칙의 동작점이며 "
                    "임계값을 높이면 적시성이 개선되나 오경보가 비례하여 증가한다. "
                    "조건부 모델은 현행 점수표와 동일한 오경보(1.99%)에서 "
                    "TTC 3.0초 규칙에 준하는 적시성을 달성한다.", 130 * mm)
    story += [
        P("모델은 현행 점수표 대비 적시 경보율을 16.9%p 개선했다(대응 t-검정 t=3.15, "
          "p=0.034, 5개 시드 중 4개에서 우위). TTC 3.0초 규칙은 유사한 적시성을 보이나 "
          "오경보를 2.2배 사용한다.", "body"),
        P("<b>학습 라벨 설계의 영향.</b> 초기 실험에서 모델의 개선폭은 +0.7%p(통계적 "
          "유의성 없음)에 그쳤다. 원인은 라벨이 \"3초 이내 위험\"으로 정의되어 0.5초 후 "
          "접촉하는 시점도 정답에 포함된 데 있었다. 대응이 불가능한 시점의 경보를 정답으로 "
          "학습시킨 것이다. 라벨 창을 T_floor만큼 지연시켜 [2초, 3초] 구간으로 재정의하자 "
          "개선폭이 +16.9%p로 증가했다.", "body"),
        P("주목할 점은 이 과정에서 <b>AUC가 0.970에서 0.848로 하락</b>했다는 것이다. "
          "AUC는 시점별 판별 순위를 측정하는 반면 본 연구의 목적은 최초 경보의 시의성이다. "
          "라벨 수정 전 모델은 판별은 정확하나 경보가 늦은 상태였다. 지표 선택이 결론을 "
          "역전시킬 수 있음을 보여주는 사례다.", "body"),
    ]

    story.append(PageBreak())

    story += [
        P("4.2 TTC로 표현할 수 없는 위험", "h2"),
        P("모델이 탐지한 위험을 접근 기하학과 속도로 분류하고, 최초 경보 시점의 TTC 값을 "
          "조사했다.", "body"),
        table([
            ["상황", "사례 수", "TTC 무한대 비율", "경보 여유(중앙값)"],
            ["측면 · 고속", "15", "<b>53%</b>", "3.10 s"],
            ["후방 · 고속", "4", "<b>75%</b>", "2.65 s"],
            ["정면 · 저속", "3", "33%", "4.80 s"],
            ["측면 · 저속", "6", "0%", "3.95 s"],
            ["정면 · 고속", "5", "0%", "3.50 s"],
            ["정지 차량", "2", "0%", "2.30 s"],
        ], [38 * mm, 25 * mm, 38 * mm, 39 * mm], highlight=[0, 1]),
        Spacer(1, 4),
        P("전체의 <b>34%</b>가 TTC 값이 상한(무한대)에 도달한 상태에서 경보되었다. "
          "측면으로 통과하는 차량은 시선 방향 접근 속도가 0에 근접하여 TTC가 무한대로 "
          "산출되지만, 실제로는 보행자의 진행 경로를 곧 횡단한다. <b>이러한 위험은 "
          "임계값을 어떤 값으로 설정하더라도 탐지되지 않는다.</b> 척도 자체가 해당 "
          "위험을 표현하지 못하기 때문이다.", "body"),
        P("경보 여유시간 역시 상황에 따라 2.30초에서 4.80초까지 2배 이상 차이를 보였다. "
          "단일 임계값으로는 재현할 수 없는 분포다.", "body"),
    ]
    story += figure("alarm_ttc_by_situation.png",
                    "그림 2. (좌) 상황별 TTC 탐지 가능 여부. 적색은 TTC가 무한대인 상태에서 "
                    "경보된 비율이다. (우) 경보 여유시간 분포. 파선은 안전 하한 2.0초로, "
                    "모델 판단과 무관하게 발동한다.", 165 * mm)

    story.append(PageBreak())

    story += [
        P("4.3 판단 근거 분석", "h2"),
        P("치환 중요도(permutation importance)로 모델의 판단 근거를 분석했다.", "body"),
        table([
            ["특징", "중요도", "특징", "중요도"],
            ["접근 속도 (closing_los)", "<b>0.213</b>", "상대 위치 x", "0.046"],
            ["상대 위치 y", "0.069", "최근접 예상거리", "0.046"],
            ["차량 속도", "0.057", "상대 가속", "0.039"],
            ["<b>TTC</b>", "<b>0.053</b>", "선회율", "0.022"],
        ], [45 * mm, 25 * mm, 45 * mm, 25 * mm], highlight=[3]),
        Spacer(1, 3),
        P("TTC는 네 번째 순위이며, 접근 속도의 중요도가 4배 높다. 이는 TTC 임계값 최적화가 "
          "실패한 이유를 설명한다. TTC는 여러 신호 중 하나이며, 이를 단독 기준으로 삼으면 "
          "나머지 정보가 손실된다. 또한 본 연구에서 추가한 상대 가속과 선회율이 실제로 "
          "활용되고 있으며, 이는 등속 가정을 벗어난 상황을 판별하는 근거로 기능한다.", "body"),

        P("4.4 현행 점수표의 구조 검증", "h2"),
        P("점수표 입력 간 상관을 분석한 결과 거리와 최근접 예상거리(DCPA)의 상관계수가 "
          "0.99로 나타났다. 정보 중복이 의심되어 배점 구조를 변경한 실험을 수행했다"
          "(오경보율 1.83%로 통제).", "body"),
        table([
            ["배점 구성", "적시 경보율"],
            ["현행 점수표 (거리30/TTC35/상대속도20/차량속도10 + DCPA 게이트)", "<b>55.6%</b>"],
            ["DCPA 게이트만 제거", "22.2%"],
            ["모델 중요도 기반 재배점 (33/11/44/12)", "33.3%"],
            ["접근 속도 단독 (0/0/100/0)", "22.2%"],
        ], [105 * mm, 35 * mm], highlight=[0]),
        Spacer(1, 3),
        P("두 가지 결론이 도출된다. 첫째, <b>모델 중요도를 선형 배점으로 이전하면 성능이 "
          "저하된다</b>(55.6% → 33.3%). 중요도는 비선형 결합에서의 기여도이지 선형 가중치가 "
          "아니다. 둘째, <b>DCPA 게이트는 점수표의 핵심 구성요소</b>다. 제거 시 성능이 "
          "절반 이하로 감소한다.", "body"),
        P("상관계수 0.99에도 불구하고 DCPA가 필수적인 이유는, 상관계수가 전체 분포의 평균적 "
          "관계만을 측정하기 때문이다. 차량이 원거리에 있을 때 두 값은 함께 변동하나, "
          "판단이 실제로 요구되는 근거리 접근 구간에서 분기한다. 현행 점수표는 잘 설계되어 "
          "있으며, 모델의 우위는 가중치가 아니라 비선형 결합에서 비롯된다.", "body"),
    ]

    story.append(PageBreak())

    story += [
        P("4.5 예측 창 길이의 영향", "h2"),
    ]
    story += figure("fig_horizon.png",
                    "그림 3. 예측 창 길이별 판별 성능(AUC). 10초 창에서는 무작위 수준으로 "
                    "저하된다. 현재 관측 정보에 10초 후의 차량 거동이 포함되어 있지 않기 "
                    "때문이다. 2초 창은 판별이 용이하나 안전 하한과 동일하여 대응 시간이 "
                    "확보되지 않는다.", 130 * mm)

    # ── 5. 실기 반영 ────────────────────────────────────────────────────
    story += [
        P("5. 실기 반영", "h1"),
        P("모델의 시뮬레이션 성능이 우수함에도 실제 경보 경로에는 반영하지 않았다. "
          "실외 실측 데이터에서 모델이 위험을 판정한 구간이 9스텝에 불과하여, 학습 분포 "
          "외부에서의 거동을 검증할 수 없었기 때문이다. 대신 물리적 근거가 확실한 안전 "
          "하한만을 규칙으로 반영했다.", "body"),
        table([
            ["구성", "적시 경보율", "오경보율", "시나리오 검출"],
            ["안전 하한 미적용", "63.2%", "2.03%", "18/19"],
            ["안전 하한 TTC &le; 2.0s", "<b>68.4%</b>", "3.24%", "<b>19/19</b>"],
            ["하한 + DCPA &le; 7.5m 조건", "68.4%", "2.49%", "18/19"],
        ], [50 * mm, 32 * mm, 30 * mm, 33 * mm], highlight=[1]),
        Spacer(1, 3),
        P("DCPA 조건을 부가하면 오경보는 감소하나 미탐지가 재발생한다. 안전 하한의 목적이 "
          "미탐지 방지에 있으므로 무조건 규칙으로 채택했다. 반영 후 기존 파이프라인 회귀 "
          "테스트 233건이 모두 통과했다.", "body"),

        P("6. 논의 및 한계", "h1"),
        P("<b>실측 검증의 제약.</b> 실외 실험은 안전상의 이유로 차량을 보행자에게 근접시킬 "
          "수 없어 위험 사례가 거의 발생하지 않는다. 실측 로그에서 차량이 이동한 구간의 "
          "속도 중앙값은 1.46 m/s로, 시뮬레이터의 7.35 m/s와 5배 차이를 보였다. 이는 "
          "시뮬레이터의 오류가 아니라 실험 조건의 제약이나, <b>고속 상황의 실측 검증이 "
          "부재한다는 점은 본 연구의 명확한 한계</b>다.", "body"),
        P("<b>반응 시간 문헌의 대용.</b> 시각장애인이 촉각 경보를 인지하고 정지하기까지의 "
          "시간을 직접 측정한 연구를 확보하지 못하여, 일반 보행자의 급정지 실험 데이터를 "
          "대용했다. 지팡이로 탐색하며 보행하는 경우 보행 속도가 낮아 정지가 더 빠를 "
          "가능성과, 상황 파악이 지연될 가능성이 모두 존재한다.", "body"),
        P("<b>오경보율의 해석.</b> 보고된 오경보율은 시뮬레이터의 생성 분포를 기준으로 한 "
          "값이므로 실제 운용 빈도로 환산할 수 없다. 세 방법 간의 상대 비교로만 해석해야 "
          "한다.", "body"),
        P("<b>단일 차량 가정.</b> 본 연구는 차량 1대와 보행자 1명의 관계만을 다루었다. "
          "현재 시스템의 상태 저장소가 단일 차량만을 추적하므로 시뮬레이터만 다중화하면 "
          "실제 시스템과 괴리가 발생한다. 다중 차량 상황에서의 위험 선택은 별도 과제다.", "body"),
    ]

    story += [
        P("7. 결론", "h1"),
        P("본 연구는 \"최적의 TTC 임계값은 몇 초인가\"라는 질문에서 출발하여, 그 질문이 "
          "성립하지 않음을 실증적으로 보였다.", "body"),
        P("모델이 탐지한 위험의 34%는 TTC가 무한대인 상태에서 발생했다. "
          "임계값을 어떤 값으로 설정하든 탐지되지 않는 위험이 존재한다면, "
          "최적화 대상은 임계값이 아니라 판단 구조다.", "quote"),
        P("조건부 판단 모델은 동일한 오경보 예산에서 적시 경보율을 66.6%에서 83.4%로 "
          "개선했다(p=0.034). 그러나 이 결과가 성립하기 위해서는 세 가지 전제가 필요했다. "
          "정답이 검증 대상과 독립적일 것, 평가 지표가 목적을 반영할 것, 그리고 학습 라벨이 "
          "대응 가능한 시점만을 정답으로 정의할 것이다. 세 번째 조건이 충족되기 전 개선폭은 "
          "통계적으로 유의하지 않았다.", "body"),
        P("동시에 현행 규칙 기반 점수표가 잘 설계되어 있음도 확인되었다. 특히 DCPA 게이트는 "
          "제거 시 성능이 절반으로 감소하는 핵심 구성요소이며, 모델의 우위는 가중치 조정이 "
          "아닌 비선형 결합에서 비롯된다. 학습 모델은 규칙을 대체하는 것이 아니라, 물리적 "
          "근거로 결정된 안전 하한 위에서 규칙이 다루지 못하는 영역을 보완하는 계층으로 "
          "배치되어야 한다.", "body"),
    ]

    # ── 참고문헌 / 부록 ─────────────────────────────────────────────────
    story += [
        P("참고문헌", "h1"),
        P("[1] ISO 15623:2013, <i>Intelligent transport systems — Forward vehicle "
          "collision warning systems — Performance requirements and test procedures</i>. "
          "고정 TTC 임계값을 규정하지 않으며, TTC 정의식과 2단계 경고 구조를 제시한다.", "ref"),
        P("[2] GB/T 33577-2017, <i>Intelligent transport — Forward vehicle collision "
          "warning systems</i>. ISO 15623 준용, 최소 2초 경보 요구.", "ref"),
        P("[3] Post-Encroachment Time. 교통 상충 분석의 표준 지표로, 한 객체가 특정 지점을 "
          "떠난 후 다른 객체가 해당 지점에 도달하기까지의 시간.", "ref"),
        P("[4] Responsibility-Sensitive Safety (RSS) 및 ISO 21448(SOTIF)의 안전 감시자 개념. "
          "학습 기반 판단의 실패에 대비해 물리적으로 결정된 제약을 하위 계층에 배치한다.", "ref"),
        P("[5] 촉각 자극에 대한 단순 반응시간 연구. 촉각 120~180ms로 청각(140~170ms) 및 "
          "시각(180~250ms)보다 빠르거나 유사한 범위.", "ref"),
        P("[6] 보행 중 급정지의 생체역학 연구. 제동시간 0.84~1.21초, 감속도 "
          "0.91~1.57 m/s<super>2</super>. 연령과 성별에 따른 차이 존재.", "ref"),
        P("[7] van der Horst, R. (1991). 보행자 상충 분석에서 TTC 1.5초 이하를 심각 상충으로 "
          "분류하는 기준의 근거.", "ref"),

        P("부록. 재현", "h1"),
        P("본 연구의 모든 수치는 다음 명령으로 재현된다. 코드와 상세 설계 문서는 "
          "저장소의 03_jetson/ttc_study/ 경로에 있다.", "body"),
        P("python evaluate.py --n-scenarios 1200 --repeats 5 --lead-s 2.0<br/>"
          "python alarm_profile.py --n-scenarios 1500<br/>"
          "python field_check.py<br/>"
          "python -m pytest        # 모듈 테스트 82건", "code"),
        P("구성 모듈: scenario_sim.py(시나리오 생성), oracle.py(규칙 독립 정답), "
          "features.py(특징 산출), dataset.py(층화 분할), baselines.py(규칙 채점), "
          "safety_floor.py(안전 하한), model.py(조건부 모델), evaluate.py(비교), "
          "field_check.py(실측 교차검증).", "body"),
    ]

    doc = SimpleDocTemplate(
        str(OUT), pagesize=A4,
        leftMargin=22 * mm, rightMargin=22 * mm,
        topMargin=20 * mm, bottomMargin=20 * mm,
        title="최적의 TTC는 존재하지 않는다",
        author="V2X 시각장애인 보행 지원 연구",
    )

    def footer(canvas, doc_):
        canvas.saveState()
        canvas.setFont("Malgun", 8)
        canvas.setFillColor(colors.HexColor("#888888"))
        canvas.drawCentredString(A4[0] / 2, 12 * mm, str(doc_.page))
        canvas.restoreState()

    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    return OUT


if __name__ == "__main__":
    print("저장:", build())
