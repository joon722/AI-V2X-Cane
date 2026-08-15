#!/usr/bin/env python3
"""구조 설명용 다이어그램 세 장.

표만으로는 "무엇이 무엇에 연결되는가"가 안 읽혀서 그림으로 만든다. 발표와 문서
양쪽에 쓰도록 PNG(붙여넣기용)와 SVG(PowerPoint 삽입용)를 함께 낸다.

  fig_decision_flow  노드 GPS에서 지팡이 진동까지의 판정 경로
  fig_layers         TTC 축으로 본 세 층 구조와 학습 데이터가 들어오는 자리
  fig_two_models     GBM과 v3가 도는 서로 다른 경로

색은 의미를 담는다 - 빨강은 물리로 계산해 검증된 층, 주황은 학습 기반이라 분포
밖에서 틀릴 수 있는 층이다. 회색은 검토했으나 넣지 않은 것이다.
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

HERE = Path(__file__).parent

# make_findings.py의 PDF 색과 맞춘다. 문서와 그림이 따로 놀지 않게 하기 위해서다.
C = {
    "red":   ("#c0392b", "#fdeaea"),
    "amber": ("#b9770e", "#fdf3e3"),
    "blue":  ("#2f6ea5", "#eaf1f8"),
    "teal":  ("#148f77", "#e8f6f3"),
    "gray":  ("#7f8c8d", "#f2f4f6"),
}


def use_korean_font():
    for name in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


def canvas(w_in, h_in, xmax=100, ymax=100):
    fig, ax = plt.subplots(figsize=(w_in, h_in))
    ax.set_xlim(0, xmax)
    ax.set_ylim(0, ymax)
    ax.axis("off")
    ax.invert_yaxis()  # 위에서 아래로 읽는 순서와 좌표를 맞춘다
    return fig, ax


def box(ax, x, y, w, h, title, lines=(), color="gray", title_size=11, line_size=9):
    edge, face = C[color]
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0,rounding_size=1.2",
        linewidth=1.3, edgecolor=edge, facecolor=face, zorder=2))
    # 제목과 본문 줄을 박스 높이에 균등 배분한다. 줄 수가 늘어도 넘치지 않게
    # 간격을 고정값이 아니라 높이에서 나눠 쓴다.
    cx = x + w / 2
    rows = [(title, title_size, "#1a1a1a")] + [
        (line, line_size, "#4a4a4a") for line in lines]
    gap = h / (len(rows) + 1)
    for i, (text, size, color) in enumerate(rows):
        ax.text(cx, y + gap * (i + 1), text, ha="center", va="center",
                fontsize=size, color=color, zorder=3)


def arrow(ax, x1, y1, x2, y2, dashed=False, color="#666"):
    ax.add_patch(FancyArrowPatch(
        (x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=11,
        linewidth=1.2, color=color, zorder=1,
        linestyle=(0, (4, 3)) if dashed else "solid",
        shrinkA=0, shrinkB=0))


def save(fig, stem):
    for ext in ("png", "svg"):
        fig.savefig(HERE / f"{stem}.{ext}", dpi=200, bbox_inches="tight",
                    transparent=False, facecolor="white")
    plt.close(fig)
    print(f"  {stem}.png / .svg")


# --- 1. 판정 흐름 -----------------------------------------------------------

def decision_flow():
    fig, ax = canvas(9.5, 7.4, 100, 84)

    box(ax, 12, 2, 30, 9, "지팡이 노드", ["GPS 5Hz"], "blue")
    box(ax, 58, 2, 30, 9, "차량 노드", ["GPS 5Hz"], "blue")
    arrow(ax, 30, 11, 38, 16)
    arrow(ax, 70, 11, 62, 16)

    box(ax, 22, 16, 56, 10, "칼만 필터 · 상대 운동학",
        ["거리 · 접근속도 · TTC · DCPA · TCPA"], "gray")

    arrow(ax, 42, 26, 19, 32)
    arrow(ax, 50, 26, 50, 32)
    arrow(ax, 58, 26, 81, 32)

    box(ax, 2, 32, 29, 14, "안전 하한",
        ["TTC 2초 이하 → 레벨 3", "AI가 못 내림"], "red")
    box(ax, 35.5, 32, 29, 14, "규칙 점수표",
        ["5항목 × 스침거리 계수", "레벨 0~3"], "red")
    box(ax, 69, 32, 29, 14, "AI 모델",
        ["15개 값 → 위험 확률", "올리기만 · 레벨 2까지"], "amber")

    arrow(ax, 16.5, 46, 42, 52)
    arrow(ax, 50, 46, 50, 52)
    arrow(ax, 83.5, 46, 58, 52)

    box(ax, 33, 52, 34, 10, "셋 중 최대값",
        ["내려갈 땐 2초 유지"], "gray")
    arrow(ax, 50, 62, 50, 66)

    box(ax, 28, 66, 44, 8, "무선 전송 — 레벨 숫자 하나만", (), "gray", 10)
    arrow(ax, 50, 74, 50, 77)

    box(ax, 36, 77, 28, 6, "지팡이 진동", (), "blue", 10)

    fig.tight_layout()
    save(fig, "fig_decision_flow")


# --- 2. 세 층 구조 ----------------------------------------------------------

def layers():
    fig, ax = canvas(9.5, 6.2, 100, 70)

    ax.text(6, 4, "TTC 큼", ha="center", fontsize=9, color="#4a4a4a")
    arrow(ax, 6, 7, 6, 55, color="#999")
    ax.text(6, 59, "TTC 작음", ha="center", fontsize=9, color="#4a4a4a")

    box(ax, 14, 4, 52, 12, "상한 게이트",
        ["멀고 접근 안 함 → 무조건 레벨 0", "재봤더니 손해라 넣지 않음"], "gray")
    box(ax, 14, 20, 52, 18, "AI 조건부 판단",
        ["각도 · 속도 · DCPA · 보행자 상태를 보고",
         "상황마다 다르게 판단",
         "TTC로는 안 보이는 위험까지 잡는다"], "amber")
    box(ax, 14, 42, 52, 12, "안전 하한",
        ["TTC 2초 이하 → 무조건 레벨 3", "AI가 뭐라 하든 무시"], "red")

    arrow(ax, 74, 29, 67, 29)
    box(ax, 74, 21, 24, 16, "학습 데이터",
        ["지금 — 자체 시뮬", "예정 — SUMO", "오라클로 라벨링"], "teal", 10, 8.5)

    ax.text(14, 61, "최종 = max( 안전 하한, 규칙 점수표, AI 판정 )",
            fontsize=11, color="#1a1a1a")

    for i, (label, key) in enumerate(
            (("절대 규칙", "red"), ("학습 기반", "amber"),
             ("보류", "gray"), ("데이터 공급", "teal"))):
        x = 14 + i * 21
        ax.add_patch(FancyBboxPatch(
            (x, 66), 2.4, 2.4, boxstyle="round,pad=0,rounding_size=0.4",
            linewidth=1, edgecolor=C[key][0], facecolor=C[key][1]))
        ax.text(x + 3.6, 67.2, label, va="center", fontsize=8.5, color="#4a4a4a")

    fig.tight_layout()
    save(fig, "fig_layers")


# --- 3. 두 모델 경로 --------------------------------------------------------

def two_models():
    fig, ax = canvas(9.5, 4.6, 100, 52)

    ax.text(2, 4, "지팡이 실시간 경보", fontsize=11, color="#1a1a1a")
    row1 = [("노드 GPS", "지팡이 · 차량", "blue"),
            ("젯슨 step2~8", "5Hz 실시간", "blue"),
            ("GBM", "risk_model.json", "amber"),
            ("지팡이 진동", "0.1초 안", "blue")]
    for i, (title, sub, key) in enumerate(row1):
        x = 2 + i * 25
        box(ax, x, 7, 21, 10, title, [sub], key, 10, 8.5)
        if i < 3:
            arrow(ax, x + 21, 12, x + 25, 12)

    ax.plot([2, 98], [23, 23], color="#ccc", linewidth=1, linestyle=(0, (4, 3)))

    ax.text(2, 29, "위험지도", fontsize=11, color="#1a1a1a")
    row2 = [("GCP 서버", "SUMO 시나리오", "teal"),
            ("젯슨 cron", "1분마다 배치", "teal"),
            ("v3 트랜스포머", "ONNX", "amber"),
            ("위험지도", "웹 · 행정용", "teal")]
    for i, (title, sub, key) in enumerate(row2):
        x = 2 + i * 25
        box(ax, x, 32, 21, 10, title, [sub], key, 10, 8.5)
        if i < 3:
            arrow(ax, x + 21, 37, x + 25, 37)

    ax.text(2, 48, "두 흐름은 데이터를 주고받지 않는다. "
                   "같이 쓰는 것은 step7의 계산 함수 몇 개뿐이다.",
            fontsize=8.5, color="#4a4a4a")

    fig.tight_layout()
    save(fig, "fig_two_models")


# --- 4. 전체 한 장 ----------------------------------------------------------

def overview():
    """실시간 경보와 지도, 그리고 둘을 잇게 될 학습 경로를 한 장에 담는다."""
    fig, ax = canvas(8.4, 9.9, 100, 118)

    box(ax, 10, 2, 28, 9, "지팡이 노드", ["GPS 5Hz"], "blue", 10)
    box(ax, 56, 2, 28, 9, "차량 노드", ["GPS 5Hz"], "blue", 10)
    arrow(ax, 28, 11, 36, 16)
    arrow(ax, 66, 11, 58, 16)

    box(ax, 20, 16, 54, 10, "칼만 필터 · 상대 운동학",
        ["거리 · 접근속도 · TTC · DCPA · TCPA"], "gray", 10, 8.5)
    arrow(ax, 40, 26, 18, 32)
    arrow(ax, 47, 26, 47, 32)
    arrow(ax, 54, 26, 76, 32)

    box(ax, 2, 32, 28, 16, "안전 하한",
        ["TTC 2초 이하", "→ 무조건 레벨 3", "AI가 못 내림"], "red", 10, 8.5)
    box(ax, 33, 32, 28, 16, "규칙 점수표",
        ["5항목 × 스침거리", "레벨 0~3"], "red", 10, 8.5)
    box(ax, 64, 32, 28, 16, "AI 모델",
        ["15개 값 → 확률", "올리기만 · 레벨 2", "자체 시뮬로 학습"], "amber", 10, 8.5)

    arrow(ax, 16, 48, 40, 54)
    arrow(ax, 47, 48, 47, 54)
    arrow(ax, 78, 48, 54, 54)

    box(ax, 33, 54, 28, 10, "셋 중 최대값", ["내려갈 땐 2초 유지"], "gray", 10, 8.5)
    arrow(ax, 47, 64, 47, 68)
    box(ax, 25, 68, 44, 8, "무선 전송 — 레벨 숫자 하나만", (), "gray", 10)
    arrow(ax, 47, 76, 47, 79)
    box(ax, 33, 79, 28, 6, "지팡이 진동", (), "blue", 10)

    ax.plot([2, 98], [88, 88], color="#ccc", linewidth=1, linestyle=(0, (4, 3)))
    ax.text(2, 95, "위험지도 — 별도 경로", fontsize=10, color="#1a1a1a")

    row = [("GCP 서버", "SUMO 시나리오", "teal"),
           ("젯슨 cron", "1분마다 배치", "teal"),
           ("v3 트랜스포머", "ONNX", "amber"),
           ("위험지도", "웹 · 행정용", "teal")]
    for i, (title, sub, key) in enumerate(row):
        x = 2 + i * 24
        box(ax, x, 98, 21, 10, title, [sub], key, 9.5, 8)
        if i < 3:
            arrow(ax, x + 21, 103, x + 24, 103)

    # SUMO 데이터를 AI 학습에도 쓰는 경로. 아직 코드가 없어 점선으로 둔다.
    ax.plot([12.5, 12.5, 95, 95], [98, 92, 92, 44],
            linestyle=(0, (4, 3)), color=C["teal"][0], linewidth=1.2, zorder=1)
    arrow(ax, 95, 44, 92.5, 44, color=C["teal"][0])
    ax.text(54, 90, "같은 데이터로 AI를 학습시키는 경로 — 아직 없음",
            fontsize=8.5, color=C["teal"][0], ha="center")

    ax.text(2, 114, "빨강 = 물리로 계산한 층 (AI가 못 뒤집음)   ·   "
                    "주황 = 학습 기반   ·   청록 = 시뮬레이션 쪽",
            fontsize=8.5, color="#4a4a4a")

    fig.tight_layout()
    save(fig, "fig_overview")


def main():
    use_korean_font()
    print("생성:")
    decision_flow()
    layers()
    two_models()
    overview()


if __name__ == "__main__":
    main()
