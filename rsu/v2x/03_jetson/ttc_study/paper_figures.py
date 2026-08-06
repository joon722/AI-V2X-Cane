#!/usr/bin/env python3
"""논문에 넣을 보조 그림 두 장.

Figure 1  세 방법의 적시경보-오경보 트레이드오프. 오경보 예산을 고정한 비교이므로
          한 축에 몰아 그리면 "같은 값을 치르고 무엇을 얻었나"가 바로 읽힌다.
Figure 2  예측 창 길이별 AUC. 창을 늘릴수록 맞힐 수 없는 문제가 된다는 것을 보이고,
          3초를 고른 근거를 시각화한다.

수치는 evaluate.py와 라벨 창 실험에서 나온 값을 그대로 옮긴 것이다. 재현하려면
  python evaluate.py --n-scenarios 1200 --repeats 5 --lead-s 2.0
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


def use_korean_font():
    for name in ("Malgun Gothic", "NanumGothic", "AppleGothic"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


# (이름, 적시경보%, 표준편차, 오경보%, 색, 마커)
METHODS = [
    ("현 점수표", 66.6, 5.2, 1.99, "#4a7fb5", "o"),
    ("TTC <= 2.0s", 30.8, 8.4, 2.15, "#8c8c8c", "s"),
    ("TTC <= 2.5s", 70.0, 10.9, 3.20, "#8c8c8c", "s"),
    ("TTC <= 3.0s", 82.7, 6.7, 4.29, "#8c8c8c", "s"),
    ("조건부 모델", 83.4, 8.3, 1.99, "#c0392b", "D"),
]

HORIZON_AUC = [(2.0, 0.958), (3.0, 0.713), (5.0, 0.719), (10.0, 0.541)]


def figure_tradeoff(out_path):
    use_korean_font()
    fig, ax = plt.subplots(figsize=(7.2, 5.0))

    for name, timely, sd, far, color, marker in METHODS:
        ax.errorbar(far, timely, yerr=sd, fmt=marker, color=color, markersize=11,
                    capsize=4, elinewidth=1.2, markeredgecolor="white",
                    markeredgewidth=1.2, zorder=3)
        dx = 0.12
        ha = "left"
        if name == "TTC <= 3.0s":
            dx, ha = -0.12, "right"
        ax.annotate(name, (far + dx, timely), va="center", ha=ha, fontsize=10)

    # 같은 오경보 예산에서의 개선폭
    ax.annotate("", xy=(1.99, 83.4), xytext=(1.99, 66.6),
                arrowprops=dict(arrowstyle="->", color="#c0392b", lw=2))
    ax.text(1.86, 75.0, "+16.9%p\n(p=0.034)", color="#c0392b", fontsize=10,
            ha="right", va="center", fontweight="bold")

    ax.set_xlabel("오경보율 (%)  — 낮을수록 좋다")
    ax.set_ylabel("적시 경보율 (%)  — 높을수록 좋다")
    ax.set_title("같은 오경보를 치르고 무엇을 얻었나", fontsize=13, pad=12)
    ax.grid(alpha=0.3)
    ax.set_xlim(1.5, 4.9)
    ax.set_ylim(20, 100)
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    return out_path


def figure_horizon(out_path):
    use_korean_font()
    fig, ax = plt.subplots(figsize=(7.2, 4.2))

    h = [x for x, _ in HORIZON_AUC]
    auc = [y for _, y in HORIZON_AUC]
    ax.plot(h, auc, "o-", color="#4a7fb5", markersize=9, linewidth=2)
    for x, y in HORIZON_AUC:
        ax.annotate(f"{y:.3f}", (x, y), textcoords="offset points",
                    xytext=(0, 11), ha="center", fontsize=9.5)

    ax.axhline(0.5, color="#8c8c8c", linestyle=":", linewidth=1.5)
    ax.text(9.6, 0.515, "무작위 (0.5)", ha="right", fontsize=9, color="#666")

    ax.axvspan(2.6, 3.4, color="#c0392b", alpha=0.10)
    ax.text(3.0, 0.90, "채택: 3초", ha="center", fontsize=10, color="#c0392b",
            fontweight="bold")
    ax.annotate("반응 시간(2.0s) +\n여유 1.0s", xy=(3.0, 0.713), xytext=(4.6, 0.86),
                arrowprops=dict(arrowstyle="->", color="#c0392b", lw=1.4),
                fontsize=9, color="#c0392b")

    ax.set_xlabel("예측 창 길이 (초)")
    ax.set_ylabel("AUC")
    ax.set_title("멀리 볼수록 맞힐 수 없는 문제가 된다", fontsize=13, pad=12)
    ax.grid(alpha=0.3)
    ax.set_xlim(1.4, 10.6)
    ax.set_ylim(0.45, 1.0)
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    return out_path


def main():
    here = Path(__file__).parent
    print("저장:", figure_tradeoff(here / "fig_tradeoff.png"))
    print("저장:", figure_horizon(here / "fig_horizon.png"))


if __name__ == "__main__":
    main()
