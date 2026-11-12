"""Gera os gráficos de resposta em frequência usados em passa-baixas.md.

Uso: python graficos_butterworth.py   (requer numpy, scipy, matplotlib)
Os SVGs são gravados em ../img/.
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter
from scipy import signal

FS = 2000.0  # Hz, taxa do backend usada nos exemplos do texto
OUT = Path(__file__).resolve().parent.parent / "img"

# Paleta (tema claro) e tokens de texto
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT2 = "#52514e"
GRID = "#e4e3df"
REF = "#8a8984"
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 10,
    "axes.facecolor": SURFACE,
    "figure.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID,
    "axes.labelcolor": TEXT2,
    "xtick.color": TEXT2,
    "ytick.color": TEXT2,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "lines.linewidth": 2,
    "lines.solid_capstyle": "round",
    "svg.fonttype": "none",
})


def biquad(fc, fs=FS):
    """Coeficientes de LowPassFilter2p (Butterworth 2ª ordem, bilinear com prewarp)."""
    K = np.tan(np.pi * fc / fs)
    c = 1 + np.sqrt(2) * K + K * K
    b0 = K * K / c
    b = [b0, 2 * b0, b0]
    a = [1, 2 * (K * K - 1) / c, (1 - np.sqrt(2) * K + K * K) / c]
    return b, a


def primeira_ordem(fc, fs=FS):
    """Coeficientes de LowPassFilter (RC por Euler implícito)."""
    T, rc = 1 / fs, 1 / (2 * np.pi * fc)
    alpha = T / (T + rc)
    return [alpha], [1, -(1 - alpha)]


def resposta(b, a, f):
    _, h = signal.freqz(b, a, worN=f, fs=FS)
    _, gd = signal.group_delay((b, a), w=f, fs=FS)
    mag = 20 * np.log10(np.maximum(np.abs(h), 1e-12))
    fase = np.degrees(np.unwrap(np.angle(h)))
    return mag, fase, gd / FS * 1e3  # atraso de grupo em ms


def eixo_freq(ax):
    ax.set_xscale("log")
    ax.set_xlim(F[0], F[-1])
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))


def num(v, casas=0):
    """Número em formato pt-BR, com sinal de menos tipográfico."""
    return f"{v:.{casas}f}".replace("-", "\u2212").replace(".", ",")


def rotulo(ax, x, y, texto, cor, dy=0, ha="left"):
    """Rótulo direto ao lado da curva, com marcador da cor da série."""
    ax.plot([x], [y], "o", ms=5, color=cor, mec=SURFACE, mew=1.5, zorder=5)
    ax.annotate(texto, (x, y), xytext=(6 if ha == "left" else -6, dy),
                textcoords="offset points", ha=ha, va="center", color=TEXT, fontsize=9)


F = np.logspace(np.log10(0.5), np.log10(999), 2000)


def grafico_bode():
    fc = 20.0
    m2, p2, g2 = resposta(*biquad(fc), F)
    m1, p1, g1 = resposta(*primeira_ordem(fc), F)

    fig, (am, ap, ag) = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    fig.suptitle(f"Passa-baixas com fc = {fc:g} Hz, fs = {FS/1000:g} kHz",
                 color=TEXT, fontsize=12, x=0.08, ha="left")

    for ax in (am, ap, ag):
        eixo_freq(ax)
        ax.axvline(fc, color=REF, lw=1, ls="--", zorder=1)

    am.plot(F, m2, color=C1, label="Butterworth 2ª ordem (INS_GYRO_FILTER)")
    am.plot(F, m1, color=C2, label="1ª ordem (filtros do AC_PID)")
    am.axhline(-3, color=REF, lw=1, ls=":", zorder=1)
    am.set_ylim(-80, 5)
    am.set_ylabel("Ganho (dB)")
    rotulo(am, fc, -3, "−3 dB em fc", TEXT2, dy=-10)
    i = np.argmin(abs(F - 300))
    rotulo(am, 300, m2[i], f"{num(m2[i])} dB  (−40 dB/déc)", C1, dy=-8)
    rotulo(am, 300, m1[i], f"{num(m1[i])} dB  (−20 dB/déc)", C2, dy=8)
    am.legend(loc="lower left", frameon=False, labelcolor=TEXT, fontsize=9)

    ap.plot(F, p2, color=C1)
    ap.plot(F, p1, color=C2)
    ap.set_ylim(-185, 5)
    ap.set_yticks([0, -45, -90, -135, -180])
    ap.set_ylabel("Fase (°)")
    i10 = np.argmin(abs(F - 10))
    rotulo(ap, 10, p2[i10], f"{num(p2[i10])}° a 10 Hz", C1, dy=-8)
    rotulo(ap, 10, p1[i10], f"{num(p1[i10])}° a 10 Hz", C2, dy=8)
    rotulo(ap, fc, -90, "−90° em fc", C1, dy=-8)

    ag.plot(F, g2, color=C1)
    ag.plot(F, g1, color=C2)
    ag.set_ylim(0, 16)
    ag.set_ylabel("Atraso de grupo (ms)")
    ag.set_xlabel("Frequência (Hz)")
    i1 = np.argmin(abs(F - 1))
    rotulo(ag, 1, g2[i1], f"{num(g2[i1], 1)} ms", C1, dy=8)
    rotulo(ag, 1, g1[i1], f"{num(g1[i1], 1)} ms", C2, dy=-8)

    fig.tight_layout()
    fig.savefig(OUT / "butterworth-bode.svg")
    plt.close(fig)


def grafico_cortes():
    cortes = [(20, C1, -3), (40, C2, 0), (80, C3, 4)]
    fig, (am, ap) = plt.subplots(2, 1, figsize=(8, 6.5), sharex=True)
    fig.suptitle(f"Butterworth 2ª ordem: efeito de INS_GYRO_FILTER (fs = {FS/1000:g} kHz)",
                 color=TEXT, fontsize=12, x=0.08, ha="left")
    for ax in (am, ap):
        eixo_freq(ax)
        ax.axvline(10, color=REF, lw=1, ls="--", zorder=1)
    am.text(10 * 1.05, 2, "10 Hz (banda de controle)", color=TEXT2, fontsize=8, va="bottom")

    for fc, cor, dy in cortes:
        m, p, _ = resposta(*biquad(fc), F)
        am.plot(F, m, color=cor, label=f"fc = {fc} Hz")
        ap.plot(F, p, color=cor)
        i = np.argmin(abs(F - 200))
        rotulo(am, 200, m[i], f"{num(m[i])} dB a 200 Hz", cor)
        i = np.argmin(abs(F - 10))
        rotulo(ap, 10, p[i], f"{num(p[i])}°", cor, dy=dy, ha="right")

    am.axhline(-3, color=REF, lw=1, ls=":", zorder=1)
    am.set_ylim(-80, 5)
    am.set_ylabel("Ganho (dB)")
    am.legend(loc="lower left", frameon=False, labelcolor=TEXT, fontsize=9)
    ap.set_ylim(-185, 5)
    ap.set_yticks([0, -45, -90, -135, -180])
    ap.set_ylabel("Fase (°)")
    ap.set_xlabel("Frequência (Hz)")

    fig.tight_layout()
    fig.savefig(OUT / "butterworth-cortes.svg")
    plt.close(fig)


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    grafico_bode()
    grafico_cortes()
    print("gráficos gravados em", OUT)
