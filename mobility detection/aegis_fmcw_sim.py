#!/usr/bin/env python3
"""
aegis_fmcw_sim.py
FMCW through-rubble vital-sign radar simulator (post-dechirp baseband model).

Pipeline (the same chain you would later move into FPGA):
  scene -> dechirped IQ per chirp -> 12-bit ADC
  -> range FFT -> platform-motion compensation -> static-clutter removal
  -> vital-band energy map (picks the range bin with the human)
  -> PCA displacement signal -> band-pass -> breathing / heart rate

Usage:
  python aegis_fmcw_sim.py                  # one run + plots
  python aegis_fmcw_sim.py --loss 22        # 22 dB ONE-WAY rubble loss
  python aegis_fmcw_sim.py --platform 6     # drone hover motion, mm rms
  python aegis_fmcw_sim.py --sweep          # detection probability vs rubble loss
  python aegis_fmcw_sim.py --export         # int12 IQ frame as golden vector for HDL

Deliberately NOT modelled (add these once you have real measurements):
  TX->RX leakage, phase noise, rubble dielectric delay, multipath inside the
  rubble, chest orientation (cos factor), range migration inside a chirp.
  Rubble loss is a placeholder number, replace it with measured dB.
"""
import argparse
from dataclasses import dataclass

import numpy as np
from scipy import signal
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

C = 299_792_458.0
KB = 1.380649e-23


# ------------------------------------------------------------------ parameters
@dataclass
class Radar:
    fc: float = 2.4e9        # centre frequency (ISM band)
    bw: float = 50e6         # sweep bandwidth (Pluto-class)
    tc: float = 1e-3         # chirp duration
    fs: float = 100e3        # post-dechirp complex sample rate
    prf: float = 50.0        # chirps per second = slow-time sample rate
    pt_dbm: float = 10.0     # TX power
    gt_dbi: float = 10.0
    gr_dbi: float = 10.0
    nf_db: float = 6.0
    adc_bits: int = 12       # 0 = ideal ADC
    t_obs: float = 30.0      # observation window (s)
    window: str = "hann"     # range-FFT window: "hann" or "cheb80" (80 dB sidelobes)

    @property
    def lam(self):
        return C / self.fc

    @property
    def slope(self):
        return self.bw / self.tc

    @property
    def n_fast(self):
        return int(round(self.fs * self.tc))

    @property
    def n_slow(self):
        return int(round(self.prf * self.t_obs))

    @property
    def range_res(self):
        return C / (2 * self.bw)


@dataclass
class Human:
    range_m: float = 7.0
    rcs_m2: float = 0.1
    breath_bpm: float = 15.0
    breath_mm: float = 4.0      # chest displacement amplitude
    heart_bpm: float = 72.0
    heart_mm: float = 0.3
    rubble_loss_db: float = 15.0  # ONE-WAY loss through rubble (placeholder)


@dataclass
class Clutter:
    range_m: float
    rcs_m2: float


DEFAULT_CLUTTER = [
    Clutter(3.0, 5.0),    # rubble front face (also the platform-motion reference)
    Clutter(11.0, 2.0),   # ground
    Clutter(15.0, 3.0),   # far wall
]


# ------------------------------------------------------------------ link budget
def rx_power_w(radar, rcs, rng_m, one_way_loss_db=0.0):
    pt = 1e-3 * 10 ** (radar.pt_dbm / 10)
    gt = 10 ** (radar.gt_dbi / 10)
    gr = 10 ** (radar.gr_dbi / 10)
    p = pt * gt * gr * radar.lam ** 2 * rcs / ((4 * np.pi) ** 3 * rng_m ** 4)
    return p * 10 ** (-2 * one_way_loss_db / 10)  # loss applies on the way in AND out


def noise_power_w(radar, temp=290.0):
    return KB * temp * radar.fs * 10 ** (radar.nf_db / 10)


def dbm(w):
    return 10 * np.log10(w / 1e-3)


# ------------------------------------------------------------------ simulation
def platform_motion(radar, rms_mm, rng):
    """Drone hover motion: band-limited noise, 0.05-5 Hz, given rms (metres)."""
    if rms_mm <= 0:
        return np.zeros(radar.n_slow)
    sos = signal.butter(2, [0.05, 5.0], btype="band", fs=radar.prf, output="sos")
    x = signal.sosfilt(sos, rng.standard_normal(radar.n_slow + 500))[500:]
    return x * (rms_mm * 1e-3) / np.std(x)


def simulate(radar, human, clutter, platform_rms_mm, rng):
    tf = np.arange(radar.n_fast) / radar.fs
    tm = np.arange(radar.n_slow) / radar.prf
    p = platform_motion(radar, platform_rms_mm, rng)

    ph_b, ph_h = rng.uniform(0, 2 * np.pi, 2)
    chest_b = (human.breath_mm * 1e-3) * np.sin(2 * np.pi * human.breath_bpm / 60 * tm + ph_b)
    chest_h = (human.heart_mm * 1e-3) * np.sin(2 * np.pi * human.heart_bpm / 60 * tm + ph_h)

    targets = [(c.range_m, rx_power_w(radar, c.rcs_m2, c.range_m), 0.0) for c in clutter]
    targets.append((human.range_m,
                    rx_power_w(radar, human.rcs_m2, human.range_m, human.rubble_loss_db),
                    chest_b + chest_h))

    iq = np.zeros((radar.n_slow, radar.n_fast), complex)
    for r0, pw, disp in targets:
        r = r0 + p + disp                              # range per chirp
        fb = 2 * radar.slope * r / C                   # beat frequency
        ph0 = 4 * np.pi * radar.fc * r / C + rng.uniform(0, 2 * np.pi)
        iq += np.sqrt(pw) * np.exp(1j * (2 * np.pi * fb[:, None] * tf[None, :] + ph0[:, None]))

    sigma = np.sqrt(noise_power_w(radar) / 2)
    iq += sigma * (rng.standard_normal(iq.shape) + 1j * rng.standard_normal(iq.shape))

    codes = None
    if radar.adc_bits:
        fs_amp = 1.1 * max(np.abs(iq.real).max(), np.abs(iq.imag).max())  # ideal AGC
        q = 2 ** (radar.adc_bits - 1)
        ci = np.clip(np.round(iq.real / fs_amp * q), -q, q - 1).astype(np.int16)
        cq = np.clip(np.round(iq.imag / fs_amp * q), -q, q - 1).astype(np.int16)
        iq = (ci + 1j * cq) / q * fs_amp
        codes = (ci, cq)

    truth = dict(tm=tm, chest_b=chest_b, chest_h=chest_h, platform=p)
    return iq, codes, truth


# ------------------------------------------------------------------ processing
def range_fft(radar, iq, nfft=1024, r_min=0.5, r_max=20.0):
    if radar.window == "cheb80":
        win = signal.windows.chebwin(radar.n_fast, at=80)
    else:
        win = np.hanning(radar.n_fast)
    rp = np.fft.fft(iq * win[None, :], n=nfft, axis=1)
    f = np.fft.fftfreq(nfft, 1 / radar.fs)
    ra = f * C / (2 * radar.slope)
    keep = (ra >= r_min) & (ra <= r_max)
    return rp[:, keep], ra[keep]


def compensate_raw(radar, iq, ref_series):
    """Remove common platform motion from the raw dechirped IQ, BEFORE the range FFT.

    The strongest static reflector (rubble face) gives the motion. Two terms are
    corrected: the carrier phase 4*pi*p/lambda and the range-dependent beat-phase
    term 4*pi*S*p*t_fast/c. Fixing only the first leaves a small residual on the
    huge rubble-face echo that is independent of rubble loss and can swamp a
    weak human."""
    theta = np.unwrap(np.angle(ref_series))
    theta -= theta.mean()
    # The FFT-bin phase of a static reflector also carries the beat term at the window
    # centre, so it scales as 4*pi*p*(1/lambda + S*t_mid/c), not 4*pi*p/lambda.
    # Using the wrong scale leaves a ~1 % residual (30 um for 3 mm of motion) that
    # lands on the huge rubble-face echo.
    t_mid = (radar.n_fast - 1) / (2 * radar.fs)
    p = theta / (4 * np.pi * (1 / radar.lam + radar.slope * t_mid / C))   # platform motion (m)
    tf = np.arange(radar.n_fast) / radar.fs
    corr = np.exp(-1j * 4 * np.pi * p[:, None] * (1 / radar.lam + radar.slope * tf[None, :] / C))
    return iq * corr, theta


def range_tol(radar):
    """A detection counts as in the right place if within one cell (min 0.75 m)."""
    return max(radar.range_res / 2, 0.75)


def band_energy_map(radar, rp, f_lo=0.1, f_hi=2.0):
    x = rp - rp.mean(axis=0, keepdims=True)            # static clutter removal
    x = x * np.hanning(x.shape[0])[:, None]
    sp = np.abs(np.fft.fft(x, axis=0)) ** 2
    f = np.fft.fftfreq(x.shape[0], 1 / radar.prf)
    band = (np.abs(f) >= f_lo) & (np.abs(f) <= f_hi)
    return sp[band].sum(axis=0)


def peak_bpm(sig, fs, lo, hi, nfft=2 ** 16):
    sp = np.abs(np.fft.rfft(sig * np.hanning(len(sig)), nfft))
    f = np.fft.rfftfreq(nfft, 1 / fs)
    m = (f >= lo) & (f <= hi)
    return 60 * f[m][np.argmax(sp[m])]


def extract_vitals(radar, x):
    x = x - x.mean()
    xy = np.c_[x.real, x.imag]
    _, _, vt = np.linalg.svd(xy, full_matrices=False)
    d = xy @ vt[0]                                      # displacement-proportional signal
    sb = signal.butter(4, [0.1, 0.6], "band", fs=radar.prf, output="sos")
    sh = signal.butter(4, [0.8, 2.0], "band", fs=radar.prf, output="sos")
    breath = signal.sosfiltfilt(sb, d)
    heart = signal.sosfiltfilt(sh, d)
    nfft = 2 ** 16
    spec = np.abs(np.fft.rfft(d * np.hanning(len(d)), nfft))
    faxis = np.fft.rfftfreq(nfft, 1 / radar.prf)
    return dict(d=d, breath=breath, heart=heart,
                breath_bpm=peak_bpm(breath, radar.prf, 0.1, 0.6),
                heart_bpm=peak_bpm(heart, radar.prf, 0.8, 2.0),
                faxis=faxis, spec=spec)


def run_chain(radar, iq, comp=True, guard_m=0.0):
    rp, ra = range_fft(radar, iq)
    ref = int(np.argmax(np.abs(rp).mean(axis=0)))
    if comp:
        iq_c, theta = compensate_raw(radar, iq, rp[:, ref])
        rp_use, _ = range_fft(radar, iq_c)
    else:
        rp_use, theta = rp, np.zeros(rp.shape[0])
    energy = band_energy_map(radar, rp_use)
    # Optional guard zone around the strongest reflector (off by default). It made no
    # difference once the motion compensation was exact, and it assumes no victim
    # within guard_m of the rubble face, so only turn it on if you see clutter picks.
    masked = np.where(np.abs(ra - ra[ref]) < guard_m, 0.0, energy)
    sel = int(np.argmax(masked))
    v = extract_vitals(radar, rp_use[:, sel])
    return dict(ra=ra, rp=rp, ref=ref, energy=energy, sel=sel,
                sel_range=float(ra[sel]), theta=theta, **v)


# ------------------------------------------------------------------ plots
def _norm(x):
    return (x - x.mean()) / (np.std(x) + 1e-30)


def plot_run(radar, human, truth, a, b, summary, path):
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 3)

    ax = fig.add_subplot(gs[0, 0])
    ax.plot(a["ra"], 20 * np.log10(np.abs(a["rp"]).mean(axis=0) + 1e-30), c="k")
    ax.axvline(human.range_m, ls="--", c="r", label="human (true range)")
    ax.set_title("1. Plain range profile: human is invisible")
    ax.set_xlabel("range (m)")
    ax.set_ylabel("mean |range FFT| (dB)")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[0, 1])
    ref_db = 10 * np.log10(max(a["energy"].max(), b["energy"].max()))
    ax.plot(a["ra"], 10 * np.log10(a["energy"] + 1e-30) - ref_db, c="g", label="with motion compensation")
    ax.plot(b["ra"], 10 * np.log10(b["energy"] + 1e-30) - ref_db, c="gray", label="no compensation")
    ax.axvline(human.range_m, ls="--", c="r")
    ax.set_title("2. Vital-band energy vs range")
    ax.set_xlabel("range (m)")
    ax.set_ylabel("dB rel. max")
    ax.set_ylim(-110, 3)
    ax.legend()
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[0, 2])
    m = (a["faxis"] > 0.05) & (a["faxis"] < 3.0)
    ax.plot(a["faxis"][m], a["spec"][m] / a["spec"][m].max(), c="g")
    ax.axvline(human.breath_bpm / 60, ls="--", c="r")
    ax.axvline(human.heart_bpm / 60, ls="--", c="r")
    ax.set_title("3. Spectrum at selected bin (red = truth)")
    ax.set_xlabel("Hz")
    ax.grid(alpha=0.3)

    tm = truth["tm"]
    win = tm < 20
    ax = fig.add_subplot(gs[1, 0])
    s = np.sign(np.corrcoef(a["breath"], truth["chest_b"])[0, 1]) or 1.0
    ax.plot(tm[win], _norm(truth["chest_b"])[win], c="r", alpha=0.6, label="truth")
    ax.plot(tm[win], s * _norm(a["breath"])[win], c="g", label="recovered")
    ax.set_title("4. Breathing")
    ax.set_xlabel("s")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 1])
    s = np.sign(np.corrcoef(a["heart"], truth["chest_h"])[0, 1]) or 1.0
    ax.plot(tm[win], _norm(truth["chest_h"])[win], c="r", alpha=0.6, label="truth")
    ax.plot(tm[win], s * _norm(a["heart"])[win], c="g", label="recovered")
    ax.set_title("5. Heartbeat")
    ax.set_xlabel("s")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    ax.text(0, 1, summary, va="top", family="monospace", fontsize=10)

    fig.suptitle("AEGIS FMCW through-rubble vital-sign radar: simulation", fontsize=14)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_sweep(losses, res, path):
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(losses, res["breath_c"], "o-", c="g", label="breathing, with compensation")
    ax.plot(losses, res["heart_c"], "s-", c="b", label="heartbeat, with compensation")
    ax.plot(losses, res["breath_r"], "o--", c="gray", label="breathing, no compensation")
    ax.plot(losses, res["heart_r"], "s--", c="silver", label="heartbeat, no compensation")
    ax.set_xlabel("one-way rubble loss (dB)")
    ax.set_ylabel("detection probability")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title("Detection vs rubble loss (Monte Carlo)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ sweep
def sweep(radar, losses, trials, platform_mm, seed):
    rng = np.random.default_rng(seed)
    out = {k: [] for k in ("breath_c", "heart_c", "breath_r", "heart_r")}
    for L in losses:
        cnt = dict.fromkeys(out, 0)
        for _ in range(trials):
            h = Human(range_m=rng.uniform(5.5, 9.5),
                      breath_bpm=rng.uniform(12, 20), breath_mm=rng.uniform(3, 6),
                      heart_bpm=rng.uniform(60, 90), heart_mm=rng.uniform(0.2, 0.5),
                      rubble_loss_db=L)
            iq, _, _ = simulate(radar, h, DEFAULT_CLUTTER, platform_mm, rng)
            for tag, comp in (("c", True), ("r", False)):
                r = run_chain(radar, iq, comp)
                in_bin = abs(r["sel_range"] - h.range_m) <= range_tol(radar)
                cnt["breath_" + tag] += int(in_bin and abs(r["breath_bpm"] - h.breath_bpm) < 1.5)
                cnt["heart_" + tag] += int(in_bin and abs(r["heart_bpm"] - h.heart_bpm) < 3.0)
        for k in out:
            out[k].append(cnt[k] / trials)
        print(f"loss {L:4.0f} dB | breath {out['breath_c'][-1]:.2f} heart {out['heart_c'][-1]:.2f}"
              f" | no-comp breath {out['breath_r'][-1]:.2f} heart {out['heart_r'][-1]:.2f}")
    return {k: np.array(v) for k, v in out.items()}


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loss", type=float, default=15.0, help="one-way rubble loss, dB")
    ap.add_argument("--platform", type=float, default=3.0, help="drone hover motion, mm rms")
    ap.add_argument("--range", type=float, default=7.0, help="human range, m")
    ap.add_argument("--bits", type=int, default=12, help="ADC bits (0 = ideal)")
    ap.add_argument("--bw", type=float, default=50.0, help="sweep bandwidth, MHz")
    ap.add_argument("--window", default="hann", choices=["hann", "cheb80"], help="range-FFT window")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--export", action="store_true", help="write int ADC codes of chirp 0 as CSV")
    ap.add_argument("--out", default="aegis_fmcw")
    args = ap.parse_args()

    radar = Radar(adc_bits=args.bits, bw=args.bw * 1e6, window=args.window)
    human = Human(range_m=args.range, rubble_loss_db=args.loss)
    rng = np.random.default_rng(args.seed)

    p_h = rx_power_w(radar, human.rcs_m2, human.range_m, human.rubble_loss_db)
    p_c = rx_power_w(radar, DEFAULT_CLUTTER[0].rcs_m2, DEFAULT_CLUTTER[0].range_m)
    p_n = noise_power_w(radar)
    print(f"range resolution      : {radar.range_res:.2f} m  (phase, not range, gives the vitals)")
    print(f"wavelength            : {radar.lam * 100:.1f} cm; 1 mm chest motion = "
          f"{np.degrees(4 * np.pi * 1e-3 / radar.lam):.1f} deg phase")
    print(f"human echo            : {dbm(p_h):7.1f} dBm  (after {2 * human.rubble_loss_db:.0f} dB two-way rubble loss)")
    print(f"rubble-face clutter   : {dbm(p_c):7.1f} dBm  ({dbm(p_c) - dbm(p_h):.0f} dB above the human)")
    print(f"thermal noise / sample: {dbm(p_n):7.1f} dBm")
    if radar.adc_bits:
        print(f"ADC                   : {radar.adc_bits} bit, ~{6.02 * radar.adc_bits + 1.76:.0f} dB SQNR")

    if args.sweep:
        losses = np.arange(0, 36, 5)
        res = sweep(radar, losses, args.trials, args.platform, args.seed)
        plot_sweep(losses, res, f"{args.out}_sweep.png")
        print(f"saved {args.out}_sweep.png")
        return

    iq, codes, truth = simulate(radar, human, DEFAULT_CLUTTER, args.platform, rng)
    a = run_chain(radar, iq, comp=True)
    b = run_chain(radar, iq, comp=False)

    ok_r = abs(a["sel_range"] - human.range_m) <= range_tol(radar)
    rows = [
        ("", "truth", "estimate"),
        ("range (m)", f"{human.range_m:.1f}", f"{a['sel_range']:.1f}" + ("" if ok_r else "  WRONG BIN")),
        ("breathing/min", f"{human.breath_bpm:.1f}", f"{a['breath_bpm']:.1f}"),
        ("heart bpm", f"{human.heart_bpm:.1f}", f"{a['heart_bpm']:.1f}"),
        ("", "", ""),
        ("rubble loss", f"{human.rubble_loss_db:.0f} dB one-way", ""),
        ("drone motion", f"{args.platform:.1f} mm rms", ""),
        ("no-comp bin", f"{b['sel_range']:.1f} m", "(wrong = fails)" if abs(b['sel_range'] - human.range_m) > 1.5 else ""),
    ]
    summary = "\n".join(f"{x:<15}{y:<18}{z}" for x, y, z in rows)
    print("\n" + summary)

    plot_run(radar, human, truth, a, b, summary, f"{args.out}_run.png")
    print(f"\nsaved {args.out}_run.png")

    if args.export and codes is not None:
        np.savetxt(f"{args.out}_chirp0_int{radar.adc_bits}.csv",
                   np.c_[codes[0][0], codes[1][0]], fmt="%d", delimiter=",", header="I,Q", comments="")
        print(f"saved {args.out}_chirp0_int{radar.adc_bits}.csv (golden vector for HDL testbench)")


if __name__ == "__main__":
    main()
