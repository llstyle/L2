"""
Графики "исходный / обработанный сигнал" по логу из Serial Monitor Wokwi (версия 2).

1) В Wokwi выдели весь текст Serial Monitor (Ctrl+A, Ctrl+C) и вставь в файл,
   например rest.csv (строки, начинающиеся с '#', пропускаются автоматически).
2) pip install matplotlib
3) python plot_log.py rest.csv        -> rest_x.png + статистика для таблицы отчёта
"""
import sys, os, csv, statistics as st
import matplotlib.pyplot as plt

LSB_PER_G, G = 4096.0, 9.80665          # диапазон ±8 g
to_ms2 = lambda v: v / LSB_PER_G * G


def load(path):
    with open(path, encoding="utf-8", errors="ignore") as f:
        lines = [l for l in f if l.strip() and not l.startswith("#")]
    rows = []
    for r in csv.DictReader(lines):
        try:
            rows.append({k: float(v) for k, v in r.items()})
        except (ValueError, TypeError):
            pass
    return rows


def sd(x): return st.pstdev(x) if len(x) > 1 else 0.0


def events(t, flags):
    """Начала событий: моменты, когда флаг меняется 0 -> 1."""
    out, prev = [], 0
    for ti, f in zip(t, flags):
        if f and not prev:
            out.append(ti)
        prev = f
    return out


def main(path, axis="x"):
    d = load(path)
    if not d:
        sys.exit("нет данных в " + path)
    t0 = d[0]["t_ms"]
    t = [(r["t_ms"] - t0) / 1000 for r in d]
    raw = [to_ms2(r[f"a{axis}_raw"]) for r in d]
    flt = [to_ms2(r[f"a{axis}_f"]) for r in d]
    ok = [r["ok"] for r in d]
    dev = [r["dev"] for r in d]
    pk = [r["dev_pk"] for r in d]
    sh = [r["shock"] for r in d]
    tam = [r["tamper"] for r in d]
    mot = [r.get("mot", 0) for r in d]

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True,
                                 gridspec_kw={"height_ratios": [3, 2]})
    a1.plot(t, raw, color="#b0b0b0", lw=0.8, label=f"a{axis} исходный (raw)")
    a1.plot(t, flt, color="#d62728", lw=1.6, label=f"a{axis} после обработки")
    bad = [ti for ti, o in zip(t, ok) if o == 0]
    if bad:
        a1.plot(bad, [0] * len(bad), "x", color="black", ms=5, label="сбой чтения (отброшен)")
    a1.set_ylabel("ускорение, м/с²"); a1.grid(alpha=.3); a1.legend(loc="upper right")
    a1.set_title(f"Ось {axis.upper()}: исходный и обработанный сигнал — {os.path.basename(path)}")

    a2.plot(t, pk, color="#ff7f0e", lw=0.9, label="dev_pk — канал удара, м/с²")
    a2.plot(t, dev, color="#1f77b4", lw=1.6, label="dev — медленный канал, м/с²")
    top = max(max(dev), max(pk))
    a2.fill_between(t, 0, [top * v for v in tam], color="#d62728", alpha=.12,
                    step="mid", label="tamper = 1")
    a2.axhline(2.0, color="#1f77b4", ls="--", lw=0.8)
    a2.axhline(4.9, color="#ff7f0e", ls="--", lw=0.8)
    a2.set_ylabel("м/с²"); a2.set_xlabel("время, с"); a2.grid(alpha=.3)
    a2.legend(loc="upper right")
    fig.tight_layout()
    out = os.path.splitext(path)[0] + f"_{axis}.png"
    fig.savefig(out, dpi=150)

    dt = [r["dt_us"] for r in d[1:]]
    print(f"График: {out}")
    print(f"Отсчётов: {len(d)}, длительность {t[-1]:.1f} с")
    print(f"Ось {axis.upper()}: СКО raw = {sd(raw):.3f} м/с², размах {max(raw)-min(raw):.2f}")
    print(f"Ось {axis.upper()}: СКО обработ. = {sd(flt):.3f} м/с², размах {max(flt)-min(flt):.2f}")
    print(f"Отброшено сбоев чтения: {sum(1 for o in ok if o == 0)}, насыщений: {sum(1 for o in ok if o == 2)}")
    print(f"Макс. dev = {max(dev):.2f} м/с², макс. dev_pk = {max(pk):.2f} м/с²")
    print(f"Удары (shock): {int(sum(sh))} отсчётов, начала: {', '.join(f'{x:.2f}' for x in events(t, sh)) or '—'} с")
    print(f"Срабатываний tamper: {int(sum(tam))} отсчётов, начала: {', '.join(f'{x:.2f}' for x in events(t, tam)) or '—'} с")
    print(f"Детектор движения MPU6050 (mot): {int(sum(mot))} отсчётов")
    if dt:
        print(f"Период: среднее {st.mean(dt):.0f} мкс, мин {min(dt):.0f}, макс {max(dt):.0f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "x")
