"""Project files (.sasproj, JSON): everything typed in the UI plus the tuned controller
parameters. Results are recomputed on load (no tuning is repeated)."""
import json

import numpy as np

from core import autopilot as ap
from core.schedule import FlightPoint

VERSION = 1


def _default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    raise TypeError(f"not serialisable: {type(o)}")


def _settings_dict(st):
    return {"ref": list(np.atleast_1d(st.ref)), "T": st.T, "dt": st.dt, "prefilter": st.prefilter,
            "limits": list(st.limits), "rates": list(st.rates), "gm_spec": st.gm_spec, "pm_spec": st.pm_spec,
            "dm_spec": st.dm_spec, "x0_track": list(np.atleast_1d(st.x0_track)),
            "noise_std": list(np.atleast_1d(st.noise_std)), "noise_bw": st.noise_bw,
            "use_saturation": st.use_saturation, "antiwindup": st.antiwindup, "bank_limit": st.bank_limit}


def _settings_from(d):
    d = dict(d)
    d["limits"] = np.array(d["limits"], float)
    d["rates"] = np.array(d["rates"], float)
    return ap.APSettings(**d)


def collect(win):
    m, s, h, a, sc, ex = win.model_tab, win.sas_tab, win.hq_tab, win.ap_tab, win.sched_tab, win.export_tab
    p = win.project
    data = {"version": VERSION, "app": "SAS & Autopilot Designer"}
    data["model"] = {
        "name": m.name.text(), "cls": m.cls.currentText(), "cat": m.cat.currentText(),
        "V0": m.V0.value(), "mach": m.mach.value(), "alt": m.alt.value(), "theta0": m.theta0.value(),
        "g": m.gg.value(), "consts": m.consts.text(), "conv": m.conv.currentData(),
        "A_long": m.Along.toPlainText(), "B_long": m.Blong.toPlainText(), "in_long": m.in_long.text(),
        "A_lat": m.Alat.toPlainText(), "B_lat": m.Blat.toPlainText(), "in_lat": m.in_lat.text(),
        "act_tau": m.act_tau.value(), "act_delay": m.act_delay.value()}
    data["sas"] = {
        "l_input": s.l_input.currentIndex(), "l_struct": s.l_struct.currentIndex(),
        "wsp": s.wsp.value(), "zsp": s.zsp.value(), "wph": s.wph.value(), "zph": s.zph.value(),
        "lat_inputs": [c.isChecked() for c in s.lat_checks],
        "wdr": s.wdr.value(), "zdr": s.zdr.value(), "proll": s.proll.value(), "pspir": s.pspir.value(),
        "washout": s.washout.value(), "min_effort": s.min_eff.isChecked(), "sas_lim": s.sas_lim.value(),
        "long_designed": p.long_sas is not None, "lat_designed": p.lat_sas is not None}
    data["hq"] = {"bw1": h.bw1.value(), "tp1": h.tp1.value(), "bw2": h.bw2.value(), "tp2": h.tp2.value(),
                  "hold": h.hold.value(), "da": h.da.value(), "e_in": h.e_in.currentIndex(),
                  "a_in": h.a_in.currentIndex(), "evaluated": p.hq1797 is not None}
    data["autopilot"] = {"ui": a.state(), "entries": {}}
    for mode, e in p.ap.items():
        dg = e.get("digital")
        d0 = next(iter(dg.values())) if dg else None
        data["autopilot"]["entries"][mode] = {
            "input": int(e["plant"].j) if e["plant"].q == 1 else None,
            "settings": _settings_dict(e["settings"]),
            "params": {meth: c.params for meth, c in e["ctrls"].items()},
            "digital": {"Ts": d0["Ts"], "method": d0["method"], "delay": d0["delay"]} if d0 else None}
    data["schedule"] = {
        "points": [{"mach": pt.mach, "V0": pt.V0, "A_long": pt.A_long, "B_long": pt.B_long,
                    "A_lat": pt.A_lat, "B_lat": pt.B_lat} for pt in p.schedule_points],
        "ap_mode": sc.ap_mode.currentIndex(), "conv": sc.conv.currentData(), "computed": p.schedule is not None}
    data["export"] = {"folder": ex.folder.text(), "title": ex.title.text()}
    data["robust"] = win.robust_tab.state()
    data["flight_controller"] = win.fc_tab.state()
    if p.aero:
        data["aero"] = p.aero
    return data


def save(win, path):
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(collect(win), fh, indent=1, default=_default)


def load_file(path):
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    if data.get("app") != "SAS & Autopilot Designer":
        raise ValueError("Not a SAS & Autopilot Designer project file.")
    return data


def apply_sync(win, data):
    """Fill the UI and redo the quick steps (parse, SAS, HQ). Returns the jobs for the
    background part (autopilot rebuild + schedule)."""
    m, s, h, a, sc, ex = win.model_tab, win.sas_tab, win.hq_tab, win.ap_tab, win.sched_tab, win.export_tab
    md = data["model"]
    m.name.setText(md["name"]); m.cls.setCurrentText(md["cls"]); m.cat.setCurrentText(md["cat"])
    m.V0.setValue(md["V0"]); m.mach.setValue(md["mach"]); m.alt.setValue(md["alt"])
    m.theta0.setValue(md["theta0"]); m.gg.setValue(md["g"]); m.consts.setText(md["consts"])
    m.conv.setCurrentIndex(max(m.conv.findData(md["conv"]), 0))
    m.Along.setPlainText(md["A_long"]); m.Blong.setPlainText(md["B_long"]); m.in_long.setText(md["in_long"])
    m.Alat.setPlainText(md["A_lat"]); m.Blat.setPlainText(md["B_lat"]); m.in_lat.setText(md["in_lat"])
    m.act_tau.setValue(md.get("act_tau", 0.0)); m.act_delay.setValue(md.get("act_delay", 0.0))
    win.project.aero = data.get("aero")
    ex.folder.setText(data.get("export", {}).get("folder", ex.folder.text()))
    ex.title.setText(data.get("export", {}).get("title", ""))
    win.project.schedule_points = [
        FlightPoint(pt["mach"], pt["V0"], np.array(pt["A_long"]), np.array(pt["B_long"]),
                    np.array(pt["A_lat"]) if pt["A_lat"] is not None else None,
                    np.array(pt["B_lat"]) if pt["B_lat"] is not None else None)
        for pt in data.get("schedule", {}).get("points", [])]
    sc.refresh()
    sc.ap_mode.setCurrentIndex(data.get("schedule", {}).get("ap_mode", 0))
    sc.conv.setCurrentIndex(max(sc.conv.findData(data.get("schedule", {}).get("conv", "uwqt")), 0))
    if not (md["A_long"].strip() or md["A_lat"].strip()):
        return None
    if not m.analyse():
        return None
    sd = data["sas"]
    s.min_eff.setChecked(sd.get("min_effort", True)); s.sas_lim.setValue(sd.get("sas_lim", 30.0))
    if "robust" in data:
        win.robust_tab.set_state(data["robust"])
    if "flight_controller" in data:
        win.fc_tab.set_state(data["flight_controller"])
    if win.project.long_model is not None:
        s.l_input.setCurrentIndex(sd["l_input"]); s.l_struct.setCurrentIndex(sd["l_struct"])
        s.wsp.setValue(sd["wsp"]); s.zsp.setValue(sd["zsp"]); s.wph.setValue(sd["wph"]); s.zph.setValue(sd["zph"])
        if sd["long_designed"]:
            s.design_long()
    if win.project.lat_model is not None:
        for c, v in zip(s.lat_checks, sd["lat_inputs"]):
            c.setChecked(v)
        s.wdr.setValue(sd["wdr"]); s.zdr.setValue(sd["zdr"]); s.proll.setValue(sd["proll"])
        s.pspir.setValue(sd["pspir"]); s.washout.setValue(sd.get("washout", 0.0))
        if sd["lat_designed"]:
            s.design_lat()
    hd = data.get("hq", {})
    if hd:
        h.refresh_inputs()
        h.bw1.setValue(hd["bw1"]); h.tp1.setValue(hd["tp1"]); h.bw2.setValue(hd["bw2"]); h.tp2.setValue(hd["tp2"])
        h.hold.setValue(hd["hold"]); h.da.setValue(hd["da"])
        h.e_in.setCurrentIndex(max(hd.get("e_in", 0), 0)); h.a_in.setCurrentIndex(max(hd.get("a_in", 0), 0))
        if hd.get("evaluated"):
            h.evaluate()
    a.set_state(data["autopilot"]["ui"])
    jobs = []
    p = win.project
    for mode, ed in data["autopilot"]["entries"].items():
        try:
            plant = ap.build_plant(mode, p.long_model, p.long_sas, p.lat_model, p.lat_sas, ed["input"], p.actuator)
        except ap.APError:
            continue
        jobs.append((plant, _settings_from(ed["settings"]), ed["params"], mode, ed.get("digital")))
    return {"ap": jobs, "schedule": data.get("schedule", {}).get("computed", False)}
