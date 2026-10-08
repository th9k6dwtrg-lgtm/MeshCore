#!/usr/bin/env python3
"""Nastavení radaru HLK-LD2410S přes UART (115200 Bd) z Macu / Linuxu / Windows.

Podle "HLK-LD2410S serial communication protocol V1.00" (Hi-Link).
Potřebuje jen pyserial:  python3 -m pip install --user pyserial

Použití (PORT je např. /dev/cu.usbmodem5A7A0123451 nebo /dev/cu.wchusbserial...):
  python3 ld2410s.py PORT info                 verze, parametry, prahy + záloha do JSON
  python3 ld2410s.py PORT sleduj [sekund]      živě stav, vzdálenost a energie v branách
  python3 ld2410s.py PORT param far=8 near=0 delay=30 speed=normal status_hz=0.5 dist_hz=0.5
  python3 ld2410s.py PORT trigger 0=50 1=46 ... | all=40      prahy pro sepnutí (bez osoby -> osoba)
  python3 ld2410s.py PORT hold    0=15 1=15 ... | all=12      prahy pro udržení (osoba trvá)
  python3 ld2410s.py PORT obnov ZALOHA.json    vrátí parametry a prahy ze zálohy
  python3 ld2410s.py PORT auto [sken_s] [trigger_faktor] [hold_faktor]   automatické prahy (start za 30 s, prázdná místnost!)
"""
import json
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Chybí pyserial. Nainstaluj:  python3 -m pip install --user pyserial")

BAUD = 115200
CMD_HEAD = bytes.fromhex("FDFCFBFA")
CMD_TAIL = bytes.fromhex("04030201")
DATA_HEAD = bytes.fromhex("F4F3F2F1")
DATA_TAIL = bytes.fromhex("F8F7F6F5")
GATES = 16

# id parametru: (název, převod uživatel -> hodnota, převod hodnota -> text)
PARAMS = {
    "far": (0x05, "nejvzdálenější brána (1–16)"),
    "near": (0x0A, "nejbližší brána (0–16)"),
    "delay": (0x06, "doba držení bez osoby, s (10–120)"),
    "status_hz": (0x02, "frekvence hlášení stavu, Hz (0,5–8)"),
    "dist_hz": (0x0C, "frekvence hlášení vzdálenosti, Hz (0,5–8)"),
    "speed": (0x0B, "rychlost reakce (normal / fast)"),
}
PARAM_BY_ID = {v[0]: k for k, v in PARAMS.items()}


def to_raw(name, text):
    if name in ("status_hz", "dist_hz"):
        hz = float(text.replace(",", "."))
        if not (0.5 <= hz <= 8) or (hz * 2) != int(hz * 2):
            raise ValueError(f"{name}: 0.5 až 8 po 0.5")
        return int(round(hz * 10))
    if name == "speed":
        return {"normal": 5, "fast": 10, "5": 5, "10": 10}[text.lower()]
    val = int(text)
    lim = {"far": (1, 16), "near": (0, 16), "delay": (10, 120)}[name]
    if not lim[0] <= val <= lim[1]:
        raise ValueError(f"{name}: povolený rozsah {lim[0]}–{lim[1]}")
    return val


def from_raw(name, raw):
    if name in ("status_hz", "dist_hz"):
        return f"{raw / 10:g} Hz"
    if name == "speed":
        return {5: "normal", 10: "fast"}.get(raw, str(raw))
    if name == "delay":
        return f"{raw} s"
    return str(raw)


def u16(v):
    return v.to_bytes(2, "little")


def u32(v):
    return v.to_bytes(4, "little")


class Radar:
    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, timeout=0.1)
        self.buf = bytearray()

    def _read(self):
        self.buf += self.s.read(256)

    def command(self, cmd, data=b"", timeout=1.5):
        body = u16(cmd) + data
        frame = CMD_HEAD + u16(len(body)) + body + CMD_TAIL
        for _ in range(3):
            self.buf.clear()
            self.s.reset_input_buffer()
            self.s.write(frame)
            end = time.time() + timeout
            while time.time() < end:
                self._read()
                i = self.buf.find(CMD_HEAD)
                if i < 0 or len(self.buf) < i + 6:
                    continue
                ln = int.from_bytes(self.buf[i + 4:i + 6], "little")
                if len(self.buf) < i + 6 + ln + 4:
                    continue
                payload = bytes(self.buf[i + 6:i + 6 + ln])
                del self.buf[:i + 6 + ln + 4]
                if int.from_bytes(payload[:2], "little") != (cmd | 0x0100):
                    continue
                status = int.from_bytes(payload[2:4], "little") if cmd != 0x0000 else 0
                if status != 0:
                    raise RuntimeError(f"radar odmítl příkaz 0x{cmd:04X} (stav {status})")
                return payload[4:] if cmd != 0x0000 else payload[2:]
        raise RuntimeError(f"radar neodpověděl na příkaz 0x{cmd:04X} "
                           "(zkontroluj TX/RX prohozené, 3,3 V a GND, správný port)")

    def config_on(self):
        self.command(0x00FF, u16(0x0001))

    def config_off(self):
        self.command(0x00FE)

    def version(self):
        p = self.command(0x0000)
        if len(p) >= 6:
            parts = [int.from_bytes(p[i:i + 2], "little") for i in range(0, 6, 2)]
            return "{}.{}.{}".format(*parts)
        return p.hex(" ")

    def read_params(self):
        ids = [v[0] for v in PARAMS.values()]
        p = self.command(0x0071, b"".join(u16(i) for i in ids))
        out = {}
        if len(p) >= 4 * len(ids):  # hodnoty po 4 B ve stejném pořadí jako dotaz
            for n, pid in enumerate(ids):
                out[PARAM_BY_ID[pid]] = int.from_bytes(p[4 * n:4 * n + 4], "little")
        else:
            out["_raw"] = p.hex(" ")
        return out

    def write_params(self, values):
        data = b"".join(u16(PARAMS[k][0]) + u32(v) for k, v in values.items())
        self.command(0x0070, data)

    def read_thresholds(self, cmd):
        p = self.command(cmd, b"".join(u16(g) for g in range(GATES)))
        return [int.from_bytes(p[4 * g:4 * g + 4], "little") for g in range(GATES)]

    def write_thresholds(self, cmd, values):
        data = b"".join(u16(g) + u32(v) for g, v in sorted(values.items()))
        self.command(cmd, data)

    def output_mode(self, standard):
        self.command(0x007A, bytes([0, 0, 1 if standard else 0, 0, 0, 0]))

    def frames(self, seconds):
        """Vrací (typ, data) z datových rámců až do vypršení času."""
        end = time.time() + seconds if seconds else None
        self.buf.clear()
        while end is None or time.time() < end:
            self._read()
            while True:
                i = self.buf.find(DATA_HEAD)
                if i < 0:
                    # minimální rámec 6E st dist(2) 62
                    j = self.buf.find(b"\x6e")
                    if j >= 0 and len(self.buf) >= j + 5 and self.buf[j + 4] == 0x62:
                        fr = bytes(self.buf[j:j + 5])
                        del self.buf[:j + 5]
                        yield "min", fr
                        continue
                    if len(self.buf) > 512:
                        del self.buf[:-8]
                    break
                if len(self.buf) < i + 6:
                    break
                ln = int.from_bytes(self.buf[i + 4:i + 6], "little")
                if ln > 200:
                    del self.buf[:i + 4]
                    continue
                if len(self.buf) < i + 6 + ln + 4:
                    break
                data = bytes(self.buf[i + 6:i + 6 + ln])
                del self.buf[:i + 6 + ln + 4]
                yield "std", data


AUTO_START_DELAY = 30  # s na odchod z místnosti před začátkem skenu
AUTO_WAIT_MAX = 600    # s, jak dlouho po skenu čekat, než radar začne odpovídat

STATE = {0: "nikdo", 1: "nikdo", 2: "PŘÍTOMNOST", 3: "PŘÍTOMNOST"}


def print_info(r, save=True):
    r.config_on()
    try:
        ver = r.version()
        params = r.read_params()
        trig = r.read_thresholds(0x0073)
        hold = r.read_thresholds(0x0077)
    finally:
        r.config_off()
    print(f"Firmware radaru: {ver}\n")
    print("Obecné parametry:")
    for k, v in params.items():
        if k in PARAMS:
            print(f"  {k:<10} = {from_raw(k, v):<8}  {PARAMS[k][1]}")
        else:
            print(f"  (nečitelná odpověď: {v})")
    print("\nPrahy po branách (vyšší číslo = méně citlivé):")
    print("  brána   " + " ".join(f"{g:>3}" for g in range(GATES)))
    print("  trigger " + " ".join(f"{v:>3}" for v in trig))
    print("  hold    " + " ".join(f"{v:>3}" for v in hold))
    if save:
        name = time.strftime("ld2410s-zaloha-%Y%m%d-%H%M%S.json")
        with open(name, "w") as f:
            json.dump({"params": {k: v for k, v in params.items() if k in PARAMS},
                       "trigger": trig, "hold": hold}, f, indent=1)
        print(f"\nZáloha uložena do {name}")


def watch(r, seconds):
    r.config_on()
    r.output_mode(True)
    r.config_off()
    peak = [0] * GATES
    print("Sleduji (Ctrl+C ukončí). Energie v branách 0–15:")
    try:
        for kind, data in r.frames(seconds):
            if kind != "std" or not data or data[0] != 0x01 or len(data) < 6 + 64:
                if kind == "std" and data and data[0] == 0x03:
                    print("auto prahy:", data.hex(" "))
                continue
            st = data[1]
            dist = int.from_bytes(data[2:4], "little")
            en = [int.from_bytes(data[6 + 4 * g:10 + 4 * g], "little") for g in range(GATES)]
            peak = [max(a, b) for a, b in zip(peak, en)]
            print(f"{time.strftime('%H:%M:%S')} {STATE.get(st, st):<11} {dist:>4} cm | "
                  + " ".join(f"{e:>3}" for e in en), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            r.config_on()
            r.output_mode(False)
            r.config_off()
        except Exception as e:  # radar zůstane ve standardním výstupu, OT2 to nevadí
            print("Varování: nepodařilo se vrátit minimální výstup:", e)
    print("\nMaximum energie za dobu sledování:")
    print("  brána " + " ".join(f"{g:>3}" for g in range(GATES)))
    print("  max   " + " ".join(f"{v:>3}" for v in peak))


def parse_gates(args):
    vals = {}
    for a in args:
        k, v = a.split("=")
        v = int(v)
        if not 0 <= v <= 100:
            raise ValueError("práh má být rozumné číslo 0–100")
        if k == "all":
            vals.update({g: v for g in range(GATES)})
        else:
            g = int(k)
            if not 0 <= g < GATES:
                raise ValueError("brána 0–15")
            vals[g] = v
    if not vals:
        raise ValueError("chybí hodnoty, např. all=40 nebo 0=50 1=46")
    return vals


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    port, action, args = sys.argv[1], sys.argv[2], sys.argv[3:]
    r = Radar(port)
    if action == "info":
        print_info(r)
    elif action == "sleduj":
        watch(r, float(args[0]) if args else 0)
    elif action == "param":
        vals = {}
        for a in args:
            k, v = a.split("=")
            if k not in PARAMS:
                sys.exit(f"Neznámý parametr {k}. Možné: {', '.join(PARAMS)}")
            vals[k] = to_raw(k, v)
        r.config_on()
        try:
            r.write_params(vals)
        finally:
            r.config_off()
        print("Zapsáno. Aktuální stav:\n")
        print_info(r, save=False)
    elif action in ("trigger", "hold"):
        vals = parse_gates(args)
        r.config_on()
        try:
            r.write_thresholds(0x0072 if action == "trigger" else 0x0076, vals)
        finally:
            r.config_off()
        print("Zapsáno. Aktuální stav:\n")
        print_info(r, save=False)
    elif action == "obnov":
        with open(args[0]) as f:
            z = json.load(f)
        r.config_on()
        try:
            r.write_params(z["params"])
            r.write_thresholds(0x0072, dict(enumerate(z["trigger"])))
            r.write_thresholds(0x0076, dict(enumerate(z["hold"])))
        finally:
            r.config_off()
        print("Obnoveno ze zálohy. Aktuální stav:\n")
        print_info(r, save=False)
    elif action == "auto":
        scan = int(args[0]) if len(args) > 0 else 120
        tf = int(args[1]) if len(args) > 1 else 2
        hf = int(args[2]) if len(args) > 2 else 1
        r.config_on()
        try:
            before = r.read_params()
        finally:
            r.config_off()
        print(f"Sken začne za {AUTO_START_DELAY} s. Odejdi z dosahu radaru "
              "a vrať se až po výpisu prahů.", flush=True)
        for left in range(AUTO_START_DELAY, 0, -5):
            print(f"  start za {left} s", flush=True)
            time.sleep(5)
        r.config_on()
        r.command(0x0009, u16(tf) + u16(hf) + u16(scan))
        r.config_off()
        print(f"Automatické prahy běží {scan} s.", flush=True)
        last = -1
        for kind, data in r.frames(scan + 30):
            if kind == "std" and data and data[0] == 0x03:
                prog = int.from_bytes(data[-2:], "little")
                if prog // 10 != last // 10:
                    print(f"  průběh: {prog} %   ({time.strftime('%H:%M:%S')})", flush=True)
                last = prog
                if prog >= 100:
                    break
        # po 100 % radar nějakou dobu (nebo až do odpojení napájení) neodpovídá na příkazy;
        # zkoušíme to až AUTO_WAIT_MAX s a vypisujeme, co mezitím posílá
        print(f"\nKonec skenu. Zkouším každých 10 s, jestli radar odpovídá "
              f"(nejvýš {AUTO_WAIT_MAX // 60} min). Do místnosti už můžeš.", flush=True)
        t0 = time.time()
        while True:
            seen = {}
            for kind, data in r.frames(10):
                if kind == "min":
                    key = "stav"
                elif data and data[0] == 0x03:
                    key = f"průběh {int.from_bytes(data[-2:], 'little')} %"
                else:
                    key = "jiná data"
                seen[key] = seen.get(key, 0) + 1
            el = int(time.time() - t0)
            try:
                r.config_on()
                r.config_off()
                print(f"  radar odpovídá {el} s po konci skenu", flush=True)
                break
            except RuntimeError:
                pass
            got = ", ".join(f"{n}× {k}" for k, n in seen.items()) or "nic"
            print(f"  {el:>3} s: neodpovídá, posílá: {got}", flush=True)
            if el >= AUTO_WAIT_MAX:
                raise RuntimeError(f"radar ani po {AUTO_WAIT_MAX // 60} min neodpovídá. Odpoj "
                                   "programátor z USB, připoj znovu a spusť info")
        # kalibrace umí přepsat i obecné parametry (doba držení 10 -> 40 s), vrátíme je
        r.config_on()
        try:
            after = r.read_params()
            changed = {k: v for k, v in before.items() if k in PARAMS and after.get(k) != v}
            if changed:
                r.write_params(changed)
                for k, v in changed.items():
                    print(f"  kalibrace změnila {k} z {from_raw(k, v)} na "
                          f"{from_raw(k, after[k])}, vráceno zpět", flush=True)
        finally:
            r.config_off()
        print()
        print_info(r, save=False)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, serial.SerialException) as e:
        sys.exit(f"Chyba: {e}")
