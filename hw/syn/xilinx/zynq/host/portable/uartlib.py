#!/usr/bin/env python3
# Cross-platform UART helper for the Vortex Zybo Z7-20 flows (Windows, Linux,
# macOS). Drop-in replacement for the termios-based host/linux/boot/uartlib.py:
# same Uart API, same hard-won framing logic, but the transport is pyserial so
# it works against a Windows COM port.
#
# A dedicated reader thread drains the port continuously into an in-process
# buffer, so the driver's receive buffer can never overflow while a caller is
# busy between reads — the root cause of the dropped/garbled chunks seen with
# poll-style readers on this FTDI (whose latency timer is 16 ms).
#
#   pip install pyserial
#
# Port selection, in order:
#   1. explicit dev= argument
#   2. $UART_DEV / %UART_DEV%   e.g. COM4, /dev/ttyUSB1
#   3. autodetect — the FT2232's second channel (UART); channel A is JTAG
import os
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    raise SystemExit(
        "pyserial is required: python -m pip install pyserial"
    )

BAUD = 115200

# The Zybo's FT2232HQ exposes two channels on one USB device:
#   interface 0 -> JTAG  (claimed by the Digilent/FTDI JTAG driver)
#   interface 1 -> UART  (this is the one we want)
# On Windows only the UART channel usually surfaces as a COM port, because
# the Digilent driver takes channel A. On Linux it is the higher ttyUSB*.
FTDI_VID = 0x0403
FT2232_PIDS = (0x6010,)


def find_port():
    """Best-effort autodetect of the board's UART channel."""
    explicit = os.environ.get("UART_DEV")
    if explicit:
        return explicit

    cands = []
    for p in list_ports.comports():
        if p.vid == FTDI_VID and p.pid in FT2232_PIDS:
            # location looks like "...-1.2:1.1" / "...:1.1"; trailing digit is
            # the USB interface number. Prefer interface 1 (the UART channel).
            loc = (p.location or "")
            iface = 1 if loc.endswith(".1") else 0 if loc.endswith(".0") else None
            cands.append((iface, p.device))

    if not cands:
        ports = ", ".join(p.device for p in list_ports.comports()) or "none"
        raise SystemExit(
            "could not find the Zybo UART. Serial ports present: " + ports +
            "\nSet UART_DEV to the right one (e.g. set UART_DEV=COM4)."
        )

    # interface 1 if we could identify it, else the last enumerated candidate
    for iface, dev in cands:
        if iface == 1:
            return dev
    return sorted(cands, key=lambda c: c[1])[-1][1]


class Uart:
    def __init__(self, dev=None, baud=BAUD):
        self.dev = dev or find_port()
        # timeout gives the reader thread a bounded blocking read; write_timeout
        # keeps a wedged board from hanging the orchestrator forever.
        self.ser = serial.Serial(self.dev, baud, timeout=0.05, write_timeout=5)
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._stop = False
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        while not self._stop:
            try:
                n = self.ser.in_waiting
                # read(1) blocks up to `timeout`, so this loop never spins hot
                chunk = self.ser.read(n if n else 1)
            except Exception:
                time.sleep(0.05)
                continue
            if chunk:
                with self._lock:
                    self._buf += chunk

    def close(self):
        self._stop = True
        self._t.join(timeout=1)
        try:
            self.ser.close()
        except Exception:
            pass

    # -- buffered reading --------------------------------------------------
    def take(self):
        """Return and clear everything received so far."""
        with self._lock:
            b = bytes(self._buf)
            self._buf.clear()
        return b

    def peek(self):
        with self._lock:
            return bytes(self._buf)

    def clear(self):
        with self._lock:
            self._buf.clear()

    def wait_for(self, patterns, timeout_s):
        """Wait until any pattern appears in the stream. Returns
        (matched_pattern_or_None, everything_received). Does not clear."""
        if isinstance(patterns, (bytes, str)):
            patterns = [patterns]
        pats = [p.encode() if isinstance(p, str) else p for p in patterns]
        end = time.time() + timeout_s
        while time.time() < end:
            data = self.peek()
            for p in pats:
                if p in data:
                    return p, data
            time.sleep(0.05)
        return None, self.peek()

    # -- writing -----------------------------------------------------------
    def send(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()

    def sendline(self, s: str, char_delay=0.002):
        # Byte-at-a-time with a small delay: the board's shell drops input if
        # fed faster, and the memcp octal delivery below depends on this.
        data = s.encode()
        if char_delay:
            for ch in data:
                self.ser.write(bytes([ch]))
                time.sleep(char_delay)
        else:
            self.ser.write(data)
        self.ser.write(b"\n")
        self.ser.flush()

    # -- conveniences ------------------------------------------------------
    def uboot_cmd(self, cmd, timeout_s=15, retries=2):
        """Run a command at the U-Boot prompt (Ctrl-C preamble for the BOOTP
        loop); returns output text up to the prompt."""
        for _ in range(retries):
            self.send(b"\x03")
            time.sleep(0.2)
            self.clear()
            self.sendline(cmd)
            p, data = self.wait_for(b"Zynq> ", timeout_s)
            if p:
                return self.take().decode("utf-8", "replace")
        raise RuntimeError(f"U-Boot command failed: {cmd}")

    def shell_cmd(self, cmd, timeout_s=15, sentinel="__RC__"):
        """Run a command at a Linux shell; append an echo sentinel so
        completion is detected reliably. Returns only the command's output
        (prompt fragments and the sentinel line stripped), so identical
        commands yield byte-identical captures."""
        # Wait until the shell is idle (stream ends at a prompt) before
        # clearing — otherwise a late-arriving prompt fragment from the
        # previous command leaks into this capture.
        end = time.time() + 2
        while time.time() < end and not self.peek().rstrip(b"\r\n").endswith(b"# "):
            time.sleep(0.05)
        self.clear()
        # split the sentinel in the command text so a tty echo of the command
        # can never match the sentinel we wait for
        half = len(sentinel) // 2
        self.sendline(f"{cmd}; echo {sentinel[:half]}\"\"{sentinel[half:]}$?")
        p, _ = self.wait_for(sentinel.encode(), timeout_s)
        out = self.take().decode("utf-8", "replace")
        if p is None:
            raise RuntimeError(f"shell command timed out: {cmd}\n{out[-500:]}")
        body = out.split(sentinel, 1)[0]
        lines = []
        for ln in body.splitlines():
            # drop stray prompt fragments (previous command's prompt racing
            # the buffer clear) and the echoed command line if echo is on
            if "# " in ln and ln.split("# ", 1)[1].strip() in ("", cmd + f"; echo {sentinel}$$?"):
                continue
            if ln.startswith(("root@", "$ ")):
                ln = ln.split("# ", 1)[-1]
                if not ln.strip():
                    continue
            lines.append(ln)
        return "\n".join(lines).strip("\n")


if __name__ == "__main__":
    # `python uartlib.py` — list candidate ports and show what autodetect picks
    print("serial ports:")
    for p in list_ports.comports():
        vid = f"{p.vid:04x}" if p.vid else "----"
        pid = f"{p.pid:04x}" if p.pid else "----"
        print(f"  {p.device:12} vid:pid={vid}:{pid}  loc={p.location}  {p.description}")
    try:
        print("\nautodetect ->", find_port())
    except SystemExit as e:
        print("\n" + str(e))
