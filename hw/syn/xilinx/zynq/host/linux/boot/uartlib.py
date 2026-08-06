#!/usr/bin/env python3
# Robust UART helper for the Arty Z7 flows. A dedicated reader thread drains
# the port continuously into an in-process buffer, so the kernel tty buffer
# can never overflow while a caller is busy between reads — the root cause
# of the dropped/garbled chunks seen with poll-style readers on this FTDI
# (whose latency_timer is 16 ms and root-owned).
import os
import termios
import threading
import time

DEV = os.environ.get("UART_DEV", "/dev/ttyUSB1")


class Uart:
    def __init__(self, dev=DEV):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = 0; a[1] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0; a[4] = termios.B115200; a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._stop = False
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        import select
        while not self._stop:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if r:
                try:
                    chunk = os.read(self.fd, 65536)
                except (BlockingIOError, OSError):
                    continue
                if chunk:
                    with self._lock:
                        self._buf += chunk

    def close(self):
        self._stop = True
        self._t.join(timeout=1)
        os.close(self.fd)

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
        os.write(self.fd, data)

    def sendline(self, s: str, char_delay=0.002):
        for ch in s.encode():
            os.write(self.fd, bytes([ch]))
            if char_delay:
                time.sleep(char_delay)
        os.write(self.fd, b"\n")

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
