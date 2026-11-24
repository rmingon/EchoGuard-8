#!/usr/bin/env python3
"""Minimal RTT terminal for J-Link probes whose RTT tools refuse to work (e.g. J-Link OB clones).

Drives a single JLinkExe session, polls the firmware's RTT up-buffer 0 over SWD (without
halting the CPU) and prints it to stdout.

Usage:  python3 tools/rtt_read.py [--seconds N] [--elf path/to/firmware.elf]
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

DEFAULT_ELF = ".pio/build/stm32f103c8_freertos/firmware.elf"
NM = os.path.expanduser("~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm")
SENTINEL_ADDR = 0x08000000  # start of flash: always readable, never inside RTT RAM
MEM_LINE = re.compile(r"^([0-9A-F]{8}) = ((?:[0-9A-F]{2} ?)+)")


def find_control_block(elf):
    out = subprocess.run([NM, elf], capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == "_SEGGER_RTT":
            return int(parts[0], 16)
    sys.exit(f"_SEGGER_RTT not found in {elf}")


class JLink:
    def __init__(self, device):
        exe = shutil.which("JLinkExe")
        if exe is None:
            sys.exit("JLinkExe not found in PATH")
        self.proc = subprocess.Popen(
            [exe, "-device", device, "-if", "SWD", "-speed", "4000", "-autoconnect", "1", "-NoGui", "1"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def send(self, cmd):
        self.proc.stdin.write(cmd + "\n")
        self.proc.stdin.flush()

    def command(self, cmd):
        """Run a command and return its output lines.

        JLinkExe gives no end-of-output marker (and mem8 may print more bytes than asked), so
        a sentinel read of a fixed flash word is sent after it to know where the output ends.
        """
        self.send(cmd)
        self.send(f"mem32 0x{SENTINEL_ADDR:08X} 1")
        lines = []
        while True:
            line = self.proc.stdout.readline()
            if not line:
                sys.exit("JLinkExe exited")
            line = line.strip()
            if line.startswith(f"{SENTINEL_ADDR:08X} = "):
                return lines
            lines.append(line)

    def read_mem(self, addr, length):
        data = bytearray(length)
        got = 0
        for line in self.command(f"mem8 0x{addr:08X} {length}"):
            m = MEM_LINE.match(line)
            if not m:
                continue
            line_addr = int(m.group(1), 16)
            chunk = bytes.fromhex(m.group(2).replace(" ", ""))
            for i, b in enumerate(chunk):
                off = line_addr - addr + i
                if 0 <= off < length:
                    data[off] = b
                    got += 1
        if got < length:
            raise IOError(f"short read at 0x{addr:08X}: {got}/{length}")
        return bytes(data)

    def write_u32(self, addr, value):
        self.command(f"w4 0x{addr:08X} 0x{value:08X}")

    def close(self):
        try:
            self.send("q")
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()


def u32(buf, off):
    return int.from_bytes(buf[off:off + 4], "little")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=DEFAULT_ELF)
    ap.add_argument("--device", default="STM32F103C8")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds (0 = forever)")
    args = ap.parse_args()

    cb = find_control_block(args.elf)
    up = cb + 24  # id[16] + max_up + max_down
    jl = JLink(args.device)
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                desc = jl.read_mem(up, 24)
            except IOError:
                time.sleep(0.2)  # target busy / resetting
                continue
            buf, size, wr, rd = u32(desc, 4), u32(desc, 8), u32(desc, 12), u32(desc, 16)
            if size == 0 or wr >= size or rd >= size:
                time.sleep(0.2)  # firmware not initialised yet
                continue
            if wr != rd:
                if wr > rd:
                    data = jl.read_mem(buf + rd, wr - rd)
                else:
                    data = jl.read_mem(buf + rd, size - rd) + (jl.read_mem(buf, wr) if wr else b"")
                jl.write_u32(up + 16, wr)
                sys.stdout.write(data.decode("ascii", "replace"))
                sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        jl.close()


if __name__ == "__main__":
    main()
