#!/usr/bin/env python3
"""
test_radio.py - Host-side smoke test for the at-os3 CH32V003 AT LoRa modem.

Examples:
    python3 tools/test_radio.py /dev/ttyACM0 --probe
    python3 tools/test_radio.py /dev/ttyACM0 --freq 436995000 --sf 8 --bw 62.5 --cr 7 --rx-seconds 120
    python3 tools/test_radio.py /dev/ttyACM0 --freq 436995000 --sf 8 --bw 62.5 --cr 7 --send-text ping
    python3 tools/test_radio.py /dev/ttyACM0 --freq 436995000 --sf 8 --bw 62.5 --cr 7 --send 70696E67
"""
from __future__ import annotations

import argparse
import logging
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable

try:
    import serial
except ImportError as exc:
    print("Missing dependency: pyserial. Install with: python3 -m pip install pyserial", file=sys.stderr)
    raise SystemExit(1) from exc


LOG = logging.getLogger("at-os3.test_radio")

BW_TO_CODE: dict[float, int] = {
    7.8: 0,
    10.4: 1,
    15.6: 2,
    20.8: 3,
    31.25: 4,
    41.7: 5,
    62.5: 6,
    125.0: 7,
    250.0: 8,
    500.0: 9,
}

BW_KHZ = [7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0]


@dataclass
class ReceivedPacket:
    addr: int
    data: bytes
    rssi: int
    snr: int
    frequency_error: int = 0


def parse_int(value: str) -> int:
    return int(value, 0)


def bw_to_code(bw_khz: float) -> int:
    nearest = min(BW_TO_CODE.keys(), key=lambda item: abs(item - bw_khz))
    return BW_TO_CODE[nearest]


class Radio:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.on_packet: Callable[[ReceivedPacket], None] | None = None
        self._ser: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._resp: queue.Queue[str] = queue.Queue()
        self._running = False

    def start(self) -> None:
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        LOG.info("radio started on %s @ %d baud", self.port, self.baud)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        LOG.info("radio stopped")

    def ping(self) -> bool:
        return self._cmd("AT")

    def read_reg(self, addr: int) -> int | None:
        resp = self._cmd_get(f"AT+REG={addr:02X}", "+REG=")
        if resp is None:
            return None
        try:
            return int(resp, 16)
        except ValueError:
            return None

    def probe(self) -> int | None:
        resp = self._cmd_get("AT+PROBE?", "+PROBE=")
        if resp is None:
            return None
        try:
            return int(resp, 16)
        except ValueError:
            return None

    def query_band(self) -> int | None:
        resp = self._cmd_get("AT+BAND?", "+BAND=")
        return int(resp) if resp else None

    def query_parameter(self) -> tuple[int, int, int, int] | None:
        resp = self._cmd_get("AT+PARAMETER?", "+PARAMETER=")
        if not resp:
            return None
        parts = resp.split(",")
        if len(parts) < 4:
            return None
        return tuple(int(part) for part in parts[:4])  # type: ignore[return-value]

    def query_iq_inverted(self) -> bool | None:
        resp = self._cmd_get("AT+IQI?", "+IQI=")
        if resp is None:
            return None
        return bool(int(resp))

    def configure(
        self,
        *,
        freq_hz: int,
        sf: int,
        bw_khz: float,
        cr: int,
        preamble: int,
        power: int,
        sync_word: int,
        inverted_iq: bool,
        crc: bool,
        fldro: int,
        implicit_len: int,
    ) -> bool:
        bw_code = bw_to_code(bw_khz)
        cr_code = cr - 4
        return all(
            (
                self._cmd("AT+MODE=1"),
                self._cmd(f"AT+BAND={freq_hz}"),
                self._cmd(f"AT+PARAMETER={sf},{bw_code},{cr_code},{preamble}"),
                self._cmd(f"AT+PKT={1 if crc else 0},{max(0, min(2, fldro))},{max(0, min(255, implicit_len))}"),
                self._cmd(f"AT+SYNCWORD={sync_word & 0xFF}"),
                self._cmd(f"AT+IQI={1 if inverted_iq else 0}"),
                self._cmd(f"AT+CRFOP={power}"),
            )
        )

    def set_rx(self) -> bool:
        return self._cmd("AT+MODE=0")

    def set_standby(self) -> bool:
        return self._cmd("AT+MODE=1")

    def send(self, data: bytes, addr: int = 0) -> bool:
        return self._cmd(f"AT+SEND={addr},{len(data)},{data.hex().upper()}")

    def _write(self, cmd: str) -> None:
        assert self._ser is not None
        self._ser.write((cmd + "\r\n").encode("ascii"))
        LOG.debug("> %s", cmd)

    def _drain_responses(self) -> None:
        try:
            while True:
                self._resp.get_nowait()
        except queue.Empty:
            pass

    def _cmd(self, cmd: str) -> bool:
        self._drain_responses()
        self._write(cmd)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                line = self._resp.get(timeout=0.1)
            except queue.Empty:
                continue
            LOG.debug("< %s", line)
            if line == "+OK":
                return True
            if line.startswith("+ERR"):
                LOG.warning("%s -> %s", cmd, line)
                return False
        LOG.warning("%s -> timeout", cmd)
        return False

    def _cmd_get(self, cmd: str, prefix: str) -> str | None:
        self._drain_responses()
        self._write(cmd)
        result: str | None = None
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                line = self._resp.get(timeout=0.1)
            except queue.Empty:
                continue
            LOG.debug("< %s", line)
            if line.startswith(prefix):
                result = line[len(prefix):]
            elif line == "+OK":
                break
            elif line.startswith("+ERR"):
                return None
        return result

    def _reader_loop(self) -> None:
        assert self._ser is not None
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(256)
            except serial.SerialException:
                time.sleep(0.01)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("+RCV="):
                    self._parse_rcv(line[5:])
                elif line == "+SENDED":
                    LOG.info("TX confirmed (+SENDED)")
                elif line == "+ERR=1":
                    LOG.info("RX CRC error (+ERR=1)")
                else:
                    self._resp.put(line)

    def _parse_rcv(self, body: str) -> None:
        parts = body.split(",")
        if len(parts) < 5:
            LOG.warning("malformed +RCV: %s", body)
            return
        try:
            addr = int(parts[0])
            length = int(parts[1])
            data = bytes.fromhex(parts[2])
            rssi = int(parts[3])
            snr = int(parts[4])
            frequency_error = int(parts[5]) if len(parts) >= 6 else 0
        except ValueError:
            LOG.warning("bad +RCV values: %s", body)
            return
        if len(data) != length:
            LOG.warning("+RCV len mismatch: declared %d got %d", length, len(data))
        pkt = ReceivedPacket(addr, data, rssi, snr, frequency_error)
        LOG.info("RX %d bytes rssi=%d snr=%d ferr=%d", len(data), rssi, snr, frequency_error)
        if self.on_packet:
            self.on_packet(pkt)


def on_packet(pkt: ReceivedPacket) -> None:
    print(
        f"\n[RX] {len(pkt.data)} bytes  rssi={pkt.rssi} dBm  snr={pkt.snr} dB"
        f"  ferr={pkt.frequency_error} Hz"
        f"\n     hex : {pkt.data.hex().upper()}"
        f"\n     ascii: {pkt.data.decode('ascii', errors='replace')}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="at-os3 CH32V003 AT LoRa modem test")
    parser.add_argument("port", help="serial port, e.g. /dev/ttyACM0 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--freq", type=int, default=433175000, help="frequency in Hz, integer only")
    parser.add_argument("--sf", type=int, default=12, help="LoRa spreading factor")
    parser.add_argument("--bw", type=float, default=125.0, help="LoRa bandwidth in kHz")
    parser.add_argument("--cr", type=int, default=5, help="LoRa coding rate denominator: 5..8 means 4/5..4/8")
    parser.add_argument("--pre", type=int, default=8, help="preamble length in symbols")
    parser.add_argument("--power", type=int, default=15, help="TX power argument passed to AT+CRFOP")
    parser.add_argument("--sw", "--sync-word", dest="sync_word", type=parse_int, default=0x12)
    parser.add_argument("--iqi", type=int, choices=(0, 1), default=0, help="0=normal IQ, 1=inverted IQ")
    parser.add_argument("--crc", type=int, choices=(0, 1), default=1, help="0=CRC off, 1=CRC on")
    parser.add_argument("--fldro", type=int, choices=(0, 1, 2), default=2, help="0=off, 1=on, 2=auto")
    parser.add_argument("--len", dest="implicit_len", type=int, default=0, help="0=explicit header, 1..255=implicit length")
    parser.add_argument("--send", metavar="HEX", help="send hex payload then exit")
    parser.add_argument("--send-text", metavar="TEXT", help="send UTF-8 text payload then exit")
    parser.add_argument("--rx-seconds", type=float, default=0.0, help="listen duration; default is until Ctrl-C")
    parser.add_argument("--probe", action="store_true", help="probe the selected radio backend and exit")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    if args.send and args.send_text:
        parser.error("--send and --send-text are mutually exclusive")
    if not 0 <= args.sync_word <= 0xFF:
        parser.error("--sw/--sync-word must fit in one byte")

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    radio = Radio(args.port, baud=args.baud)
    radio.on_packet = on_packet

    print(f"Opening {args.port} @ {args.baud} baud ...")
    try:
        radio.start()
    except Exception as exc:
        print(f"Cannot open port: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc

    try:
        print("AT ... ", end="", flush=True)
        if not radio.ping():
            print("no response")
            raise SystemExit(1)
        print("OK")

        probe = radio.probe()
        if probe is not None:
            probe_kind = (probe >> 8) & 0xFF
            probe_value = probe & 0xFF
            if probe_kind == 0x12:
                if probe_value == 0x12:
                    print("Probe: SX1278 RegVersion=0x12  SPI link OK")
                else:
                    print(f"Probe: SX1278 RegVersion=0x{probe_value:02X}  expected 0x12")
            elif probe_kind == 0x62:
                if probe_value in (0x00, 0xFF):
                    print(f"Probe: SX1262 status=0x{probe_value:02X}  suspicious")
                else:
                    print(f"Probe: SX1262 status=0x{probe_value:02X}  SPI link OK")
            else:
                print(f"Probe: unknown response 0x{probe:04X}")
        else:
            ver = radio.read_reg(0x42)
            if ver == 0x12:
                print("Probe: legacy SX1278 RegVersion=0x12  SPI link OK")
            elif ver is None:
                print("Probe: no response")
            else:
                print(f"Probe: legacy RegVersion=0x{ver:02X}  expected 0x12")
        if args.probe:
            return

        band = radio.query_band()
        params = radio.query_parameter()
        iqi = radio.query_iq_inverted()
        if band:
            print(f"Current band : {band} Hz")
        if params:
            sf, bw_code, cr_code, preamble = params
            bw_label = BW_KHZ[bw_code] if 0 <= bw_code < len(BW_KHZ) else f"code {bw_code}"
            print(f"Current param: SF{sf} BW{bw_label}k CR4/{cr_code + 4} pre={preamble}")
        if iqi is not None:
            print(f"Current IQ   : {'inverted' if iqi else 'normal'}")

        print(
            f"\nConfiguring: {args.freq} Hz SF{args.sf} BW{args.bw}k "
            f"CR4/{args.cr} sw=0x{args.sync_word:02X} pre={args.pre} "
            f"IQ={'inverted' if args.iqi else 'normal'} pwr={args.power}"
        )
        if not radio.configure(
            freq_hz=args.freq,
            sf=args.sf,
            bw_khz=args.bw,
            cr=args.cr,
            preamble=args.pre,
            power=args.power,
            sync_word=args.sync_word,
            inverted_iq=bool(args.iqi),
            crc=bool(args.crc),
            fldro=args.fldro,
            implicit_len=args.implicit_len,
        ):
            print("Configuration failed", file=sys.stderr)
            raise SystemExit(1)

        if args.send or args.send_text:
            if args.send_text:
                data = args.send_text.encode("utf-8")
            else:
                try:
                    data = bytes.fromhex(args.send or "")
                except ValueError as exc:
                    print(f"Invalid hex payload: {args.send}", file=sys.stderr)
                    raise SystemExit(1) from exc
            print(f"\nSending {len(data)} bytes: {data.hex().upper()}")
            if not radio.send(data):
                print("TX command failed", file=sys.stderr)
                raise SystemExit(1)
            time.sleep(2.0)
            return

        if not radio.set_rx():
            print("RX start failed", file=sys.stderr)
            raise SystemExit(1)

        duration = f"{args.rx_seconds:g}s" if args.rx_seconds else "Ctrl-C to stop"
        print(
            f"\nListening on {args.freq / 1_000_000:.3f} MHz SF{args.sf} BW{args.bw}k "
            f"sw=0x{args.sync_word:02X} IQ={'inverted' if args.iqi else 'normal'} ... ({duration})\n"
        )
        if args.rx_seconds:
            deadline = time.monotonic() + args.rx_seconds
            while time.monotonic() < deadline:
                time.sleep(0.1)
        else:
            while True:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        radio.set_standby()
        radio.stop()


if __name__ == "__main__":
    main()
