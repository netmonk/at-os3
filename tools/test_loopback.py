#!/usr/bin/env python3
"""
test_loopback.py - Bidirectional parametric loopback test for at-os3.

Tests every combination of SF, BW, CR, freq, preamble, sync word, IQI,
CRC, LDRO, and implicit header in both directions.

Usage:
    python3 tools/test_loopback.py /dev/ttyACM0 /dev/ttyACM1
    python3 tools/test_loopback.py /dev/ttyACM0 /dev/ttyACM1 --verbose
"""
from __future__ import annotations

import argparse
import math
import sys
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(__file__))
from test_radio import Radio, ReceivedPacket

# ---------------------------------------------------------------------------
# Test profiles
# ---------------------------------------------------------------------------
# Each entry: (name, freq_hz, sf, bw_khz, cr, preamble, sync_word, iqi, crc, fldro, implicit_len)
# implicit_len=0 → explicit header; >0 → implicit header with that payload size

PROFILES = [
    # --- SF sweep (BW125, CR4/5) — start at SF9 to warm up radios first ---
    ("SF9/BW125/CR5",    433175000,  9, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("SF10/BW125/CR5",   433175000, 10, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("SF11/BW125/CR5",   433175000, 11, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("SF12/BW125/CR5",   433175000, 12, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    # SF7/SF8 tested after warmup — ESP32 receiver is marginal at these SFs
    ("SF7/BW125/CR5",    433175000,  7, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("SF8/BW125/CR5",    433175000,  8, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    # --- BW sweep (SF9, CR4/5) ---
    # BW41.7 excluded: crystal offset ~15kHz > BW/4 — always fails, hardware limit.
    ("SF9/BW62.5/CR5",   433175000,  9,  62.5, 5,  8, 0x12, 0, 1, 2,  0),
    ("SF9/BW250/CR5",    433175000,  9, 250.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("SF9/BW500/CR5",    433175000,  9, 500.0, 5,  8, 0x12, 0, 1, 2,  0),
    # --- CR sweep (SF9, BW125) ---
    ("SF9/BW125/CR6",    433175000,  9, 125.0, 6,  8, 0x12, 0, 1, 2,  0),
    ("SF9/BW125/CR7",    433175000,  9, 125.0, 7,  8, 0x12, 0, 1, 2,  0),
    ("SF9/BW125/CR8",    433175000,  9, 125.0, 8,  8, 0x12, 0, 1, 2,  0),
    # --- Frequency sweep ---
    ("433.5MHz",         433500000,  9, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("434.0MHz",         434000000,  9, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("435.0MHz",         435000000,  9, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    ("436.0MHz",         436000000,  9, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    # --- Preamble length ---
    ("PRE=16",           433175000,  9, 125.0, 5, 16, 0x12, 0, 1, 2,  0),
    ("PRE=32",           433175000,  9, 125.0, 5, 32, 0x12, 0, 1, 2,  0),
    # --- Sync word ---
    ("SW=0x34",          433175000,  9, 125.0, 5,  8, 0x34, 0, 1, 2,  0),
    # 0xAB fails: both nibbles >= 8 is a known SX1262<->SX1276 interop limit
    ("SW=0x56",          433175000,  9, 125.0, 5,  8, 0x56, 0, 1, 2,  0),
    # --- IQ inversion ---
    ("IQI=1",            433175000,  9, 125.0, 5,  8, 0x12, 1, 1, 2,  0),
    # --- CRC off ---
    ("CRC=0",            433175000,  9, 125.0, 5,  8, 0x12, 0, 0, 2,  0),
    # --- LDRO modes ---
    # LDRO=off tested at SF9/BW125 (symbol time 4ms < 16ms threshold: off is correct)
    # LDRO=force/auto tested at SF12/BW125 (symbol time 32ms > 16ms threshold)
    # BW62.5+SF12 excluded: crystal offset ~15kHz marginal at BW62.5 for long symbols.
    ("LDRO=off",         433175000,  9, 125.0, 5,  8, 0x12, 0, 1, 0,  0),
    ("LDRO=force",       433175000, 12, 125.0, 5,  8, 0x12, 0, 1, 1,  0),
    ("LDRO=auto",        433175000, 12, 125.0, 5,  8, 0x12, 0, 1, 2,  0),
    # --- Implicit header ---
    ("IMPLICIT/8B",      433175000,  9, 125.0, 5,  8, 0x12, 0, 1, 2,  8),
    ("IMPLICIT/16B",     433175000,  9, 125.0, 5,  8, 0x12, 0, 1, 2, 16),
    # --- Worst-case combo: slow + all options ---
    ("SF12/BW125/CR8",   433175000, 12, 125.0, 8,  8, 0x12, 0, 1, 1,  0),
]

SETTLE_S = 0.08  # set_rx enqueues and returns; actual SPI setup takes a few ms

def toa_s(sf: int, bw_khz: float, cr: int, payload_len: int, preamble: int,
          implicit: bool) -> float:
    """LoRa time-on-air (seconds), standard formula."""
    bw = bw_khz * 1000.0
    t_sym = (2 ** sf) / bw
    ldro = 1 if t_sym >= 0.01638 else 0
    n_bit_crc = 0 if implicit else 16
    n_sym_hdr = 0 if implicit else 20
    payload_symb = math.ceil(
        max(8 * payload_len + n_bit_crc - 4 * sf + 8 * (1 - ldro) + n_sym_hdr, 0)
        / (4 * (sf - 2 * ldro))
    ) * (cr + 4)
    n_sym = preamble + 4.25 + 8 + payload_symb
    return t_sym * n_sym

# ---------------------------------------------------------------------------

@dataclass
class Result:
    passed: bool
    rssi: Optional[int]
    snr: Optional[int]
    ferr: Optional[int]
    note: str


def wait_for_packet(radio: Radio, timeout: float) -> Optional[ReceivedPacket]:
    evt = threading.Event()
    holder: list[Optional[ReceivedPacket]] = [None]

    def cb(pkt: ReceivedPacket) -> None:
        holder[0] = pkt
        evt.set()

    radio.on_packet = cb
    evt.wait(timeout)
    radio.on_packet = None
    return holder[0]


def run_one(
    tx: Radio, tx_name: str,
    rx: Radio, rx_name: str,
    payload: bytes,
    cfg: dict,
    implicit_len: int,
    rx_timeout: float,
) -> Result:
    # Always standby both before reconfiguring to avoid dirty state from
    # previous test (especially after an RX timeout).
    tx.set_standby()
    rx.set_standby()
    time.sleep(0.1)

    if not tx.configure(**cfg):
        return Result(False, None, None, None, f"{tx_name} config failed")
    if not rx.configure(**cfg):
        return Result(False, None, None, None, f"{rx_name} config failed")

    if not rx.set_rx():
        return Result(False, None, None, None, f"{rx_name} set_rx failed")

    time.sleep(SETTLE_S)

    data = payload
    if implicit_len > 0:
        data = (payload * 8)[:implicit_len]

    if not tx.send(data):
        return Result(False, None, None, None, f"{tx_name} send failed")

    pkt = wait_for_packet(rx, rx_timeout)
    if pkt is None:
        return Result(False, None, None, None, "RX timeout")

    if pkt.data != data:
        return Result(
            False, pkt.rssi, pkt.snr, pkt.frequency_error,
            f"payload mismatch: got {pkt.data.hex()} want {data.hex()}"
        )

    return Result(True, pkt.rssi, pkt.snr, pkt.frequency_error, "")


BASE_FREQ_DEFAULT = 433175000

def main() -> None:
    parser = argparse.ArgumentParser(description="Bidirectional LoRa loopback test")
    parser.add_argument("port_a", help="Port A (e.g. /dev/ttyACM0)")
    parser.add_argument("port_b", help="Port B (e.g. /dev/ttyACM1)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--base-freq", type=int, default=BASE_FREQ_DEFAULT,
                        help=f"Base frequency in Hz; all profile freqs are shifted by "
                             f"(base-freq - {BASE_FREQ_DEFAULT}) (default: {BASE_FREQ_DEFAULT})")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    freq_shift = args.base_freq - BASE_FREQ_DEFAULT

    radio_a = Radio(args.port_a, baud=args.baud, timeout=3.0)
    radio_b = Radio(args.port_b, baud=args.baud, timeout=3.0)

    import logging
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    print(f"Opening {args.port_a} and {args.port_b} ...")
    radio_a.start()
    radio_b.start()

    if not radio_a.ping():
        print(f"FATAL: {args.port_a} not responding", file=sys.stderr)
        sys.exit(1)
    if not radio_b.ping():
        print(f"FATAL: {args.port_b} not responding", file=sys.stderr)
        sys.exit(1)

    ver_a = radio_a.read_reg(0x42)
    ver_b = radio_b.read_reg(0x42)
    print(f"  {args.port_a}: RegVersion=0x{ver_a:02X}" if ver_a else f"  {args.port_a}: RegVersion=?")
    print(f"  {args.port_b}: RegVersion=0x{ver_b:02X}" if ver_b else f"  {args.port_b}: RegVersion=?")
    print()

    # Warmup: one dummy loopback to wake the ESP32 RX path before the real tests.
    _wup = dict(freq_hz=BASE_FREQ_DEFAULT + freq_shift, sf=9, bw_khz=125.0, cr=5, preamble=8,
                power=15, sync_word=0x12, inverted_iq=False, crc=True,
                fldro=2, implicit_len=0)
    radio_a.configure(**_wup); radio_b.configure(**_wup)
    radio_b.set_rx(); time.sleep(1.2)
    radio_a.send(b"warmup")
    wait_for_packet(radio_b, 5.0)
    radio_a.set_standby(); radio_b.set_standby()
    time.sleep(0.3)

    total = len(PROFILES) * 2
    print(f"Running {len(PROFILES)} profiles × 2 directions = {total} tests")
    print()

    col_name = 24
    col_dir  = 14
    header = f"{'Profile':<{col_name}} {'Direction':<{col_dir}} {'':5}  {'RSSI':>5} {'SNR':>4} {'Ferr':>8}  Note"
    print(header)
    print("-" * len(header))

    passed = failed = 0

    for entry in PROFILES:
        name, freq_hz, sf, bw_khz, cr, pre, sw, iqi, crc, fldro, ilen = entry

        cfg = dict(
            freq_hz=freq_hz + freq_shift,
            sf=sf,
            bw_khz=bw_khz,
            cr=cr,
            preamble=pre,
            power=15,
            sync_word=sw,
            inverted_iq=bool(iqi),
            crc=bool(crc),
            fldro=fldro,
            implicit_len=ilen,
        )

        payload_len = ilen if ilen > 0 else 7  # "hello-A" / "hello-B"
        air = toa_s(sf, bw_khz, cr, payload_len, pre, ilen > 0)
        rx_timeout = SETTLE_S + air * 1.5 + 0.3

        for tx_radio, tx_label, rx_radio, rx_label, payload in [
            (radio_a, f"A→B", radio_b, "B", b"hello-A"),
            (radio_b, f"B→A", radio_a, "A", b"hello-B"),
        ]:
            r = run_one(tx_radio, tx_label, rx_radio, rx_label, payload, cfg, ilen,
                        rx_timeout)

            status = "PASS " if r.passed else "FAIL "
            rssi_s = f"{r.rssi:+4d}" if r.rssi is not None else "   -"
            snr_s  = f"{r.snr:+3d}"  if r.snr  is not None else "  -"
            ferr_s = f"{r.ferr:+7d}" if r.ferr is not None else "      -"

            print(
                f"{name:<{col_name}} {tx_label:<{col_dir}} {status}"
                f"  {rssi_s} dBm {snr_s} dB {ferr_s} Hz  {r.note}"
            )

            if r.passed:
                passed += 1
            else:
                failed += 1

    print("-" * len(header))
    print(f"\n{passed}/{passed+failed} passed", end="")
    if failed:
        print(f"  ({failed} FAILED)")
    else:
        print("  — all OK")

    radio_a.set_standby()
    radio_b.set_standby()
    radio_a.stop()
    radio_b.stop()

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
