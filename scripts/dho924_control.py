#!/usr/bin/env python3
"""RIGOL DHO800/DHO900/MHO900 LAN control utility.

The module provides:

* raw TCP SCPI communication on port 5555;
* cached-device-first discovery with manual selection after every scan;
* a Python API for acquisition, channels, timebase, trigger, measurement,
  waveform and screenshot transfer, digital channels, display and system;
* command-line subcommands;
* a mouse/keyboard full-screen terminal interface (no GUI window).

Examples:

    python dho924_control.py
    python dho924_control.py --host 192.0.2.120 idn
    python dho924_control.py status
    python dho924_control.py channel --channel 1 --scale 500m --offset 0
    python dho924_control.py trigger --mode EDGE --source CHAN1 --level 500m
    python dho924_control.py measure VPP --source CHAN1
    python dho924_control.py waveform --source CHAN1 --csv ch1.csv
    python dho924_control.py screenshot scope.png

Only the standard library is required.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shutil
import socket
import struct
import sys
import textwrap
import threading
import time
import unicodedata
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Sequence


DEFAULT_HOST = "192.0.2.120"
DEFAULT_PORT = 5555
DEFAULT_DISCOVERY_PORTS = (5555, 5025, 4880)
DEFAULT_TIMEOUT = 4.0
DEFAULT_SCAN_TIMEOUT = 0.25
DEFAULT_CACHE_PATH = Path(__file__).with_name(".dho_device_cache.json")
MAX_SCAN_HOSTS = 4096

ANSI_RESET = "\x1b[0m"
ANSI_BOLD = "\x1b[1m"
ANSI_DIM = "\x1b[2m"
ANSI_REVERSE = "\x1b[7m"
ANSI_CYAN = "\x1b[36m"
ANSI_GREEN = "\x1b[32m"
ANSI_YELLOW = "\x1b[33m"
ANSI_RED = "\x1b[31m"


class ScopeError(RuntimeError):
    """Base exception for connection and SCPI protocol errors."""


def parse_engineering_number(value: str | int | float) -> float:
    """Parse values such as 10n, 500m, 2.5k and 1M."""
    if isinstance(value, (int, float)):
        return float(value)
    text = str(value).strip().replace("µ", "u").replace("μ", "u")
    match = re.fullmatch(
        r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*([fpnumkKMG]?)\s*(?:[a-zA-ZΩ%/]*)?",
        text,
    )
    if not match:
        raise ValueError(f"Invalid numeric value: {value!r}")
    number = float(match.group(1))
    suffix = match.group(2)
    scale = {
        "": 1.0,
        "f": 1e-15,
        "p": 1e-12,
        "n": 1e-9,
        "u": 1e-6,
        "m": 1e-3,
        "k": 1e3,
        "K": 1e3,
        "M": 1e6,
        "G": 1e9,
    }[suffix]
    return number * scale


def scpi_number(value: int | float) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("SCPI numeric values must be finite")
    return f"{number:.12g}"


def scpi_bool(value: bool | str | int) -> str:
    if isinstance(value, str):
        normalized = value.strip().upper()
        if normalized in {"1", "ON", "TRUE", "YES", "Y"}:
            return "ON"
        if normalized in {"0", "OFF", "FALSE", "NO", "N"}:
            return "OFF"
        raise ValueError(f"Invalid boolean value: {value!r}")
    return "ON" if bool(value) else "OFF"


def scpi_quote(value: str) -> str:
    return '"' + str(value).replace('"', '""') + '"'


def parse_bool_response(value: str) -> bool:
    return value.strip().upper() in {"1", "ON", "TRUE"}


def validate_channel(channel: int) -> int:
    channel = int(channel)
    if channel not in {1, 2, 3, 4}:
        raise ValueError("Channel must be 1, 2, 3 or 4")
    return channel


def validate_afg_channel(channel: int) -> int:
    channel = int(channel)
    if channel not in {1, 2}:
        raise ValueError("AFG channel must be 1 or 2")
    return channel


def normalize_waveform_source(source: str) -> str:
    """Return the compact source token used by DHO/MHO query replies."""
    value = str(source).strip().upper()
    analog = re.fullmatch(r"CHAN(?:NEL)?([1-4])", value)
    if analog:
        return f"CHAN{analog.group(1)}"
    math_source = re.fullmatch(r"MATH(?:EMATICS)?([1-4])", value)
    if math_source:
        return f"MATH{math_source.group(1)}"
    digital = re.fullmatch(r"D(?:IGITAL)?(1[0-5]|[0-9])", value)
    if digital:
        return f"D{digital.group(1)}"
    return value


def normalize_waveform_mode(mode: str) -> str:
    value = str(mode).strip().upper()
    if value.startswith("NORM"):
        return "NORM"
    if value.startswith("MAX"):
        return "MAX"
    return value


def normalize_waveform_format(data_format: str) -> str:
    value = str(data_format).strip().upper()
    if value.startswith("ASC"):
        return "ASC"
    return value


def invalid_measurement(value: float) -> bool:
    """RIGOL uses approximately 9.9E37 for an invalid measurement."""
    return not math.isfinite(value) or abs(value) >= 9e37


def format_engineering(value: float | int | str, unit: str = "") -> str:
    if isinstance(value, str):
        with contextlib.suppress(ValueError):
            value = float(value)
    if not isinstance(value, (int, float)):
        return f"{value}{unit}"
    number = float(value)
    if not math.isfinite(number):
        return str(number)
    if number == 0:
        return f"0 {unit}".rstrip()
    prefixes = (
        (1e9, "G"),
        (1e6, "M"),
        (1e3, "k"),
        (1.0, ""),
        (1e-3, "m"),
        (1e-6, "u"),
        (1e-9, "n"),
        (1e-12, "p"),
    )
    magnitude = abs(number)
    for factor, prefix in prefixes:
        if magnitude >= factor or factor == 1e-12:
            return f"{number / factor:.6g} {prefix}{unit}".rstrip()
    return f"{number:.6g} {unit}".rstrip()


@dataclass(frozen=True)
class DiscoveredScope:
    host: str
    port: int
    identity: str

    @property
    def fields(self) -> tuple[str, str, str, str]:
        parts = [part.strip() for part in self.identity.split(",")]
        return tuple((parts + [""] * 4)[:4])  # type: ignore[return-value]

    @property
    def manufacturer(self) -> str:
        return self.fields[0]

    @property
    def model(self) -> str:
        return self.fields[1]

    @property
    def serial(self) -> str:
        return self.fields[2]

    @property
    def version(self) -> str:
        return self.fields[3]

    def display_name(self) -> str:
        model = self.model or "未知型号"
        serial = f"SN:{self.serial}" if self.serial else "无序列号"
        manufacturer = self.manufacturer or "未知厂商"
        kind = scope_family_from_identity(self.identity) or "SCPI"
        return (
            f"{model}  |  {self.host}:{self.port}  |  {serial}  |  "
            f"{manufacturer}  |  {kind}"
        )


@dataclass(frozen=True)
class DiscoveryProgress:
    completed: int
    total: int
    current_host: str = ""
    current_port: int = 0
    candidates: tuple[DiscoveredScope, ...] = ()

    @property
    def percent(self) -> float:
        return 100.0 if self.total == 0 else self.completed * 100.0 / self.total


@dataclass(frozen=True)
class ScopeLocation:
    selected: DiscoveredScope | None
    candidates: tuple[DiscoveredScope, ...]
    source: str
    networks: tuple[str, ...] = ()


def scope_family_from_identity(identity: str) -> str | None:
    fields = [part.strip().upper() for part in identity.split(",")]
    if len(fields) < 2:
        return None
    if "RIGOL" not in fields[0]:
        return None
    model = fields[1]
    if model.startswith(("DHO8", "DHO9")):
        return "DHO"
    if model.startswith("MHO9"):
        return "MHO"
    return None


def is_mho_scope_identity(identity: str) -> bool:
    return scope_family_from_identity(identity) == "MHO"


def is_supported_scope_identity(identity: str) -> bool:
    return scope_family_from_identity(identity) is not None


def identity_key(identity: str) -> tuple[str, str, str] | tuple[str]:
    fields = [part.strip().upper() for part in identity.split(",")]
    if len(fields) >= 3 and fields[2]:
        return fields[0], fields[1], fields[2]
    return (identity.strip().upper(),)


def load_last_scope(path: Path | str | None = DEFAULT_CACHE_PATH) -> DiscoveredScope | None:
    if path is None:
        return None
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        host = str(data["host"]).strip()
        port = int(data["port"])
        identity = str(data["identity"]).strip()
        if not host or not identity or not 1 <= port <= 65535:
            return None
        return DiscoveredScope(host, port, identity)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


def save_last_scope(
    scope: DiscoveredScope,
    path: Path | str | None = DEFAULT_CACHE_PATH,
) -> bool:
    if path is None:
        return False
    target = Path(path)
    payload = {
        "host": scope.host,
        "port": scope.port,
        "identity": scope.identity,
        "last_connected_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
    }
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        os.replace(temporary, target)
        return True
    except OSError:
        return False


def discovery_networks(
    subnets: Sequence[str] = (), preferred_hosts: Sequence[str | None] = ()
) -> tuple[ipaddress.IPv4Network, ...]:
    networks: set[ipaddress.IPv4Network] = set()
    for subnet in subnets:
        network = ipaddress.ip_network(subnet, strict=False)
        if network.version != 4:
            raise ValueError(f"Only IPv4 discovery networks are supported: {subnet}")
        networks.add(network)
    # Explicit discovery ranges are an override.  This keeps targeted scans
    # bounded and prevents an unrelated local adapter /24 from being added.
    if networks:
        return tuple(sorted(networks, key=lambda item: int(item.network_address)))
    for host in preferred_hosts:
        if not host:
            continue
        with contextlib.suppress(ValueError):
            address = ipaddress.IPv4Address(host)
            networks.add(ipaddress.IPv4Network(f"{address}/24", strict=False))
    with contextlib.suppress(OSError):
        for result in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            address = ipaddress.IPv4Address(result[4][0])
            if not address.is_loopback and not address.is_link_local:
                networks.add(ipaddress.IPv4Network(f"{address}/24", strict=False))
    if not networks:
        networks.add(ipaddress.IPv4Network(f"{DEFAULT_HOST}/24", strict=False))
    return tuple(sorted(networks, key=lambda item: int(item.network_address)))


def probe_scope(host: str, port: int = DEFAULT_PORT, timeout: float = 0.25) -> DiscoveredScope | None:
    try:
        with socket.create_connection((host, port), timeout=timeout) as connection:
            connection.settimeout(timeout)
            connection.sendall(b"*IDN?\n")
            response = bytearray()
            while b"\n" not in response and len(response) < 4096:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                response.extend(chunk)
            identity = bytes(response).split(b"\n", 1)[0].rstrip(b"\r").decode(
                "ascii", errors="replace"
            ).strip()
            return DiscoveredScope(host, int(port), identity) if identity else None
    except (OSError, ScopeError):
        return None


def discover_scopes(
    subnets: Sequence[str] = (),
    *,
    ports: Sequence[int] = (DEFAULT_PORT,),
    timeout: float = DEFAULT_SCAN_TIMEOUT,
    workers: int = 64,
    preferred_hosts: Sequence[str | None] = (),
    progress_callback: Callable[[DiscoveryProgress], None] | None = None,
) -> list[DiscoveredScope]:
    networks = discovery_networks(subnets, preferred_hosts)
    endpoints: list[tuple[str, int]] = []
    for network in networks:
        hosts = list(network.hosts())
        if len(hosts) * len(ports) > MAX_SCAN_HOSTS:
            raise ValueError(
                f"Discovery range {network} is too large; limit is {MAX_SCAN_HOSTS} endpoints"
            )
        endpoints.extend((str(host), int(port)) for host in hosts for port in ports)
    seen: set[tuple[str, int]] = set()
    endpoints = [endpoint for endpoint in endpoints if not (endpoint in seen or seen.add(endpoint))]
    total = len(endpoints)
    candidates: list[DiscoveredScope] = []
    if progress_callback:
        progress_callback(DiscoveryProgress(0, total))
    if not endpoints:
        return []
    with ThreadPoolExecutor(max_workers=max(1, min(workers, total))) as executor:
        futures = {
            executor.submit(probe_scope, host, port, timeout): (host, port)
            for host, port in endpoints
        }
        completed = 0
        for future in as_completed(futures):
            host, port = futures[future]
            completed += 1
            with contextlib.suppress(Exception):
                candidate = future.result()
                if candidate is not None:
                    candidates.append(candidate)
                    candidates.sort(
                        key=lambda item: (
                            0 if is_supported_scope_identity(item.identity) else 1,
                            int(ipaddress.IPv4Address(item.host)),
                            item.port,
                        )
                    )
            if progress_callback:
                progress_callback(
                    DiscoveryProgress(completed, total, host, port, tuple(candidates))
                )
    return candidates


def locate_scope(
    requested_host: str | None = None,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
    *,
    subnets: Sequence[str] = (),
    scan_timeout: float = DEFAULT_SCAN_TIMEOUT,
    cache_path: Path | str | None = DEFAULT_CACHE_PATH,
    allow_scan: bool = True,
    force_scan: bool = False,
    progress_callback: Callable[[DiscoveryProgress], None] | None = None,
) -> ScopeLocation:
    last = load_last_scope(cache_path)
    attempted: set[tuple[str, int]] = set()

    def try_endpoint(host: str | None, endpoint_port: int, source: str, probe_timeout: float):
        if not host or (host, endpoint_port) in attempted:
            return None
        attempted.add((host, endpoint_port))
        candidate = probe_scope(host, endpoint_port, probe_timeout)
        return ScopeLocation(candidate, (candidate,), source) if candidate else None

    if not force_scan:
        direct = try_endpoint(requested_host, port, "explicit", timeout)
        if direct:
            return direct
        if requested_host is not None:
            return ScopeLocation(None, (), "explicit-not-found")
        if last:
            direct = try_endpoint(
                last.host, last.port, "last", min(timeout, max(0.5, scan_timeout))
            )
            if direct:
                return direct
        else:
            direct = try_endpoint(
                DEFAULT_HOST, port, "default", min(timeout, max(0.5, scan_timeout))
            )
            if direct:
                return direct
    if not allow_scan:
        return ScopeLocation(None, (), "not-found")
    preferred = (requested_host, last.host if last else None, DEFAULT_HOST)
    networks = discovery_networks(subnets, preferred)
    scan_ports = {int(port), *DEFAULT_DISCOVERY_PORTS}
    if last:
        scan_ports.add(last.port)
    candidates = discover_scopes(
        tuple(map(str, networks)),
        ports=tuple(sorted(scan_ports)),
        timeout=scan_timeout,
        preferred_hosts=preferred,
        progress_callback=progress_callback,
    )
    if last:
        previous_key = identity_key(last.identity)
        candidates.sort(
            key=lambda item: (
                0 if identity_key(item.identity) == previous_key else 1,
                0 if is_supported_scope_identity(item.identity) else 1,
                int(ipaddress.IPv4Address(item.host)),
                item.port,
            )
        )
    return ScopeLocation(
        None,
        tuple(candidates),
        "scan-results" if candidates else "not-found",
        tuple(map(str, networks)),
    )


@dataclass(frozen=True)
class WaveformPreamble:
    format_code: int
    mode_code: int
    points: int
    count: int
    x_increment: float
    x_origin: float
    x_reference: float
    y_increment: float
    y_origin: float
    y_reference: float

    @classmethod
    def parse(cls, response: str) -> "WaveformPreamble":
        values = [item.strip() for item in response.split(",")]
        if len(values) != 10:
            raise ScopeError(f"Unexpected waveform preamble: {response!r}")
        return cls(
            int(float(values[0])),
            int(float(values[1])),
            int(float(values[2])),
            int(float(values[3])),
            *(float(value) for value in values[4:]),
        )

    @property
    def format_name(self) -> str:
        return {0: "BYTE", 1: "WORD", 2: "ASCII"}.get(
            self.format_code, str(self.format_code)
        )

    @property
    def mode_name(self) -> str:
        return {0: "NORMAL", 1: "MAXIMUM", 2: "RAW"}.get(
            self.mode_code, str(self.mode_code)
        )


@dataclass
class WaveformData:
    source: str
    preamble: WaveformPreamble
    times: list[float]
    volts: list[float]
    raw: bytes = b""

    @property
    def minimum(self) -> float:
        return min(self.volts) if self.volts else math.nan

    @property
    def maximum(self) -> float:
        return max(self.volts) if self.volts else math.nan

    @property
    def peak_to_peak(self) -> float:
        return self.maximum - self.minimum

    @property
    def mean(self) -> float:
        return sum(self.volts) / len(self.volts) if self.volts else math.nan

    def summary(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "format": self.preamble.format_name,
            "mode": self.preamble.mode_name,
            "points": len(self.volts),
            "time_start_s": self.times[0] if self.times else None,
            "time_end_s": self.times[-1] if self.times else None,
            "voltage_min_v": self.minimum,
            "voltage_max_v": self.maximum,
            "voltage_mean_v": self.mean,
            "voltage_vpp_v": self.peak_to_peak,
        }

    def save_csv(self, path: Path | str) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8", newline="") as stream:
            stream.write("index,time_s,voltage_v\n")
            for index, (time_value, voltage) in enumerate(zip(self.times, self.volts)):
                stream.write(f"{index},{time_value:.15g},{voltage:.15g}\n")
        return target


class RigolDHO:
    """Thread-safe SCPI client for supported RIGOL DHO/MHO oscilloscopes."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self._socket: socket.socket | None = None
        self._receive_buffer = bytearray()
        self._lock = threading.RLock()
        self._identity: str | None = None

    def connect(self) -> "RigolDHO":
        with self._lock:
            if self._socket is not None:
                return self
            try:
                connection = socket.create_connection(
                    (self.host, self.port), timeout=self.timeout
                )
                connection.settimeout(self.timeout)
                self._socket = connection
            except OSError as exc:
                self._socket = None
                raise ScopeError(
                    f"Cannot connect to {self.host}:{self.port}: {exc}"
                ) from exc
        return self

    def close(self) -> None:
        with self._lock:
            if self._socket is not None:
                try:
                    self._socket.close()
                finally:
                    self._socket = None
                    self._receive_buffer.clear()
                    self._identity = None

    def reconnect(self) -> "RigolDHO":
        self.close()
        return self.connect()

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def __enter__(self) -> "RigolDHO":
        return self.connect()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _connection(self) -> socket.socket:
        if self._socket is None:
            raise ScopeError("Oscilloscope is not connected")
        return self._socket

    def write(self, command: str) -> None:
        command = str(command).strip()
        if not command:
            raise ValueError("SCPI command cannot be empty")
        with self._lock:
            try:
                self._connection().sendall(command.encode("ascii") + b"\n")
            except (OSError, UnicodeEncodeError) as exc:
                raise ScopeError(f"Failed to send {command!r}: {exc}") from exc

    def _fill_buffer(self, minimum: int = 1) -> None:
        connection = self._connection()
        while len(self._receive_buffer) < minimum:
            try:
                chunk = connection.recv(max(4096, minimum - len(self._receive_buffer)))
            except socket.timeout as exc:
                raise ScopeError("Timed out waiting for oscilloscope response") from exc
            except OSError as exc:
                raise ScopeError(f"Failed to read oscilloscope response: {exc}") from exc
            if not chunk:
                raise ScopeError("Oscilloscope closed the connection")
            self._receive_buffer.extend(chunk)

    def _read_exact(self, count: int) -> bytes:
        self._fill_buffer(count)
        result = bytes(self._receive_buffer[:count])
        del self._receive_buffer[:count]
        return result

    def _read_line(self) -> str:
        while b"\n" not in self._receive_buffer:
            self._fill_buffer(len(self._receive_buffer) + 1)
        line, _, remaining = self._receive_buffer.partition(b"\n")
        self._receive_buffer = bytearray(remaining)
        return line.rstrip(b"\r").decode("ascii", errors="replace").strip()

    def _read_block(self) -> bytes:
        marker = self._read_exact(1)
        if marker != b"#":
            prefix = marker + self._read_exact(31)
            raise ScopeError(f"Expected IEEE block, received {prefix!r}")
        digits_raw = self._read_exact(1)
        if not digits_raw.isdigit():
            raise ScopeError(f"Invalid IEEE block length digit: {digits_raw!r}")
        digits = int(digits_raw)
        if digits == 0:
            raise ScopeError("Indefinite-length IEEE blocks are not supported")
        length_raw = self._read_exact(digits)
        if not length_raw.isdigit():
            raise ScopeError(f"Invalid IEEE block length: {length_raw!r}")
        payload = self._read_exact(int(length_raw))
        # DHO waveform and screenshot blocks are terminated with LF.  Consume
        # it explicitly so a delayed terminator cannot become an empty reply
        # to the next SCPI query.
        self._fill_buffer(1)
        if self._receive_buffer[:1] == b"\r":
            del self._receive_buffer[:1]
            self._fill_buffer(1)
        if self._receive_buffer[:1] == b"\n":
            del self._receive_buffer[:1]
        return payload

    def query(self, command: str) -> str:
        with self._lock:
            self.write(command)
            return self._read_line()

    def query_block(self, command: str) -> bytes:
        with self._lock:
            self.write(command)
            return self._read_block()

    def query_float(self, command: str) -> float:
        return parse_engineering_number(self.query(command))

    def query_int(self, command: str) -> int:
        return int(float(self.query(command)))

    def query_bool(self, command: str) -> bool:
        return parse_bool_response(self.query(command))

    def identify(self) -> str:
        self._identity = self.query("*IDN?")
        return self._identity

    @property
    def scope_family(self) -> str | None:
        identity = self._identity or self.identify()
        return scope_family_from_identity(identity)

    @property
    def is_mho(self) -> bool:
        return self.scope_family == "MHO"

    def scpi_version(self) -> str:
        return self.query(":SYSTem:VERSion?")

    def operation_complete(self) -> bool:
        return self.query("*OPC?") == "1"

    def system_error(self) -> str:
        return self.query(":SYSTem:ERRor?")

    def system_modules(self) -> str:
        return self.query(":SYSTem:MODules?")

    def option_status(self, option: str) -> bool:
        option = str(option).strip().upper()
        if not option:
            raise ValueError("Option name cannot be empty")
        return self.query_bool(f":SYSTem:OPTion:STATus? {option}")

    def clear_status(self) -> None:
        self.write("*CLS")

    def reset(self) -> None:
        self.write("*RST")

    def run(self) -> None:
        self.write(":RUN")

    def stop(self) -> None:
        self.write(":STOP")

    def single(self) -> None:
        self.write(":SINGle")

    def force_trigger(self) -> None:
        self.write(":TFORce")

    def autoset(self) -> None:
        self.write(":AUToset")

    def query_parameter(self, header: str, arguments: str = "") -> str:
        header = header.strip().rstrip("?")
        suffix = f" {arguments.strip()}" if arguments.strip() else ""
        return self.query(f"{header}?{suffix}")

    def set_parameter(self, header: str, value: Any) -> None:
        header = header.strip().rstrip("?")
        self.write(f"{header} {value}")

    def channel_state(self, channel: int) -> dict[str, Any]:
        channel = validate_channel(channel)
        prefix = f":CHANnel{channel}"
        return {
            "display": self.query_bool(prefix + ":DISPlay?"),
            "scale": self.query_float(prefix + ":SCALe?"),
            "offset": self.query_float(prefix + ":OFFSet?"),
            "coupling": self.query(prefix + ":COUPling?"),
            "probe": self.query_float(prefix + ":PROBe?"),
            "bandwidth": self.query(prefix + ":BWLimit?"),
            "invert": self.query_bool(prefix + ":INVert?"),
            "units": self.query(prefix + ":UNITs?"),
            "vernier": self.query_bool(prefix + ":VERNier?"),
            "position": self.query_float(prefix + ":POSition?"),
        }

    def displayed_channels(self) -> tuple[int, ...]:
        """Return analog channels whose display/acquisition path is enabled."""
        with self._lock:
            return tuple(
                channel
                for channel in range(1, 5)
                if self.query_bool(f":CHANnel{channel}:DISPlay?")
            )

    def configure_channel(
        self,
        channel: int,
        *,
        display: bool | str | None = None,
        scale: float | None = None,
        offset: float | None = None,
        coupling: str | None = None,
        probe: float | None = None,
        bandwidth: str | None = None,
        invert: bool | str | None = None,
        units: str | None = None,
        vernier: bool | str | None = None,
        position: float | None = None,
        label: str | None = None,
        label_show: bool | str | None = None,
    ) -> dict[str, Any]:
        channel = validate_channel(channel)
        prefix = f":CHANnel{channel}"
        commands: list[tuple[str, str]] = []
        if display is not None:
            commands.append((":DISPlay", scpi_bool(display)))
        if scale is not None:
            commands.append((":SCALe", scpi_number(scale)))
        if offset is not None:
            commands.append((":OFFSet", scpi_number(offset)))
        if coupling is not None:
            commands.append((":COUPling", coupling.upper()))
        if probe is not None:
            commands.append((":PROBe", scpi_number(probe)))
        if bandwidth is not None:
            commands.append((":BWLimit", bandwidth.upper()))
        if invert is not None:
            commands.append((":INVert", scpi_bool(invert)))
        if units is not None:
            commands.append((":UNITs", units.upper()))
        if vernier is not None:
            commands.append((":VERNier", scpi_bool(vernier)))
        if position is not None:
            commands.append((":POSition", scpi_number(position)))
        if label is not None:
            commands.append((":LABel:CONTent", scpi_quote(label)))
        if label_show is not None:
            commands.append((":LABel:SHOW", scpi_bool(label_show)))
        for suffix, value in commands:
            self.write(f"{prefix}{suffix} {value}")
        return self.channel_state(channel)

    def acquisition_state(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "type": self.query(":ACQuire:TYPE?"),
            "sample_rate": self.query_float(":ACQuire:SRATe?"),
            "memory_depth": self.query_float(":ACQuire:MDEPth?"),
            "averages": self.query_int(":ACQuire:AVERages?"),
        }
        if self.is_mho:
            result["bits"] = self.query_int(":ACQuire:BITS?")
        else:
            result.update(
                ultra_mode=self.query(":ACQuire:ULTRa:MODE?"),
                ultra_timeout=self.query_float(":ACQuire:ULTRa:TIMeout?"),
                ultra_max_frames=self.query_int(":ACQuire:ULTRa:MAXFrame?"),
            )
        return result

    def configure_acquisition(
        self,
        *,
        acquisition_type: str | None = None,
        memory_depth: str | int | float | None = None,
        averages: int | None = None,
        resolution_bits: int | None = None,
        ultra_mode: str | None = None,
        ultra_timeout: float | None = None,
        ultra_max_frames: int | None = None,
    ) -> dict[str, Any]:
        mho = self.is_mho
        normalized_type = acquisition_type.upper() if acquisition_type else None
        if mho and normalized_type == "ULTRA":
            raise ValueError("MHO900 uses HRES acquisition and does not support ULTRA")
        if not mho and normalized_type == "HRES":
            raise ValueError("HRES acquisition is only supported by MHO900")
        if mho and any(
            value is not None
            for value in (ultra_mode, ultra_timeout, ultra_max_frames)
        ):
            raise ValueError("MHO900 does not support DHO ULTRA acquisition parameters")
        if not mho and resolution_bits is not None:
            raise ValueError("Resolution bits are only supported by MHO900")
        values: list[tuple[str, Any]] = [
            (":ACQuire:TYPE", normalized_type),
            (":ACQuire:MDEPth", memory_depth),
            (":ACQuire:AVERages", averages),
        ]
        if mho:
            values.append((":ACQuire:BITS", resolution_bits))
        else:
            values.extend(
                (
                    (":ACQuire:ULTRa:MODE", ultra_mode.upper() if ultra_mode else None),
                    (":ACQuire:ULTRa:TIMeout", ultra_timeout),
                    (":ACQuire:ULTRa:MAXFrame", ultra_max_frames),
                )
            )
        for header, value in values:
            if value is not None:
                self.write(f"{header} {value}")
        return self.acquisition_state()

    def timebase_state(self) -> dict[str, Any]:
        return {
            "scale": self.query_float(":TIMebase:MAIN:SCALe?"),
            "offset": self.query_float(":TIMebase:MAIN:OFFSet?"),
            "mode": self.query(":TIMebase:MODE?"),
            "reference_mode": self.query(":TIMebase:HREFerence:MODE?"),
            "reference_position": self.query_float(
                ":TIMebase:HREFerence:POSition?"
            ),
            "vernier": self.query_bool(":TIMebase:VERNier?"),
            "roll": self.query_bool(":TIMebase:ROLL?"),
            "delayed": self.query_bool(":TIMebase:DELay:ENABle?"),
            "delay_scale": self.query_float(":TIMebase:DELay:SCALe?"),
            "delay_offset": self.query_float(":TIMebase:DELay:OFFSet?"),
        }

    def configure_timebase(
        self,
        *,
        scale: float | None = None,
        offset: float | None = None,
        mode: str | None = None,
        reference_mode: str | None = None,
        reference_position: float | None = None,
        vernier: bool | str | None = None,
        roll: bool | str | None = None,
        delayed: bool | str | None = None,
        delay_scale: float | None = None,
        delay_offset: float | None = None,
    ) -> dict[str, Any]:
        values = (
            (":TIMebase:MAIN:SCALe", scpi_number(scale) if scale is not None else None),
            (":TIMebase:MAIN:OFFSet", scpi_number(offset) if offset is not None else None),
            (":TIMebase:MODE", mode.upper() if mode else None),
            (":TIMebase:HREFerence:MODE", reference_mode.upper() if reference_mode else None),
            (":TIMebase:HREFerence:POSition", scpi_number(reference_position) if reference_position is not None else None),
            (":TIMebase:VERNier", scpi_bool(vernier) if vernier is not None else None),
            (":TIMebase:ROLL", scpi_bool(roll) if roll is not None else None),
            (":TIMebase:DELay:ENABle", scpi_bool(delayed) if delayed is not None else None),
            (":TIMebase:DELay:SCALe", scpi_number(delay_scale) if delay_scale is not None else None),
            (":TIMebase:DELay:OFFSet", scpi_number(delay_offset) if delay_offset is not None else None),
        )
        for header, value in values:
            if value is not None:
                self.write(f"{header} {value}")
        return self.timebase_state()

    def trigger_state(self) -> dict[str, Any]:
        mode = self.query(":TRIGger:MODE?")
        result: dict[str, Any] = {
            "status": self.query(":TRIGger:STATus?"),
            "mode": mode,
            "sweep": self.query(":TRIGger:SWEep?"),
            "coupling": self.query(":TRIGger:COUPling?"),
            "holdoff": self.query_float(":TRIGger:HOLDoff?"),
            "noise_reject": self.query_bool(":TRIGger:NREJect?"),
        }
        family = mode.strip().upper()
        if family == "EDGE":
            result.update(
                source=self.query(":TRIGger:EDGE:SOURce?"),
                slope=self.query(":TRIGger:EDGE:SLOPe?"),
                level=self.query_float(":TRIGger:EDGE:LEVel?"),
            )
        return result

    def configure_trigger(
        self,
        *,
        mode: str | None = None,
        source: str | None = None,
        level: float | None = None,
        slope: str | None = None,
        sweep: str | None = None,
        coupling: str | None = None,
        holdoff: float | None = None,
        noise_reject: bool | str | None = None,
    ) -> dict[str, Any]:
        selected_mode = mode.upper() if mode else self.query(":TRIGger:MODE?").upper()
        if mode:
            self.write(f":TRIGger:MODE {selected_mode}")
        for header, value in (
            (":TRIGger:SWEep", sweep.upper() if sweep else None),
            (":TRIGger:COUPling", coupling.upper() if coupling else None),
            (":TRIGger:HOLDoff", scpi_number(holdoff) if holdoff is not None else None),
            (":TRIGger:NREJect", scpi_bool(noise_reject) if noise_reject is not None else None),
        ):
            if value is not None:
                self.write(f"{header} {value}")
        family = "EDGE" if selected_mode == "EDGE" else selected_mode
        if source:
            self.write(f":TRIGger:{family}:SOURce {source.upper()}")
        if level is not None:
            self.write(f":TRIGger:{family}:LEVel {scpi_number(level)}")
        if slope:
            self.write(f":TRIGger:{family}:SLOPe {slope.upper()}")
        return self.trigger_state()

    def measure(self, item: str, source: str = "CHAN1", source_b: str | None = None) -> float:
        arguments = f"{item.upper()},{source.upper()}"
        if source_b:
            arguments += f",{source_b.upper()}"
        return self.query_float(f":MEASure:ITEM? {arguments}")

    def counter_state(self) -> dict[str, Any]:
        return {
            "enabled": self.query_bool(":COUNter:ENABle?"),
            "source": self.query(":COUNter:SOURce?"),
            "mode": self.query(":COUNter:MODE?"),
            "digits": self.query_int(":COUNter:NDIGits?"),
            "current": self.query_float(":COUNter:CURRent?"),
        }

    def configure_counter(
        self,
        *,
        enabled: bool | str | None = None,
        source: str | None = None,
        mode: str | None = None,
        digits: int | None = None,
    ) -> dict[str, Any]:
        for header, value in (
            (":COUNter:ENABle", scpi_bool(enabled) if enabled is not None else None),
            (":COUNter:SOURce", source.upper() if source else None),
            (":COUNter:MODE", mode.upper() if mode else None),
            (":COUNter:NDIGits", digits),
        ):
            if value is not None:
                self.write(f"{header} {value}")
        return self.counter_state()

    def dvm_state(self) -> dict[str, Any]:
        enabled = self.query_bool(":DVM:ENABle?")
        return {
            "enabled": enabled,
            "source": self.query(":DVM:SOURce?"),
            "mode": self.query(":DVM:MODE?"),
            # MHO934 firmware does not return DVM:CURRent? while DVM is off.
            "current": self.query_float(":DVM:CURRent?") if enabled else None,
        }

    def configure_dvm(
        self,
        *,
        enabled: bool | str | None = None,
        source: str | None = None,
        mode: str | None = None,
    ) -> dict[str, Any]:
        for header, value in (
            (":DVM:ENABle", scpi_bool(enabled) if enabled is not None else None),
            (":DVM:SOURce", source.upper() if source else None),
            (":DVM:MODE", mode.upper() if mode else None),
        ):
            if value is not None:
                self.write(f"{header} {value}")
        return self.dvm_state()

    def digital_state(self) -> dict[str, Any]:
        enabled = self.query_bool(":LA:ENABle?")
        result: dict[str, Any] = {
            "enabled": enabled,
            # MHO934 returns no response for LA:ACTive? until LA is enabled.
            "active": None,
            "size": self.query(":LA:SIZE?"),
        }
        for pod in (1, 2):
            result[f"pod{pod}_display"] = self.query_bool(f":LA:POD{pod}:DISPlay?")
            result[f"pod{pod}_threshold"] = self.query_float(
                f":LA:POD{pod}:THReshold?"
            )
        if enabled:
            result["active"] = self.query(":LA:ACTive?")
        return result

    def display_state(self) -> dict[str, Any]:
        return {
            "type": self.query(":DISPlay:TYPE?"),
            "wave_brightness": self.query_int(":DISPlay:WBRightness?"),
            "grid": self.query(":DISPlay:GRID?"),
            "grid_brightness": self.query_int(":DISPlay:GBRightness?"),
            "rulers": self.query_bool(":DISPlay:RULers?"),
            "color_temperature": self.query_bool(":DISPlay:COLor?"),
        }

    def _require_mho(self) -> None:
        if not self.is_mho:
            raise ScopeError("MHO900-specific function requested on a non-MHO scope")

    def afg_option_state(self) -> dict[str, bool]:
        self._require_mho()
        return {
            "bundle": self.option_status("BND"),
            "afg100": self.option_status("AFG100"),
            "afg50": self.option_status("AFG50"),
        }

    def _require_mho_afg(self) -> None:
        self._require_mho()
        if not (
            self.option_status("AFG100") or self.option_status("AFG50")
        ):
            raise ScopeError("The connected MHO900 does not have an active AFG option")

    def afg_state(self, channel: int = 1) -> dict[str, Any]:
        self._require_mho_afg()
        channel = validate_afg_channel(channel)
        prefix = f":SOURce{channel}"
        function = self.query(prefix + ":FUNCtion?").strip().upper()
        modulation_enabled = self.query_bool(prefix + ":MOD:STATe?")
        modulation_type = self.query(prefix + ":MOD:TYPe?").strip().upper()
        result: dict[str, Any] = {
            "channel": channel,
            "output": self.query_bool(prefix + ":OUTPut:STATe?"),
            "function": function,
            "amplitude": self.query_float(prefix + ":VOLTage:AMPLitude?"),
            "offset": self.query_float(prefix + ":VOLTage:OFFSet?"),
            "high": self.query_float(prefix + ":VOLTage:HIGH?"),
            "low": self.query_float(prefix + ":VOLTage:LOW?"),
            "impedance": self.query(prefix + ":IMPedance?"),
            "modulation": modulation_enabled,
            "modulation_type": modulation_type,
        }
        if function not in {"DC", "NOIS"}:
            result.update(
                frequency=self.query_float(prefix + ":FREQuency?"),
                period=self.query_float(prefix + ":PERiod?"),
                phase=self.query_float(prefix + ":PHASe?"),
            )
        if function == "RAMP":
            result["symmetry"] = self.query_float(
                prefix + ":FUNCtion:RAMP:SYMMetry?"
            )
        elif function == "SQU":
            result["duty"] = self.query_float(
                prefix + ":FUNCtion:SQUare:DUTY?"
            )
        elif function == "ARB":
            result["arbitrary_path"] = self.query(prefix + ":LOAD:ARBitrary?")
        if modulation_enabled and modulation_type in {"AM", "FM", "PM"}:
            mod_prefix = prefix + f":MOD:{modulation_type}"
            if modulation_type == "AM":
                result["modulation_depth"] = self.query_float(mod_prefix + ":DEPTh?")
            else:
                result["modulation_deviation"] = self.query_float(
                    mod_prefix + ":DEViation?"
                )
            result["modulation_frequency"] = self.query_float(
                mod_prefix + ":INTernal:FREQuency?"
            )
            result["modulation_function"] = self.query(
                mod_prefix + ":INTernal:FUNCtion?"
            )
        return result

    def configure_afg(
        self,
        channel: int = 1,
        *,
        output: bool | str | None = None,
        function: str | None = None,
        arbitrary_path: str | None = None,
        frequency: float | None = None,
        period: float | None = None,
        phase: float | None = None,
        symmetry: float | None = None,
        duty: float | None = None,
        amplitude: float | None = None,
        offset: float | None = None,
        high: float | None = None,
        low: float | None = None,
        impedance: str | None = None,
        modulation: bool | str | None = None,
        modulation_type: str | None = None,
        am_depth: float | None = None,
        fm_deviation: float | None = None,
        pm_deviation: float | None = None,
        modulation_frequency: float | None = None,
        modulation_function: str | None = None,
    ) -> dict[str, Any]:
        self._require_mho_afg()
        channel = validate_afg_channel(channel)
        if frequency is not None and period is not None:
            raise ValueError("Specify frequency or period, not both")
        if any(value is not None for value in (amplitude, offset)) and any(
            value is not None for value in (high, low)
        ):
            raise ValueError("Use amplitude/offset or high/low, not both")
        prefix = f":SOURce{channel}"
        values: list[tuple[str, Any]] = [
            (":FUNCtion", function.upper() if function else None),
            (":LOAD:ARBitrary", scpi_quote(arbitrary_path) if arbitrary_path else None),
            (":IMPedance", impedance.upper() if impedance else None),
            (":FREQuency", scpi_number(frequency) if frequency is not None else None),
            (":PERiod", scpi_number(period) if period is not None else None),
            (":PHASe", scpi_number(phase) if phase is not None else None),
            (":FUNCtion:RAMP:SYMMetry", scpi_number(symmetry) if symmetry is not None else None),
            (":FUNCtion:SQUare:DUTY", scpi_number(duty) if duty is not None else None),
            (":VOLTage:AMPLitude", scpi_number(amplitude) if amplitude is not None else None),
            (":VOLTage:OFFSet", scpi_number(offset) if offset is not None else None),
            (":VOLTage:HIGH", scpi_number(high) if high is not None else None),
            (":VOLTage:LOW", scpi_number(low) if low is not None else None),
            (":MOD:TYPe", modulation_type.upper() if modulation_type else None),
            (":MOD:AM:DEPTh", scpi_number(am_depth) if am_depth is not None else None),
            (":MOD:FM:DEViation", scpi_number(fm_deviation) if fm_deviation is not None else None),
            (":MOD:PM:DEViation", scpi_number(pm_deviation) if pm_deviation is not None else None),
        ]
        selected_modulation = (modulation_type or "AM").upper()
        if modulation_frequency is not None:
            values.append(
                (
                    f":MOD:{selected_modulation}:INTernal:FREQuency",
                    scpi_number(modulation_frequency),
                )
            )
        if modulation_function is not None:
            values.append(
                (
                    f":MOD:{selected_modulation}:INTernal:FUNCtion",
                    modulation_function.upper(),
                )
            )
        if modulation is not None:
            values.append((":MOD:STATe", scpi_bool(modulation)))
        if output is not None:
            # Output is intentionally applied last so a single configure call
            # cannot enable a partially configured waveform.
            values.append((":OUTPut:STATe", scpi_bool(output)))
        for suffix, value in values:
            if value is not None:
                self.write(f"{prefix}{suffix} {value}")
        return self.afg_state(channel)

    def synchronize_afg_phase(self, channel: int = 1) -> None:
        self._require_mho_afg()
        channel = validate_afg_channel(channel)
        self.write(f":SOURce{channel}:PHASe:SYNChronize")

    def waveform_preamble(self) -> WaveformPreamble:
        return WaveformPreamble.parse(self.query(":WAVeform:PREamble?"))

    def read_waveform(
        self,
        *,
        source: str | None = None,
        mode: str | None = None,
        data_format: str | None = None,
        points: int | None = None,
        start: int | None = None,
        stop: int | None = None,
    ) -> WaveformData:
        # Keep the complete setup/query/read transaction serialized.  SCPI has
        # no request IDs, so another thread must not interleave a query between
        # the preamble and binary payload.
        with self._lock:
            actual_source = self.query(":WAVeform:SOURce?")
            current_source = normalize_waveform_source(actual_source)
            requested_source = (
                normalize_waveform_source(source) if source is not None else None
            )
            if requested_source and requested_source != current_source:
                analog = re.fullmatch(r"CHAN([1-4])", requested_source)
                if analog and not self.query_bool(
                    f":CHANnel{analog.group(1)}:DISPlay?"
                ):
                    raise ScopeError(
                        f"Cannot select {requested_source} as waveform source: "
                        "channel display is OFF"
                    )
                self.write(f":WAVeform:SOURce {requested_source}")
                actual_source = self.query(":WAVeform:SOURce?")

            if mode is not None:
                requested_mode = normalize_waveform_mode(mode)
                current_mode = normalize_waveform_mode(
                    self.query(":WAVeform:MODE?")
                )
                if requested_mode != current_mode:
                    self.write(f":WAVeform:MODE {requested_mode}")
            if data_format is not None:
                requested_format = normalize_waveform_format(data_format)
                current_format = normalize_waveform_format(
                    self.query(":WAVeform:FORMat?")
                )
                if requested_format != current_format:
                    self.write(f":WAVeform:FORMat {requested_format}")
            if points is not None:
                requested_points = int(points)
                if requested_points != self.query_int(":WAVeform:POINts?"):
                    self.write(f":WAVeform:POINts {requested_points}")
            if start is not None:
                requested_start = int(start)
                if requested_start != self.query_int(":WAVeform:STARt?"):
                    self.write(f":WAVeform:STARt {requested_start}")
            if stop is not None:
                requested_stop = int(stop)
                if requested_stop != self.query_int(":WAVeform:STOP?"):
                    self.write(f":WAVeform:STOP {requested_stop}")

            preamble = self.waveform_preamble()
            if preamble.format_code == 2:
                response = self.query(":WAVeform:DATA?")
                volts = [
                    float(item) for item in response.split(",") if item.strip()
                ]
                raw = response.encode("ascii", errors="replace")
            else:
                raw = self.query_block(":WAVeform:DATA?")
                if preamble.format_code == 0:
                    samples: Iterable[int] = raw
                elif preamble.format_code == 1:
                    if len(raw) % 2:
                        raise ScopeError("WORD waveform payload has an odd byte length")
                    samples = struct.unpack(f"<{len(raw) // 2}H", raw)
                else:
                    raise ScopeError(
                        f"Unsupported waveform format {preamble.format_code}"
                    )
                volts = [
                    (sample - preamble.y_origin - preamble.y_reference)
                    * preamble.y_increment
                    for sample in samples
                ]
            times = [
                preamble.x_origin
                + (index - preamble.x_reference) * preamble.x_increment
                for index in range(len(volts))
            ]
            return WaveformData(actual_source, preamble, times, volts, raw)

    def screenshot(self, image_format: str = "PNG") -> bytes:
        image_format = image_format.upper()
        if image_format not in {"PNG", "BMP", "JPG", "JPEG"}:
            raise ValueError("Screenshot format must be PNG, BMP or JPG")
        if image_format == "JPEG":
            image_format = "JPG"
        return self.query_block(f":DISPlay:DATA? {image_format}")

    def status(self) -> dict[str, Any]:
        identity = self.identify()
        return {
            "identity": identity,
            "family": scope_family_from_identity(identity),
            "scpi_version": self.scpi_version(),
            "acquisition": self.acquisition_state(),
            "timebase": self.timebase_state(),
            "trigger": self.trigger_state(),
            "channels": {
                str(channel): self.channel_state(channel) for channel in range(1, 5)
            },
        }


@dataclass(frozen=True)
class TerminalEvent:
    kind: str
    key: str = ""
    text: str = ""
    x: int = 0
    y: int = 0
    button: int = -1


class TerminalSession:
    """Cross-platform alternate-screen keyboard and SGR mouse session."""

    def __init__(self) -> None:
        self._windows = os.name == "nt"
        self._original_input_mode: int | None = None
        self._original_output_mode: int | None = None
        self._stdin_fd: int | None = None
        self._termios_state: Any = None

    def __enter__(self) -> "TerminalSession":
        with contextlib.suppress(Exception):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        with contextlib.suppress(Exception):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
        if self._windows:
            self._enable_windows_vt()
        else:
            self._enable_posix_raw()
        sys.stdout.write(
            "\x1b[?1049h\x1b[?25l\x1b[?1000h\x1b[?1006h\x1b[2J\x1b[H"
        )
        sys.stdout.flush()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        sys.stdout.write(
            "\x1b[?1006l\x1b[?1000l\x1b[?25h\x1b[?1049l" + ANSI_RESET
        )
        sys.stdout.flush()
        if self._windows:
            self._restore_windows_modes()
        else:
            self._restore_posix_mode()

    def size(self) -> tuple[int, int]:
        size = shutil.get_terminal_size((128, 38))
        return max(72, size.columns), max(22, size.lines)

    def draw(self, content: str) -> None:
        sys.stdout.write("\x1b[H" + content)
        sys.stdout.flush()

    def read_event(self, timeout: float = 0.1) -> TerminalEvent:
        return (
            self._read_windows_event(timeout)
            if self._windows
            else self._read_posix_event(timeout)
        )

    def _enable_windows_vt(self) -> None:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        stdin_handle = kernel32.GetStdHandle(-10)
        stdout_handle = kernel32.GetStdHandle(-11)
        input_mode = ctypes.c_uint()
        output_mode = ctypes.c_uint()
        if kernel32.GetConsoleMode(stdin_handle, ctypes.byref(input_mode)):
            self._original_input_mode = input_mode.value
            kernel32.SetConsoleMode(
                stdin_handle,
                input_mode.value | 0x0080 | 0x0008 | 0x0010 | 0x0200,
            )
        if kernel32.GetConsoleMode(stdout_handle, ctypes.byref(output_mode)):
            self._original_output_mode = output_mode.value
            kernel32.SetConsoleMode(
                stdout_handle, output_mode.value | 0x0001 | 0x0004
            )

    def _restore_windows_modes(self) -> None:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        if self._original_input_mode is not None:
            kernel32.SetConsoleMode(
                kernel32.GetStdHandle(-10), self._original_input_mode
            )
        if self._original_output_mode is not None:
            kernel32.SetConsoleMode(
                kernel32.GetStdHandle(-11), self._original_output_mode
            )

    def _enable_posix_raw(self) -> None:
        import termios
        import tty

        self._stdin_fd = sys.stdin.fileno()
        self._termios_state = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

    def _restore_posix_mode(self) -> None:
        if self._stdin_fd is not None and self._termios_state is not None:
            import termios

            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._termios_state)

    def _read_windows_event(self, timeout: float) -> TerminalEvent:
        import msvcrt

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if msvcrt.kbhit():
                first = msvcrt.getwch()
                if first in {"\x00", "\xe0"}:
                    second = msvcrt.getwch()
                    mapping = {
                        "H": "up",
                        "P": "down",
                        "K": "left",
                        "M": "right",
                        "G": "home",
                        "O": "end",
                        "I": "pageup",
                        "Q": "pagedown",
                        "?": "f5",
                        "<": "f2",
                        "S": "delete",
                    }
                    return TerminalEvent("key", key=mapping.get(second, second))
                if first == "\x1b":
                    sequence = first
                    settle = time.monotonic() + 0.02
                    while time.monotonic() < settle:
                        if msvcrt.kbhit():
                            sequence += msvcrt.getwch()
                            settle = time.monotonic() + 0.005
                        else:
                            time.sleep(0.001)
                    return self._parse_escape(sequence)
                return self._character_event(first)
            time.sleep(0.005)
        return TerminalEvent("timeout")

    def _read_posix_event(self, timeout: float) -> TerminalEvent:
        import select

        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return TerminalEvent("timeout")
        data = os.read(sys.stdin.fileno(), 64).decode("utf-8", errors="replace")
        if data.startswith("\x1b"):
            return self._parse_escape(data)
        return self._character_event(data[0] if data else "")

    @staticmethod
    def _character_event(character: str) -> TerminalEvent:
        mapping = {
            "\r": "enter",
            "\n": "enter",
            "\t": "tab",
            "\x08": "backspace",
            "\x7f": "backspace",
            "\x03": "ctrl_c",
            "\x0c": "ctrl_l",
        }
        if character in mapping:
            return TerminalEvent("key", key=mapping[character])
        if character and character.isprintable():
            return TerminalEvent("text", text=character)
        return TerminalEvent("key", key=character)

    @staticmethod
    def _parse_escape(sequence: str) -> TerminalEvent:
        mouse = re.search(r"\x1b\[<(\d+);(\d+);(\d+)([Mm])", sequence)
        if mouse:
            code, x, y, action = mouse.groups()
            return TerminalEvent(
                "mouse",
                x=int(x),
                y=int(y),
                button=int(code),
                key="press" if action == "M" else "release",
            )
        mappings = {
            "\x1b[A": "up",
            "\x1b[B": "down",
            "\x1b[C": "right",
            "\x1b[D": "left",
            "\x1b[H": "home",
            "\x1b[F": "end",
            "\x1b[5~": "pageup",
            "\x1b[6~": "pagedown",
            "\x1b[15~": "f5",
            "\x1b[12~": "f2",
            "\x1b": "escape",
        }
        for prefix, key in mappings.items():
            if sequence.startswith(prefix):
                return TerminalEvent("key", key=key)
        return TerminalEvent("key", key="escape")


def display_width(text: str) -> int:
    plain = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", text)
    return sum(
        2 if unicodedata.east_asian_width(char) in {"W", "F"} else 1
        for char in plain
    )


def fit_display(text: str, width: int) -> str:
    if width <= 0:
        return ""
    if display_width(text) <= width:
        return text + " " * (width - display_width(text))
    result: list[str] = []
    used = 0
    in_escape = False
    escape: list[str] = []
    for char in text:
        if in_escape:
            escape.append(char)
            if char.isalpha():
                result.extend(escape)
                escape.clear()
                in_escape = False
            continue
        if char == "\x1b":
            in_escape = True
            escape = [char]
            continue
        cells = 2 if unicodedata.east_asian_width(char) in {"W", "F"} else 1
        if used + cells > max(0, width - 1):
            break
        result.append(char)
        used += cells
    return "".join(result) + "…" + ANSI_RESET + " " * max(0, width - used - 1)


def waveform_ascii_preview(
    values: Sequence[float], width: int = 52, height: int = 10
) -> list[str]:
    width = max(16, width)
    height = max(5, height)
    if not values:
        return [" " * width for _ in range(height)]
    low, high = min(values), max(values)
    span = high - low
    if span <= 0:
        span = 1.0
    if len(values) <= width:
        samples = list(values) + [values[-1]] * (width - len(values))
    else:
        samples = [values[int(index * (len(values) - 1) / (width - 1))] for index in range(width)]
    canvas = [[" " for _ in range(width)] for _ in range(height)]
    zero_row = None
    if low <= 0 <= high:
        zero_row = int(round((high - 0) / span * (height - 1)))
        for column in range(width):
            canvas[zero_row][column] = "·"
    previous: tuple[int, int] | None = None
    for x, value in enumerate(samples):
        y = int(round((high - value) / span * (height - 1)))
        y = max(0, min(height - 1, y))
        canvas[y][x] = "*"
        if previous is not None:
            py, px = previous
            if abs(y - py) > 1:
                for row in range(min(y, py) + 1, max(y, py)):
                    canvas[row][x] = "|"
        previous = y, x
    return ["".join(row) for row in canvas]


@dataclass
class MenuAction:
    label: str
    handler: Callable[[], Any]
    value: str = ""
    dangerous: bool = False


@dataclass(frozen=True)
class ParameterSpec:
    label: str
    header: str
    kind: str = "text"
    choices: tuple[str, ...] = ()
    quote: bool = False


CHANNEL_SPECS = (
    ParameterSpec("通道显示", ":CHANnel{ch}:DISPlay", "bool"),
    ParameterSpec("垂直档位 V/div", ":CHANnel{ch}:SCALe", "number"),
    ParameterSpec("垂直偏移 V", ":CHANnel{ch}:OFFSet", "number"),
    ParameterSpec("耦合", ":CHANnel{ch}:COUPling", "choice", ("DC", "AC", "GND")),
    ParameterSpec("探头倍率", ":CHANnel{ch}:PROBe", "number"),
    ParameterSpec("带宽限制", ":CHANnel{ch}:BWLimit", "choice", ("OFF", "20M")),
    ParameterSpec("波形反相", ":CHANnel{ch}:INVert", "bool"),
    ParameterSpec("单位", ":CHANnel{ch}:UNITs", "choice", ("VOLT", "WATT", "AMP", "UNKN")),
    ParameterSpec("垂直微调", ":CHANnel{ch}:VERNier", "bool"),
    ParameterSpec("垂直位置", ":CHANnel{ch}:POSition", "number"),
    ParameterSpec("通道延时校正 s", ":CHANnel{ch}:TCALibrate", "number"),
    ParameterSpec("标签显示", ":CHANnel{ch}:LABel:SHOW", "bool"),
    ParameterSpec("标签内容", ":CHANnel{ch}:LABel:CONTent", "text", quote=True),
)

ACQUISITION_SPECS = (
    ParameterSpec("采集类型", ":ACQuire:TYPE", "choice", ("NORMal", "AVERages", "PEAK", "ULTRa")),
    ParameterSpec("存储深度", ":ACQuire:MDEPth", "choice", ("AUTO", "1K", "10K", "100K", "1M", "5M", "10M", "25M", "50M")),
    ParameterSpec("平均次数", ":ACQuire:AVERages", "choice", ("2", "4", "8", "16", "32", "64", "128", "256", "512", "1024", "2048", "4096", "8192", "16384", "32768", "65536")),
    ParameterSpec("凝时模式", ":ACQuire:ULTRa:MODE", "choice", ("ADJacent", "OVERlay", "WATerfall", "PERSpective", "MOSaic")),
    ParameterSpec("凝时超时 s", ":ACQuire:ULTRa:TIMeout", "number"),
    ParameterSpec("凝时最大帧", ":ACQuire:ULTRa:MAXFrame", "number"),
)

MHO_ACQUISITION_SPECS = (
    ParameterSpec("采集类型", ":ACQuire:TYPE", "choice", ("NORMal", "AVERages", "PEAK", "HRESolution")),
    ParameterSpec("存储深度", ":ACQuire:MDEPth", "choice", ("AUTO", "1K", "10K", "100K", "1M", "10M", "25M", "50M", "100M", "125M", "200M", "250M", "500M")),
    ParameterSpec("平均次数", ":ACQuire:AVERages", "choice", ("2", "4", "8", "16", "32", "64", "128", "256", "512", "1024", "2048", "4096", "8192", "16384", "32768", "65536")),
    ParameterSpec("高分辨率位数", ":ACQuire:BITS", "choice", ("14", "16")),
)

MHO_AFG_FUNCTIONS = (
    "SINusoid", "SQUare", "RAMP", "NOISe", "DC", "ARB", "EXPRise",
    "EXPFall", "ECG1", "GAUSsian", "LORentz", "HAVersine", "SINC",
)
MHO_AFG_MODULATION_FUNCTIONS = (
    "SINusoid", "SQUare", "TRIangle", "UPRamp", "DNRamp", "NOISe",
)
MHO_AFG_SPECS = (
    ParameterSpec("波形类型", ":SOURce{ch}:FUNCtion", "choice", MHO_AFG_FUNCTIONS),
    ParameterSpec("任意波文件路径", ":SOURce{ch}:LOAD:ARBitrary", "text", quote=True),
    ParameterSpec("频率 Hz", ":SOURce{ch}:FREQuency", "number"),
    ParameterSpec("周期 s", ":SOURce{ch}:PERiod", "number"),
    ParameterSpec("相位 °", ":SOURce{ch}:PHASe", "number"),
    ParameterSpec("Ramp 对称度 %", ":SOURce{ch}:FUNCtion:RAMP:SYMMetry", "number"),
    ParameterSpec("Square 占空比 %", ":SOURce{ch}:FUNCtion:SQUare:DUTY", "number"),
    ParameterSpec("幅度 Vpp", ":SOURce{ch}:VOLTage:AMPLitude", "number"),
    ParameterSpec("偏置 V", ":SOURce{ch}:VOLTage:OFFSet", "number"),
    ParameterSpec("高电平 V", ":SOURce{ch}:VOLTage:HIGH", "number"),
    ParameterSpec("低电平 V", ":SOURce{ch}:VOLTage:LOW", "number"),
    ParameterSpec("输出阻抗", ":SOURce{ch}:IMPedance", "choice", ("OMEG", "FIFTy")),
    ParameterSpec("调制开关", ":SOURce{ch}:MOD:STATe", "bool"),
    ParameterSpec("调制类型", ":SOURce{ch}:MOD:TYPe", "choice", ("AM", "FM", "PM")),
    ParameterSpec("AM 深度 %", ":SOURce{ch}:MOD:AM:DEPTh", "number"),
    ParameterSpec("AM 内调制频率 Hz", ":SOURce{ch}:MOD:AM:INTernal:FREQuency", "number"),
    ParameterSpec("AM 内调制波形", ":SOURce{ch}:MOD:AM:INTernal:FUNCtion", "choice", MHO_AFG_MODULATION_FUNCTIONS),
    ParameterSpec("FM 频偏 Hz", ":SOURce{ch}:MOD:FM:DEViation", "number"),
    ParameterSpec("FM 内调制频率 Hz", ":SOURce{ch}:MOD:FM:INTernal:FREQuency", "number"),
    ParameterSpec("FM 内调制波形", ":SOURce{ch}:MOD:FM:INTernal:FUNCtion", "choice", MHO_AFG_MODULATION_FUNCTIONS),
    ParameterSpec("PM 相偏 °", ":SOURce{ch}:MOD:PM:DEViation", "number"),
    ParameterSpec("PM 内调制频率 Hz", ":SOURce{ch}:MOD:PM:INTernal:FREQuency", "number"),
    ParameterSpec("PM 内调制波形", ":SOURce{ch}:MOD:PM:INTernal:FUNCtion", "choice", MHO_AFG_MODULATION_FUNCTIONS),
)

TIMEBASE_SPECS = (
    ParameterSpec("主时基档位 s/div", ":TIMebase:MAIN:SCALe", "number"),
    ParameterSpec("主时基偏移 s", ":TIMebase:MAIN:OFFSet", "number"),
    ParameterSpec("时基模式", ":TIMebase:MODE", "choice", ("MAIN", "XY", "ROLL")),
    ParameterSpec("水平参考模式", ":TIMebase:HREFerence:MODE", "choice", ("CENTer", "LB", "RB", "TRIG", "USER")),
    ParameterSpec("水平参考位置", ":TIMebase:HREFerence:POSition", "number"),
    ParameterSpec("时基微调", ":TIMebase:VERNier", "bool"),
    ParameterSpec("滚动模式", ":TIMebase:ROLL", "bool"),
    ParameterSpec("延迟扫描", ":TIMebase:DELay:ENABle", "bool"),
    ParameterSpec("延迟时基档位", ":TIMebase:DELay:SCALe", "number"),
    ParameterSpec("延迟时基偏移", ":TIMebase:DELay:OFFSet", "number"),
)

TRIGGER_SPECS = (
    ParameterSpec("触发类型", ":TRIGger:MODE", "choice", ("EDGE", "PULSe", "SLOPe", "VIDeo", "PATTern", "DURation", "TIMeout", "RUNT", "WINDow", "DELay", "SETup", "NEDGe", "RS232", "IIC", "SPI", "CAN", "LIN")),
    ParameterSpec("触发扫描", ":TRIGger:SWEep", "choice", ("AUTO", "NORMal", "SINGle")),
    ParameterSpec("触发耦合", ":TRIGger:COUPling", "choice", ("DC", "AC", "LFR", "HFR")),
    ParameterSpec("触发释抑 s", ":TRIGger:HOLDoff", "number"),
    ParameterSpec("噪声抑制", ":TRIGger:NREJect", "bool"),
    ParameterSpec("边沿源", ":TRIGger:EDGE:SOURce", "choice", ("CHAN1", "CHAN2", "CHAN3", "CHAN4", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15", "EXT")),
    ParameterSpec("边沿斜率", ":TRIGger:EDGE:SLOPe", "choice", ("POSitive", "NEGative", "RFALl")),
    ParameterSpec("边沿电平 V", ":TRIGger:EDGE:LEVel", "number"),
    ParameterSpec("脉宽源", ":TRIGger:PULSe:SOURce", "choice", ("CHAN1", "CHAN2", "CHAN3", "CHAN4", "D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15")),
    ParameterSpec("脉宽极性", ":TRIGger:PULSe:POLarity", "choice", ("POSitive", "NEGative")),
    ParameterSpec("脉宽条件", ":TRIGger:PULSe:WHEN", "choice", ("GREater", "LESS", "GLESs")),
    ParameterSpec("脉宽上限 s", ":TRIGger:PULSe:UWIDth", "number"),
    ParameterSpec("脉宽下限 s", ":TRIGger:PULSe:LWIDth", "number"),
    ParameterSpec("脉宽电平 V", ":TRIGger:PULSe:LEVel", "number"),
)

ANALOG_SOURCES = ("CHAN1", "CHAN2", "CHAN3", "CHAN4")
MATH_SOURCES = ("MATH1", "MATH2", "MATH3", "MATH4")
DIGITAL_SOURCES = tuple(f"D{index}" for index in range(16))
WAVEFORM_SOURCES = ANALOG_SOURCES + MATH_SOURCES + DIGITAL_SOURCES
MEASUREMENT_SOURCES = ANALOG_SOURCES + MATH_SOURCES + DIGITAL_SOURCES

MEASUREMENT_ITEMS = (
    "VMAX", "VMIN", "VPP", "VTOP", "VBASE", "VAMP", "VAVG", "VRMS",
    "OVERSHOOT", "PRESHOOT", "MAREA", "MPAREA", "PERIOD", "FREQUENCY",
    "RTIME", "FTIME", "PWIDTH", "NWIDTH", "PDUTY", "NDUTY", "TVMAX",
    "TVMIN", "PSLEWRATE", "NSLEWRATE", "VUPPER", "VMID", "VLOWER",
    "VARIANCE", "PVRMS", "PPULSES", "NPULSES", "PEDGES", "NEDGES",
    "RRDELAY", "RFDELAY", "FRDELAY", "FFDELAY", "RRPHASE", "RFPHASE",
    "FRPHASE", "FFPHASE", "ACRMS",
)

DIGITAL_MEASUREMENT_ITEMS = {
    "PERIOD", "FREQUENCY", "PWIDTH", "NWIDTH", "PDUTY", "NDUTY",
    "RRDELAY", "RFDELAY", "FRDELAY", "FFDELAY", "RRPHASE", "RFPHASE",
    "FRPHASE", "FFPHASE",
}
DUAL_SOURCE_MEASUREMENT_ITEMS = {
    "RRDELAY", "RFDELAY", "FRDELAY", "FFDELAY", "RRPHASE", "RFPHASE",
    "FRPHASE", "FFPHASE",
}


class DHO924TerminalApp:
    TABS = (
        "总览",
        "通道",
        "采集",
        "时基",
        "触发",
        "测量",
        "波形/截图",
        "数字/分析",
        "MHO/AFG",
        "系统",
        "SCPI控制台",
    )

    def __init__(
        self,
        host: str | None,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
        *,
        discovery_subnets: Sequence[str] = (),
        scan_timeout: float = DEFAULT_SCAN_TIMEOUT,
        cache_path: Path | str | None = DEFAULT_CACHE_PATH,
        auto_discover: bool = True,
    ) -> None:
        self.requested_host = host
        self.host = host or DEFAULT_HOST
        self.port = int(port)
        self.timeout = float(timeout)
        self.discovery_subnets = tuple(discovery_subnets)
        self.scan_timeout = float(scan_timeout)
        self.cache_path = cache_path
        self.auto_discover = auto_discover
        self.instrument = RigolDHO(self.host, self.port, self.timeout)
        self.terminal: TerminalSession | None = None
        self.running = True
        self.tab_index = 0
        self.channel = 1
        self.selected = 0
        self.scroll = 0
        self.identity = "未连接"
        self.scope_compatible = True
        self.channel_cache: dict[int, dict[str, Any]] = {index: {} for index in range(1, 5)}
        self.acquisition_cache: dict[str, Any] = {}
        self.timebase_cache: dict[str, Any] = {}
        self.trigger_cache: dict[str, Any] = {}
        self.afg_cache: dict[int, dict[str, Any]] = {1: {}, 2: {}}
        self.last_waveform: WaveformData | None = None
        self.logs: list[str] = []
        self.status = "方向键/鼠标选择，Enter执行，1-4切换通道，F5刷新，Esc/Q退出"
        self._menu_rows: dict[int, int] = {}
        self._tab_ranges: list[tuple[int, int, int]] = []
        self._overlay_title: str | None = None
        self._overlay_lines: list[str] = []
        self._overlay_selected = 0
        self._overlay_rows: dict[int, int] = {}
        self._overlay_wide = False
        self._prompt_buffer: str | None = None
        self._prompt_cursor = 0
        self._history: list[str] = []
        self._scan_overlay_active = False
        self._scan_last_render = 0.0
        self._scan_last_found = -1

    def run(self) -> int:
        with TerminalSession() as terminal:
            self.terminal = terminal
            self._connect_initially()
            if os.environ.get("DHO_TUI_TEST") == "1":
                self._render()
                return 0
            while self.running:
                self._render()
                event = terminal.read_event(0.15)
                if event.kind != "timeout":
                    self._handle_event(event)
        self.instrument.close()
        return 0

    def _connect_initially(self) -> None:
        self._connect_with_discovery()

    def _choose_discovered_scope(
        self, candidates: Sequence[DiscoveredScope]
    ) -> DiscoveredScope | None:
        if not candidates or self.terminal is None:
            return None
        last = load_last_scope(self.cache_path)
        labels = [
            (
                "★ 上次设备  "
                if last and identity_key(candidate.identity) == identity_key(last.identity)
                else ""
            )
            + candidate.display_name()
            for candidate in candidates
        ]
        selected = self.prompt_choice(
            f"扫描完成：发现 {len(labels)} 台 SCPI 设备，必须手动选择",
            labels,
            wide=True,
        )
        return None if selected is None else candidates[labels.index(selected)]

    def _show_scan_progress(self, progress: DiscoveryProgress) -> None:
        if self.terminal is None:
            return
        now = time.monotonic()
        finished = progress.completed >= progress.total
        found_changed = len(progress.candidates) != self._scan_last_found
        if not finished and not found_changed and now - self._scan_last_render < 0.05:
            return
        self._scan_last_render = now
        self._scan_last_found = len(progress.candidates)
        self._scan_overlay_active = True
        width = 32
        ratio = 1.0 if progress.total == 0 else progress.completed / progress.total
        filled = min(width, max(0, int(round(width * ratio))))
        bar = "█" * filled + "░" * (width - filled)
        current = (
            f"{progress.current_host}:{progress.current_port}"
            if progress.current_host and progress.current_port
            else "准备扫描"
        )
        _, height = self.terminal.size()
        visible = max(1, height - 13)
        device_lines = [
            f"  {index:02d}. {candidate.display_name()}"
            for index, candidate in enumerate(progress.candidates[-visible:], 1)
        ]
        self._overlay_title = "正在扫描 SCPI 示波器"
        self._overlay_wide = True
        self._overlay_lines = [
            f"进度: {progress.completed}/{progress.total} ({progress.percent:.1f}%)",
            f"[{bar}]",
            f"当前: {current}",
            f"已发现: {len(progress.candidates)} 台",
            "",
            *(device_lines or ["  暂未发现设备"]),
        ]
        self._overlay_selected = -1
        self._render()

    def _clear_scan_progress(self) -> None:
        self._scan_overlay_active = False
        self._scan_last_render = 0.0
        self._scan_last_found = -1
        self._overlay_title = None
        self._overlay_lines = []
        self._overlay_selected = 0
        self._overlay_wide = False

    def _connect_candidate(self, candidate: DiscoveredScope, source: str) -> bool:
        same_endpoint = (
            self.instrument.is_connected
            and self.instrument.host == candidate.host
            and self.instrument.port == candidate.port
        )
        replacement = self.instrument if same_endpoint else RigolDHO(
            candidate.host, candidate.port, self.timeout
        )
        try:
            if same_endpoint:
                replacement.reconnect()
            else:
                replacement.connect()
            identity = replacement.identify()
        except Exception as exc:
            replacement.close()
            if same_endpoint:
                self.identity = "未连接"
            self._log(
                f"连接 {candidate.host}:{candidate.port} 失败: {exc}；不会自动重新扫描",
                error=True,
            )
            return False
        if not same_endpoint:
            self.instrument.close()
        self.instrument = replacement
        self.host, self.port, self.identity = candidate.host, candidate.port, identity
        self.scope_compatible = is_supported_scope_identity(identity)
        save_last_scope(DiscoveredScope(self.host, self.port, identity), self.cache_path)
        names = {
            "explicit": "指定地址",
            "last": "上次设备",
            "default": "默认地址",
            "manual-selection": "扫描选择",
            "reconnect": "当前地址",
        }
        self._log(f"已通过{names.get(source, source)}连接 {self.host}:{self.port} {identity}")
        if self.scope_compatible:
            self._safe("刷新设备状态", self._refresh_all, allow_generic=True)
        else:
            self.tab_index = len(self.TABS) - 1
            self.selected = self.scroll = 0
            self._log("该设备不是 RIGOL DHO800/DHO900/MHO900，仅开放原始 SCPI 控制台", error=True)
        return True

    def _connect_with_discovery(self, *, force_scan: bool = False) -> bool:
        scan_started = False

        def progress(progress_value: DiscoveryProgress) -> None:
            nonlocal scan_started
            scan_started = True
            self._show_scan_progress(progress_value)

        try:
            location = locate_scope(
                None if force_scan else self.requested_host,
                self.port,
                self.timeout,
                subnets=self.discovery_subnets,
                scan_timeout=self.scan_timeout,
                cache_path=self.cache_path,
                allow_scan=self.auto_discover or force_scan,
                force_scan=force_scan,
                progress_callback=progress,
            )
        except Exception as exc:
            self._clear_scan_progress()
            self._log(f"设备发现失败: {exc}", error=True)
            return False
        if scan_started:
            self._clear_scan_progress()
        candidate = location.selected
        source = location.source
        if candidate is None and location.candidates:
            candidate = self._choose_discovered_scope(location.candidates)
            source = "manual-selection"
        if candidate is None:
            if location.candidates:
                self._log("扫描结果已取消，未连接任何新设备", error=True)
            elif location.source == "explicit-not-found":
                self._log(
                    f"指定地址 {self.requested_host}:{self.port} 未响应 *IDN?；不会回退",
                    error=True,
                )
            else:
                networks = ", ".join(location.networks) or "无扫描网段"
                self._log(f"未发现 SCPI 设备；扫描范围: {networks}", error=True)
            return False
        if force_scan:
            self.requested_host = None
        return self._connect_candidate(candidate, source)

    def _log(self, message: str, *, error: bool = False) -> None:
        stamp = time.strftime("%H:%M:%S")
        clean = " ".join(str(message).split())
        if len(clean) > 700:
            clean = clean[:697] + "..."
        self.logs.append(f"{stamp} {'ERR' if error else 'OK '} {clean}")
        self.logs = self.logs[-100:]
        self.status = clean

    def _safe(
        self,
        description: str,
        callback: Callable[[], Any],
        *,
        allow_generic: bool = False,
    ) -> Any:
        try:
            if not self.instrument.is_connected:
                raise ScopeError(
                    "示波器未连接；请使用“连接/断开设备”或“扫描并选择设备”"
                )
            if not self.scope_compatible and not allow_generic:
                raise ScopeError("当前设备不是已识别的 RIGOL DHO800/DHO900/MHO900")
            result = callback()
            if result is not None:
                self._log(f"{description}: {result}")
            else:
                self._log(f"{description}: 完成")
            return result
        except Exception as exc:
            self._log(f"{description}失败: {exc}", error=True)
            return None

    def _refresh_all(self) -> dict[str, Any]:
        self.identity = self.instrument.identify()
        self.scope_compatible = is_supported_scope_identity(self.identity)
        self.acquisition_cache = self.instrument.acquisition_state()
        self.timebase_cache = self.instrument.timebase_state()
        self.trigger_cache = self.instrument.trigger_state()
        for channel in range(1, 5):
            self.channel_cache[channel] = self.instrument.channel_state(channel)
        save_last_scope(
            DiscoveredScope(self.host, self.port, self.identity), self.cache_path
        )
        return {
            "acquisition": self.acquisition_cache,
            "timebase": self.timebase_cache,
            "trigger": self.trigger_cache,
        }

    def _refresh_channel(self) -> dict[str, Any]:
        state = self.instrument.channel_state(self.channel)
        self.channel_cache[self.channel] = state
        return state

    def _header(self, spec: ParameterSpec, channel: int | None = None) -> str:
        return spec.header.format(ch=self.channel if channel is None else channel)

    def _parameter_action(
        self, spec: ParameterSpec, channel: int | None = None
    ) -> MenuAction:
        header = self._header(spec, channel)

        def handler() -> None:
            current = self._safe(
                f"查询 {spec.label}", lambda: self.instrument.query_parameter(header)
            )
            if current is None:
                current = ""
            if spec.kind == "bool":
                choice = self.prompt_choice(spec.label, ("OFF", "ON"), "ON" if parse_bool_response(str(current)) else "OFF")
                value = choice
            elif spec.kind == "choice":
                choice = self.prompt_choice(spec.label, spec.choices, str(current))
                value = choice
            elif spec.kind == "number":
                value = self.prompt_number(spec.label, str(current))
            else:
                value = self.prompt_text(spec.label, str(current).strip('"'))
            if value is None:
                return
            encoded = scpi_quote(str(value)) if spec.quote else (
                scpi_number(value) if isinstance(value, (int, float)) else str(value)
            )
            response = self._safe(
                f"设置 {spec.label}",
                lambda: (
                    self.instrument.set_parameter(header, encoded),
                    self.instrument.query_parameter(header),
                )[1],
            )
            if response is not None and "CHANnel" in spec.header:
                self._safe("刷新通道", self._refresh_channel)

        return MenuAction(spec.label, handler)

    def _actions(self) -> list[MenuAction]:
        if not self.scope_compatible and self.tab_index != len(self.TABS) - 1:
            return [
                MenuAction("扫描并选择兼容示波器", lambda: self._connect_with_discovery(force_scan=True)),
                MenuAction("跳转到 SCPI 控制台", self._open_console),
            ]
        return (
            self._overview_actions(),
            self._channel_actions(),
            self._acquisition_actions(),
            self._timebase_actions(),
            self._trigger_actions(),
            self._measurement_actions(),
            self._waveform_actions(),
            self._digital_actions(),
            self._afg_actions(),
            self._system_actions(),
            self._console_actions(),
        )[self.tab_index]

    def _overview_actions(self) -> list[MenuAction]:
        def connect_toggle() -> None:
            if self.instrument.is_connected:
                self.instrument.close()
                self.identity = "未连接"
                self._log("连接已关闭")
            else:
                self._connect_candidate(
                    DiscoveredScope(self.host, self.port, self.identity), "reconnect"
                )

        return [
            MenuAction("连接/断开当前设备", connect_toggle, self.identity),
            MenuAction("扫描并选择 SCPI 设备", lambda: self._connect_with_discovery(force_scan=True)),
            MenuAction("刷新全部状态", lambda: self._safe("刷新", self._refresh_all)),
            MenuAction("开始连续采集 RUN", lambda: self._safe("RUN", self.instrument.run)),
            MenuAction("停止采集 STOP", lambda: self._safe("STOP", self.instrument.stop)),
            MenuAction("单次采集 SINGLE", lambda: self._safe("SINGLE", self.instrument.single)),
            MenuAction("强制触发 TFORCE", lambda: self._safe("强制触发", self.instrument.force_trigger)),
            MenuAction("自动设置 AUTOSET", lambda: self.confirm("自动设置将改变通道/时基/触发，是否继续？") and self._safe("AUTOSET", self.instrument.autoset), dangerous=True),
            MenuAction("读取当前波形到预览", self._capture_preview),
            MenuAction("保存屏幕截图", self._save_screenshot),
            MenuAction("读取系统错误", lambda: self._safe("系统错误", self.instrument.system_error, allow_generic=True)),
            MenuAction("清除状态寄存器 *CLS", lambda: self._safe("清除状态", self.instrument.clear_status, allow_generic=True)),
            MenuAction("恢复出厂状态 *RST", lambda: self.confirm("*RST 将重置示波器全部设置，是否继续？") and self._safe("复位", self.instrument.reset, allow_generic=True), dangerous=True),
        ]

    def _channel_actions(self) -> list[MenuAction]:
        return [
            MenuAction(f"刷新 CH{self.channel}", lambda: self._safe("刷新通道", self._refresh_channel)),
            *[self._parameter_action(spec) for spec in CHANNEL_SPECS],
        ]

    def _acquisition_actions(self) -> list[MenuAction]:
        specs = MHO_ACQUISITION_SPECS if is_mho_scope_identity(self.identity) else ACQUISITION_SPECS
        return [
            MenuAction("刷新采集状态", lambda: self._safe("采集状态", self.instrument.acquisition_state)),
            MenuAction("查询实际采样率", lambda: self._safe("采样率", lambda: format_engineering(self.instrument.query_float(":ACQuire:SRATe?"), "Sa/s"))),
            *[self._parameter_action(spec) for spec in specs],
            MenuAction("开始连续采集", lambda: self._safe("RUN", self.instrument.run)),
            MenuAction("停止采集", lambda: self._safe("STOP", self.instrument.stop)),
            MenuAction("单次采集", lambda: self._safe("SINGLE", self.instrument.single)),
        ]

    def _timebase_actions(self) -> list[MenuAction]:
        return [
            MenuAction("刷新时基状态", lambda: self._safe("时基状态", self.instrument.timebase_state)),
            *[self._parameter_action(spec) for spec in TIMEBASE_SPECS],
        ]

    def _trigger_actions(self) -> list[MenuAction]:
        def generic_trigger_parameter() -> None:
            header = self.prompt_text("触发参数头，例如 :TRIGger:CAN:BAUD")
            if not header:
                return
            current = self._safe("查询触发参数", lambda: self.instrument.query_parameter(header))
            value = self.prompt_text("新值", "" if current is None else str(current))
            if value is not None:
                self._safe("设置触发参数", lambda: (self.instrument.set_parameter(header, value), self.instrument.query_parameter(header))[1])

        return [
            MenuAction("刷新触发状态", lambda: self._safe("触发状态", self.instrument.trigger_state)),
            MenuAction("强制触发", lambda: self._safe("强制触发", self.instrument.force_trigger)),
            *[self._parameter_action(spec) for spec in TRIGGER_SPECS],
            MenuAction("其他触发类型参数", generic_trigger_parameter),
        ]

    def _measurement_actions(self) -> list[MenuAction]:
        def measure_any() -> None:
            item = self.prompt_choice("测量项目", MEASUREMENT_ITEMS, "VPP")
            if not item:
                return
            source_choices = (
                MEASUREMENT_SOURCES
                if item in DIGITAL_MEASUREMENT_ITEMS
                else ANALOG_SOURCES + MATH_SOURCES
            )
            source = self.prompt_choice("测量源 A", source_choices, f"CHAN{self.channel}")
            if not item or not source:
                return
            source_b = None
            if item in DUAL_SOURCE_MEASUREMENT_ITEMS:
                source_b = self.prompt_choice("测量源 B", source_choices, "CHAN2")
                if not source_b:
                    return
            value = self._safe(
                f"{source} {item}",
                lambda: self.instrument.measure(item, source, source_b),
            )
            if isinstance(value, float) and invalid_measurement(value):
                self._log(f"{source} {item}: 无有效测量结果 (9.9E37)", error=True)

        def quick(item: str) -> Callable[[], None]:
            def handler() -> None:
                source = f"CHAN{self.channel}"
                value = self._safe(f"{source} {item}", lambda: self.instrument.measure(item, source))
                if isinstance(value, float) and invalid_measurement(value):
                    self._log(f"{source} {item}: 无有效测量结果", error=True)
            return handler

        def configure_counter() -> None:
            enabled = self.prompt_choice("频率计", ("OFF", "ON"), "ON")
            source = self.prompt_choice("频率计源", ("CHAN1", "CHAN2", "CHAN3", "CHAN4"), f"CHAN{self.channel}")
            mode = self.prompt_choice("频率计模式", ("FREQ", "PERiod", "TOTalize"), "FREQ")
            if enabled and source and mode:
                self._safe("配置频率计", lambda: self.instrument.configure_counter(enabled=enabled, source=source, mode=mode))

        def configure_dvm() -> None:
            enabled = self.prompt_choice("电压表", ("OFF", "ON"), "ON")
            source = self.prompt_choice("电压表源", ("CHAN1", "CHAN2", "CHAN3", "CHAN4"), f"CHAN{self.channel}")
            mode = self.prompt_choice("电压表模式", ("ACRMs", "DCRMs", "DC"), "DC")
            if enabled and source and mode:
                self._safe("配置电压表", lambda: self.instrument.configure_dvm(enabled=enabled, source=source, mode=mode))

        return [
            MenuAction("任意自动测量", measure_any),
            MenuAction("当前通道峰峰值 VPP", quick("VPP")),
            MenuAction("当前通道频率", quick("FREQUENCY")),
            MenuAction("当前通道有效值 VRMS", quick("VRMS")),
            MenuAction("当前通道最大值 VMAX", quick("VMAX")),
            MenuAction("当前通道最小值 VMIN", quick("VMIN")),
            MenuAction("查看频率计状态", lambda: self._safe("频率计", self.instrument.counter_state)),
            MenuAction("配置频率计", configure_counter),
            MenuAction("查看数字电压表状态", lambda: self._safe("电压表", self.instrument.dvm_state)),
            MenuAction("配置数字电压表", configure_dvm),
            MenuAction("光标模式", self._parameter_action(ParameterSpec("光标模式", ":CURSor:MODE", "choice", ("OFF", "MANual", "TRACk", "XY"))).handler),
            MenuAction("测量指示器", self._parameter_action(ParameterSpec("测量指示器", ":CURSor:MEASure:INDicator", "bool")).handler),
        ]

    def _capture_preview(self) -> None:
        def capture() -> WaveformData:
            displayed = self.instrument.displayed_channels()
            for channel in range(1, 5):
                self.channel_cache[channel]["display"] = channel in displayed
            if self.channel not in displayed:
                if len(displayed) == 1:
                    previous = self.channel
                    self.channel = displayed[0]
                    self._log(
                        f"CH{previous} 已关闭，自动切换到唯一开启的 CH{self.channel}"
                    )
                elif displayed:
                    choices = ", ".join(f"CH{channel}" for channel in displayed)
                    raise ScopeError(
                        f"CH{self.channel} 已关闭；请先选择已开启通道 {choices}"
                    )
                else:
                    raise ScopeError("所有模拟通道均已关闭，无法读取屏幕波形")
            source = f"CHAN{self.channel}"
            return self.instrument.read_waveform(
                source=source,
                mode="NORMal",
                data_format="WORD",
                points=1000,
            )

        waveform = self._safe(
            "读取屏幕波形",
            capture,
        )
        if isinstance(waveform, WaveformData):
            self.last_waveform = waveform
            self._log(
                f"{waveform.source} 波形 {len(waveform.volts)} 点，"
                f"Vpp={format_engineering(waveform.peak_to_peak, 'V')}"
            )

    def _save_waveform_csv(self) -> None:
        source = self.prompt_choice("波形源", WAVEFORM_SOURCES, f"CHAN{self.channel}")
        if not source:
            return
        default = str(Path.cwd() / f"{source.lower()}_waveform.csv")
        path = self.prompt_text("CSV 保存路径", default)
        if not path:
            return
        waveform = self._safe(
            f"读取 {source}",
            lambda: self.instrument.read_waveform(source=source, mode="NORMal", data_format="WORD", points=1000),
        )
        if isinstance(waveform, WaveformData):
            saved = waveform.save_csv(path)
            self.last_waveform = waveform
            self._log(f"波形已保存: {saved.resolve()}")

    def _save_screenshot(self) -> None:
        default = str(Path.cwd() / "dho924_screen.png")
        path = self.prompt_text("截图保存路径 (.png/.bmp/.jpg)", default)
        if not path:
            return
        target = Path(path)
        suffix = target.suffix.lower()
        image_format = {".bmp": "BMP", ".jpg": "JPG", ".jpeg": "JPG"}.get(suffix, "PNG")
        payload = self._safe("读取屏幕截图", lambda: self.instrument.screenshot(image_format))
        if isinstance(payload, bytes):
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
            self._log(f"截图已保存: {target.resolve()} ({len(payload)} 字节)")

    def _waveform_actions(self) -> list[MenuAction]:
        waveform_specs = (
            ParameterSpec("波形源", ":WAVeform:SOURce", "choice", WAVEFORM_SOURCES),
            ParameterSpec("读取模式", ":WAVeform:MODE", "choice", ("NORMal", "MAXimum", "RAW")),
            ParameterSpec("返回格式", ":WAVeform:FORMat", "choice", ("WORD", "BYTE", "ASCii")),
            ParameterSpec("读取点数", ":WAVeform:POINts", "number"),
            ParameterSpec("起始点", ":WAVeform:STARt", "number"),
            ParameterSpec("终止点", ":WAVeform:STOP", "number"),
        )
        return [
            MenuAction("读取当前通道到波形预览", self._capture_preview),
            MenuAction("保存波形 CSV", self._save_waveform_csv),
            MenuAction("保存屏幕截图", self._save_screenshot),
            MenuAction("查看波形前导参数", lambda: self._safe("波形前导", self.instrument.waveform_preamble)),
            *[self._parameter_action(spec) for spec in waveform_specs],
        ]

    def _digital_actions(self) -> list[MenuAction]:
        specs = (
            ParameterSpec("逻辑分析总开关", ":LA:ENABle", "bool"),
            ParameterSpec("活动数字通道", ":LA:ACTive", "choice", DIGITAL_SOURCES + ("NONE",)),
            ParameterSpec("数字通道显示尺寸", ":LA:SIZE", "choice", ("SMALl", "MEDium", "LARGe")),
            ParameterSpec("POD1 显示", ":LA:POD1:DISPlay", "bool"),
            ParameterSpec("POD1 阈值 V", ":LA:POD1:THReshold", "number"),
            ParameterSpec("POD2 显示", ":LA:POD2:DISPlay", "bool"),
            ParameterSpec("POD2 阈值 V", ":LA:POD2:THReshold", "number"),
            ParameterSpec("BUS1 显示", ":BUS1:DISPlay", "bool"),
            ParameterSpec("BUS1 类型", ":BUS1:MODE", "choice", ("PARallel", "RS232", "IIC", "SPI", "CAN", "LIN")),
            ParameterSpec("BUS1 格式", ":BUS1:FORMat", "choice", ("HEX", "ASCii", "DECimal", "BINary")),
            ParameterSpec("BUS1 标签", ":BUS1:LABel", "text", quote=True),
            ParameterSpec("BUS2 显示", ":BUS2:DISPlay", "bool"),
            ParameterSpec("BUS2 类型", ":BUS2:MODE", "choice", ("PARallel", "RS232", "IIC", "SPI", "CAN", "LIN")),
            ParameterSpec("直方图开关", ":HISTogram:ENABle", "bool"),
            ParameterSpec("直方图类型", ":HISTogram:TYPE", "choice", ("HORizontal", "VERTical")),
            ParameterSpec("直方图源", ":HISTogram:SOURce", "choice", ANALOG_SOURCES),
            ParameterSpec("搜索开关", ":SEARch:STATe", "bool"),
            ParameterSpec("搜索类型", ":SEARch:MODE", "choice", ("EDGE", "PULSe")),
        )
        return [
            MenuAction("刷新数字通道状态", lambda: self._safe("数字通道", self.instrument.digital_state)),
            *[self._parameter_action(spec) for spec in specs],
            MenuAction("查询 BUS1 解码数据", lambda: self._safe("BUS1 DATA", lambda: self.instrument.query(":BUS1:DATA?"))),
            MenuAction("查询搜索结果数量", lambda: self._safe("搜索结果", lambda: self.instrument.query(":SEARch:COUNt?"))),
        ]

    def _afg_actions(self) -> list[MenuAction]:
        if not is_mho_scope_identity(self.identity):
            return [
                MenuAction(
                    "当前设备不是 MHO900；本页不发送 AFG 命令",
                    lambda: self._log("MHO/AFG 页仅适用于支持内置 AFG 选件的 MHO900"),
                ),
                MenuAction("跳转到 SCPI 控制台", self._open_console),
            ]

        channel = self.channel if self.channel in {1, 2} else 1

        def refresh() -> dict[str, Any]:
            state = self.instrument.afg_state(channel)
            self.afg_cache[channel] = state
            return state

        def set_output() -> None:
            current = self._safe(
                f"查询 AFG{channel} 输出",
                lambda: self.instrument.query_bool(
                    f":SOURce{channel}:OUTPut:STATe?"
                ),
            )
            if current is None:
                return
            choice = self.prompt_choice(
                f"AFG{channel} 输出",
                ("OFF", "ON"),
                "ON" if current else "OFF",
            )
            if choice is None:
                return
            if choice == "ON" and not current and not self.confirm(
                f"将开启 MHO AFG{channel} 物理输出，确认外部连接安全后继续？"
            ):
                return
            result = self._safe(
                f"设置 AFG{channel} 输出 {choice}",
                lambda: self.instrument.configure_afg(channel, output=choice),
            )
            if isinstance(result, dict):
                self.afg_cache[channel] = result

        return [
            MenuAction(f"刷新 AFG{channel} 状态", lambda: self._safe(f"AFG{channel}", refresh)),
            MenuAction("查询 MHO AFG 选件", lambda: self._safe("AFG选件", self.instrument.afg_option_state)),
            MenuAction(f"AFG{channel} 输出开关", set_output, dangerous=True),
            MenuAction(
                "两路 AFG 相位同步",
                lambda: self._safe(
                    "AFG相位同步",
                    lambda: self.instrument.synchronize_afg_phase(channel),
                ),
            ),
            *[self._parameter_action(spec, channel) for spec in MHO_AFG_SPECS],
        ]

    def _system_actions(self) -> list[MenuAction]:
        specs = (
            ParameterSpec("显示类型", ":DISPlay:TYPE", "choice", ("VECTors",)),
            ParameterSpec("波形亮度 %", ":DISPlay:WBRightness", "number"),
            ParameterSpec("网格类型", ":DISPlay:GRID", "choice", ("FULL", "HALF", "NONE")),
            ParameterSpec("网格亮度 %", ":DISPlay:GBRightness", "number"),
            ParameterSpec("标尺显示", ":DISPlay:RULers", "bool"),
            ParameterSpec("色温显示", ":DISPlay:COLor", "bool"),
            ParameterSpec("蜂鸣器", ":SYSTem:BEEPer", "bool"),
            ParameterSpec("前面板锁定", ":SYSTem:LOCKed", "bool"),
            ParameterSpec("开机设置", ":SYSTem:PON", "choice", ("LATest", "DEFault")),
        )
        return [
            MenuAction("设备身份 *IDN?", lambda: self._safe("身份", self.instrument.identify, allow_generic=True)),
            MenuAction("SCPI 版本", lambda: self._safe("SCPI版本", self.instrument.scpi_version, allow_generic=True)),
            MenuAction("模块信息", lambda: self._safe("模块", lambda: self.instrument.query(":SYSTem:MODules?"))),
            MenuAction("内存容量", lambda: self._safe("内存", lambda: self.instrument.query(":SYSTem:RAMount?"))),
            MenuAction("显存容量", lambda: self._safe("显存", lambda: self.instrument.query(":SYSTem:GAMount?"))),
            MenuAction("显示状态", lambda: self._safe("显示", self.instrument.display_state)),
            *[self._parameter_action(spec) for spec in specs],
            MenuAction("保存屏幕截图", self._save_screenshot),
            MenuAction("读取系统错误", lambda: self._safe("系统错误", self.instrument.system_error, allow_generic=True)),
            MenuAction("清除状态 *CLS", lambda: self._safe("清除状态", self.instrument.clear_status, allow_generic=True)),
            MenuAction("恢复出厂状态 *RST", lambda: self.confirm("*RST 将重置全部设置，是否继续？") and self._safe("复位", self.instrument.reset, allow_generic=True), dangerous=True),
        ]

    def _open_console(self) -> None:
        self.tab_index = len(self.TABS) - 1
        self.selected = self.scroll = 0

    def _console_actions(self) -> list[MenuAction]:
        def send_command() -> None:
            command = self.prompt_text("SCPI 命令")
            if not command:
                return
            command = command.strip()
            self._history.append(command)
            self._history = self._history[-100:]
            if "?" in command:
                response = self._safe(
                    "SCPI 查询", lambda: self.instrument.query(command), allow_generic=True
                )
                if response is not None:
                    self.show_text(command, str(response))
            else:
                upper = command.upper()
                dangerous = upper.startswith(("*RST", ":SYST:RESET", ":AUT", ":RUN", ":STOP")) or bool(
                    re.match(
                        r"^:SOUR(?:CE)?[12]:OUTP(?:UT)?:STAT(?:E)?\s+(?:ON|1)\b",
                        upper,
                    )
                )
                if dangerous and not self.confirm(f"发送 {command}？"):
                    return
                self._safe(
                    "SCPI 写入", lambda: self.instrument.write(command), allow_generic=True
                )

        return [
            MenuAction("输入并发送 SCPI 命令", send_command),
            MenuAction("查看命令历史", lambda: self.show_text("SCPI 历史", "\n".join(self._history) if self._history else "暂无历史")),
            MenuAction("设备身份 *IDN?", lambda: self._safe("身份", self.instrument.identify, allow_generic=True)),
            MenuAction("系统错误 :SYST:ERR?", lambda: self._safe("错误", self.instrument.system_error, allow_generic=True)),
            MenuAction("操作完成 *OPC?", lambda: self._safe("OPC", lambda: self.instrument.query("*OPC?"), allow_generic=True)),
            MenuAction("扫描并选择设备", lambda: self._connect_with_discovery(force_scan=True)),
        ]

    def _render(self) -> None:
        if self.terminal is None:
            return
        width, height = self.terminal.size()
        lines = [" " * width for _ in range(height)]
        model = DiscoveredScope(self.host, self.port, self.identity).model or "DHO/MHO"
        title = f"{ANSI_BOLD}{ANSI_CYAN}RIGOL {model} 终端控制台{ANSI_RESET}  {self.host}:{self.port}"
        lines[0] = fit_display(title, width)
        lines[1] = fit_display(f"设备: {self.identity}", width)
        channel_parts = []
        for channel in range(1, 5):
            state = self.channel_cache.get(channel, {})
            marker = "▶" if channel == self.channel else " "
            color = ANSI_GREEN if state.get("display") else ANSI_DIM
            channel_parts.append(
                f"{color}{marker}CH{channel} {'ON' if state.get('display') else 'OFF'} "
                f"{format_engineering(state.get('scale', '?'), 'V/div')}{ANSI_RESET}"
            )
        lines[2] = fit_display("  ".join(channel_parts), width)
        cursor = 1
        tab_text = ""
        self._tab_ranges.clear()
        for index, tab in enumerate(self.TABS):
            label = f" {tab} "
            decorated = ANSI_REVERSE + label + ANSI_RESET if index == self.tab_index else label
            start = cursor
            cursor += display_width(label)
            self._tab_ranges.append((start, cursor - 1, index))
            tab_text += decorated
        lines[3] = fit_display(tab_text, width)
        content_top = 4
        content_bottom = height - 4
        content_height = max(1, content_bottom - content_top)
        left_width = max(38, min(width - 34, width * 56 // 100))
        right_width = width - left_width - 1
        actions = self._actions()
        self.selected = min(max(0, self.selected), max(0, len(actions) - 1))
        page_size = content_height
        if self.selected < self.scroll:
            self.scroll = self.selected
        if self.selected >= self.scroll + page_size:
            self.scroll = self.selected - page_size + 1
        self.scroll = min(max(0, self.scroll), max(0, len(actions) - page_size))
        self._menu_rows.clear()
        right_lines = self._status_panel(right_width, content_height)
        for row in range(content_height):
            index = self.scroll + row
            y = content_top + row
            if index < len(actions):
                action = actions[index]
                marker = "⚠" if action.dangerous else " "
                source = f"{marker} {index + 1:02d}. {action.label}"
                if action.value:
                    source += f"  [{action.value}]"
                if index == self.selected:
                    source = ANSI_REVERSE + source + ANSI_RESET
                self._menu_rows[y + 1] = index
                left = fit_display(source, left_width)
            else:
                left = " " * left_width
            right = right_lines[row] if row < len(right_lines) else ""
            lines[y] = fit_display(left + "│" + right, width)
        lines[height - 4] = fit_display("─" * width, width)
        lines[height - 3] = fit_display(f"状态: {self.status}", width)
        log1 = self.logs[-2] if len(self.logs) >= 2 else ""
        log2 = self.logs[-1] if self.logs else ""
        lines[height - 2] = fit_display(log1, width)
        lines[height - 1] = fit_display(log2, width)
        if self._overlay_title is not None:
            lines = self._render_overlay(lines, width, height)
        self.terminal.draw("\n".join(lines))

    def _status_panel(self, width: int, height: int) -> list[str]:
        lines = [f"{ANSI_BOLD}当前状态{ANSI_RESET}"]
        acquisition = self.acquisition_cache
        timebase = self.timebase_cache
        trigger = self.trigger_cache
        if acquisition:
            lines.extend(
                [
                    f"采集: {acquisition.get('type', '?')}",
                    f"采样率: {format_engineering(acquisition.get('sample_rate', '?'), 'Sa/s')}",
                    f"深度: {format_engineering(acquisition.get('memory_depth', '?'), 'pts')}",
                ]
            )
            if "bits" in acquisition:
                lines.append(f"分辨率: {acquisition.get('bits')} bit")
        if timebase:
            lines.extend(
                [
                    f"时基: {format_engineering(timebase.get('scale', '?'), 's/div')}",
                    f"偏移: {format_engineering(timebase.get('offset', '?'), 's')}",
                ]
            )
        if trigger:
            lines.extend(
                [
                    f"触发: {trigger.get('mode', '?')} / {trigger.get('status', '?')}",
                    f"扫描: {trigger.get('sweep', '?')}",
                ]
            )
        if self.TABS[self.tab_index] == "MHO/AFG" and is_mho_scope_identity(self.identity):
            afg_channel = self.channel if self.channel in {1, 2} else 1
            state = self.afg_cache.get(afg_channel, {})
            lines.extend(
                [
                    "",
                    f"{ANSI_BOLD}AFG{afg_channel}{ANSI_RESET}",
                    f"输出: {'ON' if state.get('output') else 'OFF'}",
                    f"波形: {state.get('function', '?')}",
                    f"频率: {format_engineering(state.get('frequency', '?'), 'Hz')}",
                    f"幅度: {format_engineering(state.get('amplitude', '?'), 'Vpp')}",
                    f"偏置: {format_engineering(state.get('offset', '?'), 'V')}",
                ]
            )
        else:
            state = self.channel_cache.get(self.channel, {})
            lines.extend(
                [
                    "",
                    f"{ANSI_BOLD}CH{self.channel}{ANSI_RESET}",
                    f"显示: {'ON' if state.get('display') else 'OFF'}",
                    f"档位: {format_engineering(state.get('scale', '?'), 'V/div')}",
                    f"偏移: {format_engineering(state.get('offset', '?'), 'V')}",
                    f"耦合/探头: {state.get('coupling', '?')} / {state.get('probe', '?')}X",
                ]
            )
        if self.last_waveform:
            preview_height = max(5, height - len(lines) - 4)
            lines.extend(["", f"{ANSI_BOLD}{self.last_waveform.source} 波形预览{ANSI_RESET}"])
            lines.extend(
                waveform_ascii_preview(
                    self.last_waveform.volts, max(16, width - 1), preview_height
                )
            )
            lines.append(
                f"Vpp: {format_engineering(self.last_waveform.peak_to_peak, 'V')}"
            )
        else:
            lines.extend(["", ANSI_DIM + "尚未读取波形；在总览或波形页执行读取。" + ANSI_RESET])
        return [fit_display(line, width) for line in (lines + [""] * height)[:height]]

    def _render_overlay(self, lines: list[str], width: int, height: int) -> list[str]:
        box_width = max(4, width - 4) if self._overlay_wide else min(max(48, width * 2 // 3), width - 4)
        content = list(self._overlay_lines)
        if self._prompt_buffer is not None:
            before = self._prompt_buffer[: self._prompt_cursor]
            current = self._prompt_buffer[self._prompt_cursor : self._prompt_cursor + 1] or " "
            after = self._prompt_buffer[self._prompt_cursor + (1 if self._prompt_cursor < len(self._prompt_buffer) else 0) :]
            content.append("> " + before + ANSI_REVERSE + current + ANSI_RESET + after)
        box_height = min(len(content) + 4, height - 4)
        top = max(1, (height - box_height) // 2)
        left = max(1, (width - box_width) // 2)
        lines[top] = fit_display(" " * left + "┌" + "─" * (box_width - 2) + "┐", width)
        title = f" {self._overlay_title or ''} "
        lines[top + 1] = fit_display(" " * left + "│" + fit_display(ANSI_BOLD + title + ANSI_RESET, box_width - 2) + "│", width)
        self._overlay_rows.clear()
        for index in range(box_height - 3):
            source = content[index] if index < len(content) else ""
            if self._prompt_buffer is None and index < len(self._overlay_lines):
                self._overlay_rows[top + 2 + index + 1] = index
                if index == self._overlay_selected:
                    source = ANSI_REVERSE + source + ANSI_RESET
            lines[top + 2 + index] = fit_display(" " * left + "│" + fit_display(source, box_width - 2) + "│", width)
        lines[top + box_height - 1] = fit_display(" " * left + "└" + "─" * (box_width - 2) + "┘", width)
        return lines

    def _handle_event(self, event: TerminalEvent) -> None:
        actions = self._actions()
        if event.kind == "mouse" and event.key == "press":
            if event.button == 64:
                self.selected = max(0, self.selected - 3)
                return
            if event.button == 65:
                self.selected = min(max(0, len(actions) - 1), self.selected + 3)
                return
            for start, end, index in self._tab_ranges:
                if event.y == 4 and start <= event.x <= end:
                    self.tab_index = index
                    self.selected = self.scroll = 0
                    return
            if event.y in self._menu_rows:
                self.selected = self._menu_rows[event.y]
                if event.button == 0 and self.selected < len(actions):
                    actions[self.selected].handler()
                return
        if event.kind == "text":
            key = event.text.lower()
            if key == "q":
                self.running = False
            elif key in {"1", "2", "3", "4"}:
                selected_channel = int(key)
                if self.TABS[self.tab_index] == "MHO/AFG":
                    if selected_channel in {1, 2}:
                        self.channel = selected_channel
                        self._log(f"切换到 AFG{self.channel}")
                    else:
                        self._log("MHO 内置 AFG 仅有通道 1 和 2", error=True)
                else:
                    self.channel = selected_channel
                    self._log(f"切换到 CH{self.channel}")
            elif key == "r":
                self._safe("刷新", self._refresh_all)
            elif key == "c":
                self._open_console()
            elif key in {"j", "s"}:
                self.selected = min(max(0, len(actions) - 1), self.selected + 1)
            elif key in {"k", "w"}:
                self.selected = max(0, self.selected - 1)
            return
        if event.kind != "key":
            return
        if event.key in {"escape", "ctrl_c"}:
            self.running = False
        elif event.key == "up":
            self.selected = max(0, self.selected - 1)
        elif event.key == "down":
            self.selected = min(max(0, len(actions) - 1), self.selected + 1)
        elif event.key == "pageup":
            self.selected = max(0, self.selected - 8)
        elif event.key == "pagedown":
            self.selected = min(max(0, len(actions) - 1), self.selected + 8)
        elif event.key == "left":
            self.tab_index = (self.tab_index - 1) % len(self.TABS)
            self.selected = self.scroll = 0
        elif event.key in {"right", "tab"}:
            self.tab_index = (self.tab_index + 1) % len(self.TABS)
            self.selected = self.scroll = 0
        elif event.key in {"enter", "f2"} and actions:
            actions[self.selected].handler()
        elif event.key in {"f5", "ctrl_l"}:
            self._safe("刷新", self._refresh_all)

    def prompt_text(self, title: str, initial: str = "") -> str | None:
        assert self.terminal is not None
        self._overlay_wide = False
        self._overlay_title = title + "  (Enter确认 / Esc取消)"
        self._overlay_lines = []
        self._prompt_buffer = initial
        self._prompt_cursor = len(initial)
        while True:
            self._render()
            event = self.terminal.read_event(0.2)
            if event.kind == "text":
                self._prompt_buffer = self._prompt_buffer[: self._prompt_cursor] + event.text + self._prompt_buffer[self._prompt_cursor :]
                self._prompt_cursor += len(event.text)
            elif event.kind == "key":
                if event.key == "enter":
                    result = self._prompt_buffer
                    break
                if event.key in {"escape", "ctrl_c"}:
                    result = None
                    break
                if event.key == "backspace" and self._prompt_cursor > 0:
                    self._prompt_buffer = self._prompt_buffer[: self._prompt_cursor - 1] + self._prompt_buffer[self._prompt_cursor :]
                    self._prompt_cursor -= 1
                elif event.key == "delete" and self._prompt_cursor < len(self._prompt_buffer):
                    self._prompt_buffer = self._prompt_buffer[: self._prompt_cursor] + self._prompt_buffer[self._prompt_cursor + 1 :]
                elif event.key == "left":
                    self._prompt_cursor = max(0, self._prompt_cursor - 1)
                elif event.key == "right":
                    self._prompt_cursor = min(len(self._prompt_buffer), self._prompt_cursor + 1)
                elif event.key == "home":
                    self._prompt_cursor = 0
                elif event.key == "end":
                    self._prompt_cursor = len(self._prompt_buffer)
        self._overlay_title = None
        self._overlay_lines = []
        self._prompt_buffer = None
        self._prompt_cursor = 0
        return result

    def prompt_number(self, title: str, initial: str = "") -> float | None:
        while True:
            result = self.prompt_text(title + "（支持 10n/500m/1k 等）", initial)
            if result is None:
                return None
            try:
                return parse_engineering_number(result)
            except ValueError as exc:
                self.show_text("输入错误", str(exc))

    def prompt_choice(
        self,
        title: str,
        choices: Sequence[str],
        current: str | None = None,
        *,
        wide: bool = False,
    ) -> str | None:
        assert self.terminal is not None
        if not choices:
            return None
        upper = [choice.upper() for choice in choices]
        try:
            selected = upper.index((current or "").upper())
        except ValueError:
            selected = 0
        self._overlay_wide = wide
        number_width = max(2, len(str(len(choices))))
        all_lines = [f" {index + 1:0{number_width}d}. {choice}" for index, choice in enumerate(choices)]
        offset = 0
        numeric_buffer = ""
        numeric_deadline = 0.0
        while True:
            _, height = self.terminal.size()
            page_size = max(1, height - 8)
            if selected < offset:
                offset = selected
            elif selected >= offset + page_size:
                offset = selected - page_size + 1
            offset = min(max(0, offset), max(0, len(choices) - page_size))
            self._overlay_lines = all_lines[offset : offset + page_size]
            self._overlay_selected = selected - offset
            self._overlay_title = f"{title}  [{selected + 1}/{len(choices)}]  (↑↓/PgUp/PgDn/滚轮，Enter确认)"
            self._render()
            event = self.terminal.read_event(0.2)
            if event.kind == "mouse" and event.key == "press" and event.y in self._overlay_rows:
                selected = offset + self._overlay_rows[event.y]
                if event.button == 0:
                    result = choices[selected]
                    break
            elif event.kind == "mouse" and event.key == "press":
                if event.button == 64:
                    selected = max(0, selected - 1)
                elif event.button == 65:
                    selected = min(len(choices) - 1, selected + 1)
            elif event.kind == "text" and event.text.isdigit():
                now = time.monotonic()
                if now > numeric_deadline:
                    numeric_buffer = ""
                numeric_buffer += event.text
                numeric_deadline = now + 0.8
                number = int(numeric_buffer) - 1
                if 0 <= number < len(choices):
                    selected = number
            elif event.kind == "key":
                numeric_buffer = ""
                if event.key == "up":
                    selected = (selected - 1) % len(choices)
                elif event.key == "down":
                    selected = (selected + 1) % len(choices)
                elif event.key == "pageup":
                    selected = max(0, selected - page_size)
                elif event.key == "pagedown":
                    selected = min(len(choices) - 1, selected + page_size)
                elif event.key == "enter":
                    result = choices[selected]
                    break
                elif event.key in {"escape", "ctrl_c"}:
                    result = None
                    break
        self._overlay_title = None
        self._overlay_lines = []
        self._overlay_selected = 0
        self._overlay_wide = False
        return result

    def confirm(self, title: str) -> bool:
        return self.prompt_choice(title, ("否", "是"), "否") == "是"

    def show_text(self, title: str, value: str) -> None:
        assert self.terminal is not None
        self._overlay_wide = False
        width, height = self.terminal.size()
        wrap_width = max(30, min(110, width - 12))
        source_lines: list[str] = []
        for raw_line in str(value).splitlines() or [""]:
            source_lines.extend(textwrap.wrap(raw_line, wrap_width) or [""])
        offset = 0
        page_size = max(4, height - 10)
        self._overlay_title = title + "  (↑/↓滚动，Esc/Enter关闭)"
        while True:
            self._overlay_lines = source_lines[offset : offset + page_size]
            self._overlay_selected = -1
            self._render()
            event = self.terminal.read_event(0.2)
            if event.kind == "key":
                if event.key in {"escape", "enter", "ctrl_c"}:
                    break
                if event.key == "up":
                    offset = max(0, offset - 1)
                elif event.key == "down":
                    offset = min(max(0, len(source_lines) - page_size), offset + 1)
                elif event.key == "pageup":
                    offset = max(0, offset - page_size)
                elif event.key == "pagedown":
                    offset = min(max(0, len(source_lines) - page_size), offset + page_size)
            elif event.kind == "mouse" and event.key == "press":
                if event.button == 64:
                    offset = max(0, offset - 1)
                elif event.button == 65:
                    offset = min(max(0, len(source_lines) - page_size), offset + 1)
        self._overlay_title = None
        self._overlay_lines = []
        self._overlay_selected = 0


def make_console_progress_reporter(
    output: Any = sys.stderr,
) -> Callable[[DiscoveryProgress], None]:
    last_length = 0

    def reporter(progress: DiscoveryProgress) -> None:
        nonlocal last_length
        width = 28
        ratio = 1.0 if progress.total == 0 else progress.completed / progress.total
        filled = min(width, max(0, int(round(width * ratio))))
        bar = "#" * filled + "-" * (width - filled)
        current = (
            f"{progress.current_host}:{progress.current_port}"
            if progress.current_host
            else "准备扫描"
        )
        message = (
            f"\r扫描 [{bar}] {progress.completed}/{progress.total} "
            f"{progress.percent:5.1f}%  当前 {current}  已发现 {len(progress.candidates)}"
        )
        output.write(message + " " * max(0, last_length - len(message)))
        output.flush()
        last_length = len(message)
        if progress.completed >= progress.total:
            output.write("\n")
            output.flush()

    return reporter


def choose_console_scope(
    candidates: Sequence[DiscoveredScope],
    *,
    input_func: Callable[[str], str] = input,
    output: Any = sys.stdout,
    require_tty: bool = True,
) -> DiscoveredScope | None:
    if not candidates:
        return None
    output.write(f"扫描发现 {len(candidates)} 台 SCPI 设备，必须手动选择：\n")
    for index, candidate in enumerate(candidates, 1):
        output.write(f"  {index:02d}. {candidate.display_name()}\n")
    output.flush()
    if require_tty and not sys.stdin.isatty():
        return None
    while True:
        try:
            answer = input_func("请输入设备编号（q 取消）: ").strip()
        except (EOFError, KeyboardInterrupt):
            return None
        if answer.lower() in {"q", "quit", "exit", ""}:
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(candidates):
            return candidates[int(answer) - 1]
        output.write("编号无效，请重新输入。\n")
        output.flush()


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "__dict__"):
        return value.__dict__
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def _print_json(value: Any) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, default=_json_default))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Discover and control RIGOL DHO800/DHO900/MHO900 oscilloscopes over LAN SCPI."
    )
    parser.add_argument(
        "--host",
        default=None,
        help="Oscilloscope IP/hostname; otherwise use cache and discovery",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help="Direct/default SCPI socket port; discovery also checks common raw-SCPI ports",
    )
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Socket timeout seconds")
    parser.add_argument(
        "--subnet",
        action="append",
        default=[],
        help="IPv4 discovery network such as 192.0.2.0/24; repeatable",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=DEFAULT_SCAN_TIMEOUT,
        help="Per-endpoint discovery timeout seconds",
    )
    parser.add_argument(
        "--no-discovery",
        action="store_true",
        help="Do not scan when direct/cache endpoints are unavailable",
    )
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("tui", help="Open the terminal control interface")
    subparsers.add_parser("catalog", help="List all TUI sections and actions")
    discover = subparsers.add_parser("discover", help="Scan and manually select a SCPI device")
    discover.add_argument("--json", action="store_true", dest="json_output")
    subparsers.add_parser("idn", help="Query *IDN?")
    status = subparsers.add_parser("status", help="Read oscilloscope status")
    status.add_argument("--channel", choices=("all", "1", "2", "3", "4"), default="all")
    for command in ("run", "stop", "single", "force", "autoset"):
        subparsers.add_parser(command, help=f"Execute {command.upper()}")

    channel = subparsers.add_parser("channel", help="Query or configure an analog channel")
    channel.add_argument("--channel", type=int, choices=(1, 2, 3, 4), required=True)
    channel.add_argument("--display", choices=("ON", "OFF"))
    channel.add_argument("--scale", type=parse_engineering_number)
    channel.add_argument("--offset", type=parse_engineering_number)
    channel.add_argument("--coupling", choices=("DC", "AC", "GND"))
    channel.add_argument("--probe", type=parse_engineering_number)
    channel.add_argument("--bandwidth", choices=("OFF", "20M"))
    channel.add_argument("--invert", choices=("ON", "OFF"))
    channel.add_argument("--units", choices=("VOLT", "WATT", "AMP", "UNKN"))
    channel.add_argument("--vernier", choices=("ON", "OFF"))
    channel.add_argument("--position", type=parse_engineering_number)
    channel.add_argument("--label")
    channel.add_argument("--label-show", choices=("ON", "OFF"))

    acquisition = subparsers.add_parser("acquisition", help="Query or configure acquisition")
    acquisition.add_argument("--type", dest="acquisition_type", choices=("NORM", "AVER", "PEAK", "ULTRA", "HRES"))
    acquisition.add_argument("--memory-depth")
    acquisition.add_argument("--averages", type=int)
    acquisition.add_argument("--bits", dest="resolution_bits", type=int, choices=(14, 16))
    acquisition.add_argument("--ultra-mode", choices=("ADJ", "OVER", "WATE", "PERS", "MOSA"))
    acquisition.add_argument("--ultra-timeout", type=parse_engineering_number)
    acquisition.add_argument("--ultra-max-frames", type=int)

    timebase = subparsers.add_parser("timebase", help="Query or configure the horizontal system")
    timebase.add_argument("--scale", type=parse_engineering_number)
    timebase.add_argument("--offset", type=parse_engineering_number)
    timebase.add_argument("--mode", choices=("MAIN", "XY", "ROLL"))
    timebase.add_argument("--reference-mode", choices=("CENT", "LB", "RB", "TRIG", "USER"))
    timebase.add_argument("--reference-position", type=parse_engineering_number)
    timebase.add_argument("--vernier", choices=("ON", "OFF"))
    timebase.add_argument("--roll", choices=("ON", "OFF"))
    timebase.add_argument("--delayed", choices=("ON", "OFF"))
    timebase.add_argument("--delay-scale", type=parse_engineering_number)
    timebase.add_argument("--delay-offset", type=parse_engineering_number)

    trigger = subparsers.add_parser("trigger", help="Query or configure trigger settings")
    trigger.add_argument(
        "--mode",
        choices=("EDGE", "PULSE", "SLOPE", "VIDEO", "PATTERN", "DURATION", "TIMEOUT", "RUNT", "WINDOW", "DELAY", "SETUP", "NEDGE", "RS232", "IIC", "SPI", "CAN", "LIN"),
    )
    trigger.add_argument("--source", choices=ANALOG_SOURCES + DIGITAL_SOURCES + ("EXT",))
    trigger.add_argument("--level", type=parse_engineering_number)
    trigger.add_argument("--slope", choices=("POS", "NEG", "RFALL"))
    trigger.add_argument("--sweep", choices=("AUTO", "NORM", "SINGLE"))
    trigger.add_argument("--coupling", choices=("AC", "DC", "LFREJECT", "HFREJECT"))
    trigger.add_argument("--holdoff", type=parse_engineering_number)
    trigger.add_argument("--noise-reject", choices=("ON", "OFF"))

    measure = subparsers.add_parser("measure", help="Query a measurement item")
    measure.add_argument("item", choices=MEASUREMENT_ITEMS)
    measure.add_argument("--source", default="CHAN1")
    measure.add_argument("--source-b")

    counter = subparsers.add_parser("counter", help="Query or configure the hardware counter")
    counter.add_argument("--enable", choices=("ON", "OFF"))
    counter.add_argument("--source", choices=ANALOG_SOURCES)
    counter.add_argument("--mode", choices=("FREQUENCY", "PERIOD", "TOTALIZE"))
    counter.add_argument("--digits", type=int)

    dvm = subparsers.add_parser("dvm", help="Query or configure the digital voltmeter")
    dvm.add_argument("--enable", choices=("ON", "OFF"))
    dvm.add_argument("--source", choices=ANALOG_SOURCES)
    dvm.add_argument("--mode", choices=("ACRMS", "DC", "DCRMS"))

    afg = subparsers.add_parser("afg", help="Query or configure the MHO900 built-in AFG")
    afg.add_argument("--channel", type=int, choices=(1, 2), default=1)
    afg.add_argument("--output", choices=("ON", "OFF"))
    afg.add_argument("--function", choices=("SIN", "SQU", "RAMP", "NOIS", "DC", "ARB", "EXPR", "EXPF", "ECG1", "GAUS", "LOR", "HAV", "SINC"))
    afg.add_argument("--arbitrary-path")
    afg.add_argument("--frequency", type=parse_engineering_number)
    afg.add_argument("--period", type=parse_engineering_number)
    afg.add_argument("--phase", type=parse_engineering_number)
    afg.add_argument("--symmetry", type=parse_engineering_number)
    afg.add_argument("--duty", type=parse_engineering_number)
    afg.add_argument("--amplitude", type=parse_engineering_number)
    afg.add_argument("--offset", type=parse_engineering_number)
    afg.add_argument("--high", type=parse_engineering_number)
    afg.add_argument("--low", type=parse_engineering_number)
    afg.add_argument("--impedance", choices=("OMEG", "FIFTY"))
    afg.add_argument("--modulation", choices=("ON", "OFF"))
    afg.add_argument("--modulation-type", choices=("AM", "FM", "PM"))
    afg.add_argument("--am-depth", type=parse_engineering_number)
    afg.add_argument("--fm-deviation", type=parse_engineering_number)
    afg.add_argument("--pm-deviation", type=parse_engineering_number)
    afg.add_argument("--modulation-frequency", type=parse_engineering_number)
    afg.add_argument("--modulation-function", choices=("SIN", "SQU", "TRI", "UPR", "DNR", "NOIS"))
    afg.add_argument("--sync-phase", action="store_true")

    waveform = subparsers.add_parser("waveform", help="Read waveform data")
    waveform.add_argument("--source", default="CHAN1")
    waveform.add_argument("--mode", choices=("NORM", "MAX", "RAW"), default="NORM")
    waveform.add_argument("--format", choices=("WORD", "BYTE", "ASCII"), default="WORD", dest="data_format")
    waveform.add_argument("--points", type=int, default=1000)
    waveform.add_argument("--start", type=int)
    waveform.add_argument("--stop", type=int)
    waveform.add_argument("--csv", type=Path)

    screenshot = subparsers.add_parser("screenshot", help="Save a screenshot")
    screenshot.add_argument("path", type=Path)
    screenshot.add_argument("--format", choices=("PNG", "BMP", "JPG"))

    raw = subparsers.add_parser("raw", help="Send a raw SCPI command")
    raw.add_argument("command")
    query = subparsers.add_parser("query", help="Query an arbitrary SCPI header")
    query.add_argument("header")
    query.add_argument("--arguments", default="")
    write = subparsers.add_parser("write", help="Set an arbitrary SCPI parameter")
    write.add_argument("header")
    write.add_argument("value")
    subparsers.add_parser("error", help="Read :SYSTem:ERRor?")
    return parser


def _args_have_values(args: argparse.Namespace, names: Sequence[str]) -> bool:
    return any(getattr(args, name, None) is not None for name in names)


def _discover_for_cli(args: argparse.Namespace) -> tuple[list[DiscoveredScope], tuple[str, ...]]:
    preferred = (args.host, DEFAULT_HOST)
    networks = discovery_networks(args.subnet, preferred)
    candidates = discover_scopes(
        tuple(map(str, networks)),
        ports=tuple(sorted({args.port, *DEFAULT_DISCOVERY_PORTS})),
        timeout=args.scan_timeout,
        preferred_hosts=preferred,
        progress_callback=make_console_progress_reporter(),
    )
    return candidates, tuple(map(str, networks))


def _resolve_cli_scope(args: argparse.Namespace) -> DiscoveredScope | None:
    scan_started = False

    def progress(value: DiscoveryProgress) -> None:
        nonlocal scan_started
        scan_started = True
        reporter(value)

    reporter = make_console_progress_reporter()
    location = locate_scope(
        args.host,
        args.port,
        args.timeout,
        subnets=args.subnet,
        scan_timeout=args.scan_timeout,
        cache_path=DEFAULT_CACHE_PATH,
        allow_scan=not args.no_discovery,
        progress_callback=progress,
    )
    candidate = location.selected
    if candidate is None and location.candidates:
        candidate = choose_console_scope(location.candidates)
    if candidate:
        save_last_scope(candidate)
    return candidate


def print_catalog() -> None:
    app = DHO924TerminalApp(DEFAULT_HOST, DEFAULT_PORT, DEFAULT_TIMEOUT, cache_path=None, auto_discover=False)
    total = 0
    for index, tab in enumerate(app.TABS):
        app.tab_index = index
        actions = app._actions()
        print(f"[{tab}]")
        for action in actions:
            print(f"  - {action.label}")
            total += 1
    print(f"\n总计: {len(app.TABS)} 个页签，{total} 个可执行项目")


def run_command(args: argparse.Namespace) -> int:
    if args.action == "tui":
        return DHO924TerminalApp(
            args.host,
            args.port,
            args.timeout,
            discovery_subnets=args.subnet,
            scan_timeout=args.scan_timeout,
            auto_discover=not args.no_discovery,
        ).run()
    if args.action == "catalog":
        print_catalog()
        return 0
    if args.action == "discover":
        candidates, networks = _discover_for_cli(args)
        if args.json_output:
            _print_json(
                {
                    "networks": networks,
                    "devices": [candidate.__dict__ for candidate in candidates],
                }
            )
        selected = choose_console_scope(candidates)
        if selected is None:
            print(
                "扫描结果必须在交互终端中手动选择；未保存任何设备。",
                file=sys.stderr,
            )
            return 2
        save_last_scope(selected)
        print(f"已选择: {selected.display_name()}")
        return 0

    candidate = _resolve_cli_scope(args)
    if candidate is None:
        print("未找到或未选择 SCPI 设备。", file=sys.stderr)
        return 2
    try:
        with RigolDHO(candidate.host, candidate.port, args.timeout) as scope:
            identity = scope.identify()
            compatible = is_supported_scope_identity(identity)
            generic_actions = {"idn", "raw", "query", "write", "error"}
            if not compatible and args.action not in generic_actions:
                raise ScopeError(
                    f"Selected device is not a RIGOL DHO800/DHO900/MHO900: {identity}"
                )
            if args.action == "idn":
                print(identity)
            elif args.action == "status":
                if args.channel == "all":
                    _print_json(scope.status())
                else:
                    _print_json(scope.channel_state(int(args.channel)))
            elif args.action == "run":
                scope.run()
                print("RUN")
            elif args.action == "stop":
                scope.stop()
                print("STOP")
            elif args.action == "single":
                scope.single()
                print("SINGLE")
            elif args.action == "force":
                scope.force_trigger()
                print("TFORCE")
            elif args.action == "autoset":
                scope.autoset()
                print("AUTOSET")
            elif args.action == "channel":
                names = (
                    "display", "scale", "offset", "coupling", "probe",
                    "bandwidth", "invert", "units", "vernier", "position",
                    "label", "label_show",
                )
                if _args_have_values(args, names):
                    result = scope.configure_channel(
                        args.channel,
                        **{name: getattr(args, name) for name in names},
                    )
                else:
                    result = scope.channel_state(args.channel)
                _print_json(result)
            elif args.action == "acquisition":
                names = (
                    "acquisition_type", "memory_depth", "averages", "resolution_bits", "ultra_mode",
                    "ultra_timeout", "ultra_max_frames",
                )
                result = (
                    scope.configure_acquisition(**{name: getattr(args, name) for name in names})
                    if _args_have_values(args, names)
                    else scope.acquisition_state()
                )
                _print_json(result)
            elif args.action == "timebase":
                names = (
                    "scale", "offset", "mode", "reference_mode", "reference_position",
                    "vernier", "roll", "delayed", "delay_scale", "delay_offset",
                )
                result = (
                    scope.configure_timebase(**{name: getattr(args, name) for name in names})
                    if _args_have_values(args, names)
                    else scope.timebase_state()
                )
                _print_json(result)
            elif args.action == "trigger":
                names = (
                    "mode", "source", "level", "slope", "sweep", "coupling",
                    "holdoff", "noise_reject",
                )
                result = (
                    scope.configure_trigger(**{name: getattr(args, name) for name in names})
                    if _args_have_values(args, names)
                    else scope.trigger_state()
                )
                _print_json(result)
            elif args.action == "measure":
                value = scope.measure(args.item, args.source, args.source_b)
                _print_json(
                    {
                        "item": args.item,
                        "source": args.source,
                        "value": None if invalid_measurement(value) else value,
                        "valid": not invalid_measurement(value),
                        "raw": value,
                    }
                )
            elif args.action == "counter":
                names = ("enable", "source", "mode", "digits")
                result = (
                    scope.configure_counter(
                        enabled=args.enable,
                        source=args.source,
                        mode=args.mode,
                        digits=args.digits,
                    )
                    if _args_have_values(args, names)
                    else scope.counter_state()
                )
                _print_json(result)
            elif args.action == "dvm":
                names = ("enable", "source", "mode")
                result = (
                    scope.configure_dvm(
                        enabled=args.enable, source=args.source, mode=args.mode
                    )
                    if _args_have_values(args, names)
                    else scope.dvm_state()
                )
                _print_json(result)
            elif args.action == "afg":
                names = (
                    "output", "function", "arbitrary_path", "frequency", "period",
                    "phase", "symmetry", "duty", "amplitude", "offset", "high",
                    "low", "impedance", "modulation", "modulation_type",
                    "am_depth", "fm_deviation", "pm_deviation",
                    "modulation_frequency", "modulation_function",
                )
                if _args_have_values(args, names):
                    result = scope.configure_afg(
                        args.channel,
                        **{name: getattr(args, name) for name in names},
                    )
                else:
                    result = scope.afg_state(args.channel)
                if args.sync_phase:
                    scope.synchronize_afg_phase(args.channel)
                    result = scope.afg_state(args.channel)
                _print_json(result)
            elif args.action == "waveform":
                waveform = scope.read_waveform(
                    source=args.source,
                    mode=args.mode,
                    data_format=args.data_format,
                    points=args.points,
                    start=args.start,
                    stop=args.stop,
                )
                if args.csv:
                    saved = waveform.save_csv(args.csv)
                    print(saved.resolve())
                _print_json(waveform.summary())
            elif args.action == "screenshot":
                image_format = args.format or {
                    ".bmp": "BMP", ".jpg": "JPG", ".jpeg": "JPG"
                }.get(args.path.suffix.lower(), "PNG")
                payload = scope.screenshot(image_format)
                args.path.parent.mkdir(parents=True, exist_ok=True)
                args.path.write_bytes(payload)
                print(args.path.resolve())
            elif args.action == "raw":
                if "?" in args.command:
                    print(scope.query(args.command))
                else:
                    scope.write(args.command)
                    print("OK")
            elif args.action == "query":
                print(scope.query_parameter(args.header, args.arguments))
            elif args.action == "write":
                scope.set_parameter(args.header, args.value)
                print("OK")
            elif args.action == "error":
                print(scope.system_error())
            else:
                raise ScopeError(f"Unsupported action: {args.action}")
        return 0
    except (OSError, ValueError, ScopeError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 1


def main(argv: Sequence[str] | None = None) -> int:
    with contextlib.suppress(Exception):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    with contextlib.suppress(Exception):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments:
        arguments = ["tui"]
    parser = build_parser()
    args = parser.parse_args(arguments)
    return run_command(args)


if __name__ == "__main__":
    raise SystemExit(main())
