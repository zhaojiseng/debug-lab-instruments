#!/usr/bin/env python3
"""Discover SCPI instruments and control SIGLENT SDG generators over raw TCP.

This script uses only the Python standard library.  Without --host it first
tries the last successfully identified device, then scans local IPv4 networks.
Discovery accepts any instrument that answers *IDN?; SDG-specific menus are
enabled for SIGLENT SDG models.

Known model profiles currently include SDG1032X/SDG1000X and SDG2122X/SDG2000X.

Examples:
    python sdg2122x_control.py discover
    python sdg2122x_control.py idn
    python sdg2122x_control.py status
    python sdg2122x_control.py sine --channel 1 --frequency 10k --amplitude 1 --enable
    python sdg2122x_control.py output --channel 1 off
    python sdg2122x_control.py raw "C1:BSWV?"
"""

from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import csv
import ipaddress
import json
import math
import os
import re
import shutil
import socket
import sys
import threading
import time
import textwrap
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence


DEFAULT_HOST = "192.0.2.213"
DEFAULT_PORT = 5025
DEFAULT_TIMEOUT = 3.0
DEFAULT_SCAN_TIMEOUT = 0.25
DEFAULT_SCAN_WORKERS = 64
MAX_DISCOVERY_HOSTS = 4096
DEFAULT_CACHE_PATH = Path(__file__).resolve().with_name(".sdg_device_cache.json")


class InstrumentError(RuntimeError):
    """Raised when communication with the instrument fails."""


class InstrumentCommandError(InstrumentError):
    """Raised when an instrument command is rejected or cannot be verified."""


@dataclass(frozen=True)
class DiscoveredInstrument:
    """A SCPI instrument that answered *IDN? on a TCP endpoint."""

    host: str
    port: int
    identity: str

    @property
    def identity_fields(self) -> tuple[str, str, str, str]:
        parts = [part.strip() for part in self.identity.split(",", 3)]
        parts.extend([""] * (4 - len(parts)))
        return parts[0], parts[1], parts[2], parts[3]

    @property
    def manufacturer(self) -> str:
        return self.identity_fields[0]

    @property
    def model(self) -> str:
        return self.identity_fields[1]

    @property
    def serial(self) -> str:
        return self.identity_fields[2]

    def display_name(self) -> str:
        model = self.model or "未知型号"
        manufacturer = self.manufacturer or "未知厂商"
        serial = f"SN:{self.serial}" if self.serial else "无序列号"
        kind = "SDG" if is_siglent_sdg_identity(self.identity) else "SCPI"
        return (
            f"{model}  |  {self.host}:{self.port}  |  {serial}  |  "
            f"{manufacturer}  |  {kind}"
        )


@dataclass(frozen=True)
class InstrumentLocation:
    """Result of resolving an explicit, cached, or discovered instrument."""

    selected: DiscoveredInstrument | None
    candidates: tuple[DiscoveredInstrument, ...]
    source: str
    networks: tuple[str, ...] = ()


@dataclass(frozen=True)
class DiscoveryProgress:
    """One progress update emitted while scanning SCPI endpoints."""

    completed: int
    total: int
    current_host: str | None
    current_port: int | None
    candidates: tuple[DiscoveredInstrument, ...]

    @property
    def percent(self) -> float:
        return 100.0 if self.total == 0 else self.completed * 100.0 / self.total


def is_siglent_sdg_identity(identity: str) -> bool:
    """Return True for any SIGLENT SDG model, not only the SDG2122X."""
    parts = [part.strip().upper() for part in identity.split(",")]
    if len(parts) < 2:
        return False
    return "SIGLENT" in parts[0] and parts[1].startswith("SDG")


SDG_PROFILED_FEATURES = frozenset(
    {
        "arbitrary_marker",
        "cascade",
        "combine",
        "coupling",
        "frequency_counter",
        "front_panel_keys",
        "harmonic",
        "manual_trigger_coupling",
        "noise_add",
        "phase_mode",
        "power_on_mode",
        "sample_rate",
    }
)


@dataclass(frozen=True)
class SDGModelProfile:
    """Model-specific limits and command-family availability for a SIGLENT SDG."""

    model: str
    family: str
    frequency_limits_hz: tuple[tuple[str, float], ...] = ()
    max_arbitrary_points: int | None = None
    arbitrary_index_range: tuple[int, int] | None = None
    supported_features: frozenset[str] = SDG_PROFILED_FEATURES
    languages: tuple[str, ...] = ("EN", "CH", "RU")
    documented: bool = False

    def frequency_limit(self, waveform: str) -> float | None:
        aliases = {
            "ARBITRARY": "ARB",
            "PULS": "PULSE",
            "SQU": "SQUARE",
            "TRIANGLE": "RAMP",
        }
        name = aliases.get(waveform.strip().upper(), waveform.strip().upper())
        for candidate, limit in self.frequency_limits_hz:
            if candidate == name:
                return limit
        return None

    def supports(self, feature: str) -> bool:
        return feature.strip().lower() in self.supported_features

    def summary(self) -> str:
        sine_limit = self.frequency_limit("SINE")
        limit = f"，正弦≤{format_frequency_limit(sine_limit)}" if sine_limit else ""
        source = "官方型号档案" if self.documented else "通用兼容档案"
        return f"{self.model} / {self.family}{limit} / {source}"


def format_frequency_limit(value: float | None) -> str:
    if value is None:
        return "未知"
    for scale, suffix in ((1e9, "GHz"), (1e6, "MHz"), (1e3, "kHz")):
        if value >= scale:
            number = value / scale
            return f"{number:g} {suffix}"
    return f"{value:g} Hz"


_SDG1000X_FEATURES = SDG_PROFILED_FEATURES - {
    "arbitrary_marker",
    "cascade",
    "front_panel_keys",
    "manual_trigger_coupling",
    "noise_add",
    "power_on_mode",
    "sample_rate",
}
_SDG2000X_FEATURES = SDG_PROFILED_FEATURES - {
    "arbitrary_marker",
    "front_panel_keys",
    "noise_add",
    "power_on_mode",
}


def _sdg1000x_profile(model: str, sine_square_limit: float) -> SDGModelProfile:
    return SDGModelProfile(
        model=model,
        family="SDG1000X",
        frequency_limits_hz=(
            ("SINE", sine_square_limit),
            ("SQUARE", sine_square_limit),
            ("PULSE", 12.5e6),
            ("RAMP", 500e3),
            ("ARB", 6e6),
        ),
        max_arbitrary_points=16_384,
        arbitrary_index_range=(2, 198),
        supported_features=_SDG1000X_FEATURES,
        languages=("EN", "CH"),
        documented=True,
    )


SDG_MODEL_PROFILES: dict[str, SDGModelProfile] = {
    "SDG1022X": _sdg1000x_profile("SDG1022X", 25e6),
    "SDG1032X": _sdg1000x_profile("SDG1032X", 30e6),
    "SDG1062X": _sdg1000x_profile("SDG1062X", 60e6),
    "SDG2122X": SDGModelProfile(
        model="SDG2122X",
        family="SDG2000X",
        frequency_limits_hz=(("SINE", 120e6),),
        arbitrary_index_range=(2, 198),
        supported_features=_SDG2000X_FEATURES,
        languages=("EN", "CH"),
        documented=True,
    ),
}


def get_sdg_model_profile(identity_or_model: str) -> SDGModelProfile | None:
    """Return a known or family-level profile from an IDN response or model name."""
    value = identity_or_model.strip()
    if not value:
        return None
    if "," in value:
        if not is_siglent_sdg_identity(value):
            return None
        parts = [part.strip() for part in value.split(",")]
        model = parts[1].upper()
    else:
        model = value.upper()
        if not model.startswith("SDG"):
            return None

    known = SDG_MODEL_PROFILES.get(model)
    if known is not None:
        return known
    if re.fullmatch(r"SDG10\d{2}X(?:-E)?", model):
        return SDGModelProfile(
            model=model,
            family="SDG1000X",
            max_arbitrary_points=16_384,
            arbitrary_index_range=(2, 198),
            supported_features=_SDG1000X_FEATURES,
            languages=("EN", "CH"),
        )
    if re.fullmatch(r"SDG2\d{3}X(?:-E)?", model):
        return SDGModelProfile(
            model=model,
            family="SDG2000X",
            arbitrary_index_range=(2, 198),
            supported_features=_SDG2000X_FEATURES,
            languages=("EN", "CH"),
        )
    return SDGModelProfile(model=model, family="SIGLENT SDG")


def instrument_identity_key(identity: str) -> tuple[str, ...]:
    """Build a stable match key, preferring manufacturer/model/serial."""
    parts = [part.strip().casefold() for part in identity.split(",")]
    if len(parts) >= 3 and parts[0] and parts[1] and parts[2]:
        return ("idn", parts[0], parts[1], parts[2])
    return ("raw", identity.strip().casefold())


def probe_scpi_instrument(
    host: str,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_SCAN_TIMEOUT,
) -> DiscoveredInstrument | None:
    """Read *IDN? from one endpoint without changing instrument state."""
    if not host or not 1 <= int(port) <= 65535 or timeout <= 0:
        return None
    try:
        with socket.create_connection((host, int(port)), timeout=timeout) as connection:
            connection.settimeout(timeout)
            connection.sendall(b"*IDN?\n")
            received = bytearray()
            while len(received) < 4096 and b"\n" not in received:
                try:
                    chunk = connection.recv(1024)
                except socket.timeout:
                    break
                if not chunk:
                    break
                received.extend(chunk)
    except OSError:
        return None

    first_line = bytes(received).splitlines()[0] if received else b""
    identity = first_line.decode("ascii", errors="replace").strip()
    if not identity or identity in {">", ">>"}:
        return None
    return DiscoveredInstrument(str(host), int(port), identity)


def load_last_device(
    cache_path: Path | str | None = DEFAULT_CACHE_PATH,
) -> DiscoveredInstrument | None:
    """Load the last successfully identified instrument from disk."""
    if cache_path is None:
        return None
    try:
        payload = json.loads(Path(cache_path).read_text(encoding="utf-8"))
        host = str(payload["host"]).strip()
        port = int(payload["port"])
        identity = str(payload["identity"]).strip()
        if not host or not identity or not 1 <= port <= 65535:
            return None
        return DiscoveredInstrument(host, port, identity)
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


def save_last_device(
    instrument: DiscoveredInstrument,
    cache_path: Path | str | None = DEFAULT_CACHE_PATH,
) -> bool:
    """Atomically remember the last successfully identified instrument."""
    if cache_path is None:
        return False
    path = Path(cache_path)
    temporary = path.with_name(path.name + ".tmp")
    payload = {
        "version": 1,
        "host": instrument.host,
        "port": instrument.port,
        "identity": instrument.identity,
        "last_seen_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, path)
        return True
    except OSError:
        with contextlib.suppress(OSError):
            temporary.unlink()
        return False


def _host_ipv4_addresses(host: str | None) -> list[str]:
    if not host:
        return []
    try:
        address = ipaddress.ip_address(host)
        return [str(address)] if isinstance(address, ipaddress.IPv4Address) else []
    except ValueError:
        pass
    try:
        return sorted(
            {
                item[4][0]
                for item in socket.getaddrinfo(host, None, socket.AF_INET, socket.SOCK_STREAM)
            }
        )
    except OSError:
        return []


def local_ipv4_addresses() -> list[str]:
    """Return usable local IPv4 addresses using only the standard library."""
    addresses: set[str] = set()
    with contextlib.suppress(OSError):
        for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addresses.add(item[4][0])
    with contextlib.suppress(OSError):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.0.2.1", 9))
            addresses.add(probe.getsockname()[0])

    result: list[str] = []
    for value in addresses:
        with contextlib.suppress(ValueError):
            address = ipaddress.IPv4Address(value)
            if not (address.is_loopback or address.is_link_local or address.is_unspecified):
                result.append(str(address))
    return sorted(set(result), key=lambda value: int(ipaddress.IPv4Address(value)))


def discovery_networks(
    subnets: Sequence[str] = (),
    *,
    preferred_hosts: Sequence[str | None] = (),
) -> tuple[ipaddress.IPv4Network, ...]:
    """Build explicit networks or /24 networks around known/local addresses."""
    networks: list[ipaddress.IPv4Network] = []
    if subnets:
        for subnet in subnets:
            network = ipaddress.ip_network(str(subnet).strip(), strict=False)
            if not isinstance(network, ipaddress.IPv4Network):
                raise ValueError(f"Only IPv4 discovery networks are supported: {subnet}")
            networks.append(network)
    else:
        addresses: list[str] = []
        for host in preferred_hosts:
            addresses.extend(_host_ipv4_addresses(host))
        addresses.extend(_host_ipv4_addresses(DEFAULT_HOST))
        addresses.extend(local_ipv4_addresses())
        for address in addresses:
            networks.append(ipaddress.ip_network(f"{address}/24", strict=False))

    unique: list[ipaddress.IPv4Network] = []
    seen: set[tuple[int, int]] = set()
    for network in networks:
        key = (int(network.network_address), network.prefixlen)
        if key not in seen:
            seen.add(key)
            unique.append(network)
    return tuple(unique)


def discover_instruments(
    subnets: Sequence[str] = (),
    *,
    ports: Sequence[int] = (DEFAULT_PORT,),
    timeout: float = DEFAULT_SCAN_TIMEOUT,
    workers: int = DEFAULT_SCAN_WORKERS,
    preferred_hosts: Sequence[str | None] = (),
    progress_callback: Callable[[DiscoveryProgress], None] | None = None,
) -> list[DiscoveredInstrument]:
    """Scan IPv4 networks and return every endpoint that answers *IDN?."""
    if timeout <= 0:
        raise ValueError("Discovery timeout must be greater than zero")
    valid_ports = sorted({int(port) for port in ports})
    if not valid_ports or any(not 1 <= port <= 65535 for port in valid_ports):
        raise ValueError("Discovery ports must be between 1 and 65535")

    networks = discovery_networks(subnets, preferred_hosts=preferred_hosts)
    endpoints: list[tuple[str, int]] = []
    seen_endpoints: set[tuple[str, int]] = set()
    for network in networks:
        for address in network.hosts():
            for port in valid_ports:
                endpoint = (str(address), port)
                if endpoint not in seen_endpoints:
                    seen_endpoints.add(endpoint)
                    endpoints.append(endpoint)
                    if len(endpoints) > MAX_DISCOVERY_HOSTS * len(valid_ports):
                        raise ValueError(
                            f"Discovery range is too large; limit it to {MAX_DISCOVERY_HOSTS} IPv4 hosts"
                        )

    def notify(progress: DiscoveryProgress) -> None:
        if progress_callback is not None:
            with contextlib.suppress(Exception):
                progress_callback(progress)

    notify(DiscoveryProgress(0, len(endpoints), None, None, ()))
    if not endpoints:
        return []
    results: list[DiscoveredInstrument] = []
    completed = 0
    worker_count = max(1, min(int(workers), len(endpoints)))
    with concurrent.futures.ThreadPoolExecutor(max_workers=worker_count) as executor:
        futures = {
            executor.submit(probe_scpi_instrument, host, port, timeout): (host, port)
            for host, port in endpoints
        }
        for future in concurrent.futures.as_completed(futures):
            host, port = futures[future]
            candidate = future.result()
            if candidate is not None:
                results.append(candidate)
            completed += 1
            ordered_results = tuple(
                sorted(
                    results,
                    key=lambda item: (
                        int(ipaddress.IPv4Address(item.host)),
                        item.port,
                        item.identity,
                    ),
                )
            )
            notify(
                DiscoveryProgress(
                    completed,
                    len(endpoints),
                    host,
                    port,
                    ordered_results,
                )
            )

    return sorted(
        results,
        key=lambda item: (int(ipaddress.IPv4Address(item.host)), item.port, item.identity),
    )


def locate_instrument(
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
) -> InstrumentLocation:
    """Resolve explicit/last endpoints first, then identify devices by scanning."""
    last = load_last_device(cache_path)
    attempted: set[tuple[str, int]] = set()

    def try_endpoint(host: str | None, endpoint_port: int, source: str, probe_timeout: float):
        if not host:
            return None
        endpoint = (host, endpoint_port)
        if endpoint in attempted:
            return None
        attempted.add(endpoint)
        candidate = probe_scpi_instrument(host, endpoint_port, probe_timeout)
        if candidate is not None:
            return InstrumentLocation(candidate, (candidate,), source)
        return None

    if not force_scan:
        direct = try_endpoint(requested_host, port, "explicit", timeout)
        if direct is not None:
            return direct
        if requested_host is not None:
            return InstrumentLocation(None, (), "explicit-not-found")
        if last is not None:
            direct = try_endpoint(
                last.host,
                last.port,
                "last",
                min(timeout, max(0.5, scan_timeout)),
            )
            if direct is not None:
                return direct
        elif requested_host is None:
            direct = try_endpoint(
                DEFAULT_HOST,
                port,
                "default",
                min(timeout, max(0.5, scan_timeout)),
            )
            if direct is not None:
                return direct

    if not allow_scan:
        return InstrumentLocation(None, (), "not-found")

    preferred_hosts = (requested_host, last.host if last else None, DEFAULT_HOST)
    networks = discovery_networks(subnets, preferred_hosts=preferred_hosts)
    scan_ports = {int(port)}
    if last is not None:
        scan_ports.add(last.port)
    candidates = discover_instruments(
        [str(network) for network in networks],
        ports=tuple(scan_ports),
        timeout=scan_timeout,
        preferred_hosts=preferred_hosts,
        progress_callback=progress_callback,
    )
    if last is not None:
        identity_matches = {
            candidate
            for candidate in candidates
            if instrument_identity_key(candidate.identity)
            == instrument_identity_key(last.identity)
        }
        candidates = sorted(
            candidates,
            key=lambda candidate: (
                0 if candidate in identity_matches else 1,
                int(ipaddress.IPv4Address(candidate.host)),
                candidate.port,
            ),
        )
    return InstrumentLocation(
        None,
        tuple(candidates),
        "scan-results" if candidates else "not-found",
        tuple(map(str, networks)),
    )


def make_console_progress_reporter(
    stream=None,
) -> Callable[[DiscoveryProgress], None]:
    """Create a throttled in-place progress reporter for terminal scans."""
    output = stream or sys.stderr
    last_render = 0.0
    last_found = -1

    def report(progress: DiscoveryProgress) -> None:
        nonlocal last_render, last_found
        now = time.monotonic()
        finished = progress.completed >= progress.total
        found_changed = len(progress.candidates) != last_found
        if not finished and not found_changed and now - last_render < 0.05:
            return
        last_render = now
        last_found = len(progress.candidates)
        width = 28
        ratio = 1.0 if progress.total == 0 else progress.completed / progress.total
        filled = min(width, max(0, int(round(width * ratio))))
        bar = "#" * filled + "-" * (width - filled)
        current = (
            f"{progress.current_host}:{progress.current_port}"
            if progress.current_host and progress.current_port
            else "准备扫描"
        )
        line = (
            f"\r扫描 [{bar}] {progress.completed}/{progress.total} "
            f"({progress.percent:5.1f}%)  已发现 {len(progress.candidates)}  当前 {current}"
        )
        output.write(line)
        output.flush()
        if finished:
            output.write("\n")
            output.flush()

    return report


def choose_console_device(
    candidates: Sequence[DiscoveredInstrument],
    *,
    show_list: bool = True,
    input_func: Callable[[str], str] = input,
    output=None,
    require_tty: bool = True,
) -> DiscoveredInstrument | None:
    """Require a numbered terminal selection, including for a single result."""
    stream = output or sys.stdout
    if not candidates:
        return None
    if show_list:
        print("可选择的 SCPI 设备：", file=stream)
        for index, candidate in enumerate(candidates, 1):
            print(f"  {index:02d}. {candidate.display_name()}", file=stream)
    if require_tty and not sys.stdin.isatty():
        return None
    while True:
        try:
            prompt = f"请选择设备编号 1-{len(candidates)}（Q 取消，必须手动确认）: "
            if input_func is input:
                stream.write(prompt)
                stream.flush()
                answer = sys.stdin.readline()
                if answer == "":
                    return None
            else:
                answer = input_func(prompt)
            answer = answer.strip()
        except (EOFError, KeyboardInterrupt):
            return None
        if answer.casefold() in {"q", "quit", "cancel", "取消"}:
            return None
        if answer.isdigit():
            index = int(answer) - 1
            if 0 <= index < len(candidates):
                return candidates[index]
        print("输入无效，请输入列表中的设备编号。", file=stream)


def parse_engineering_number(value: str) -> float:
    """Parse values such as 10000, 10k, 1.2M, 500m, or 2.5u."""
    match = re.fullmatch(
        r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*([pnumkKMG]?)\s*",
        value,
    )
    if not match:
        raise argparse.ArgumentTypeError(f"Invalid numeric value: {value!r}")

    number = float(match.group(1))
    suffix = match.group(2)
    multipliers = {
        "": 1.0,
        "p": 1e-12,
        "n": 1e-9,
        "u": 1e-6,
        "m": 1e-3,
        "k": 1e3,
        "K": 1e3,
        "M": 1e6,
        "G": 1e9,
    }
    return number * multipliers[suffix]


def scpi_number(value: float) -> str:
    """Format a number without unnecessary exponent notation."""
    result = f"{value:.12f}".rstrip("0").rstrip(".")
    return result if result not in {"", "-0"} else "0"


def scpi_quote(value: str) -> str:
    """Quote a SCPI string and escape embedded double quotes."""
    return '"' + value.replace('"', '""') + '"'


def scpi_value(value: Any) -> str:
    """Convert a Python value to a SCPI parameter value."""
    if isinstance(value, bool):
        return "ON" if value else "OFF"
    if isinstance(value, float):
        return scpi_number(value)
    if isinstance(value, int):
        return str(value)
    return str(value).strip()


def scpi_pairs(parameters: Mapping[str, Any] | Sequence[tuple[str, Any]]) -> str:
    """Build PARAM,VALUE pairs while preserving caller order."""
    items = parameters.items() if isinstance(parameters, Mapping) else parameters
    parts: list[str] = []
    for name, value in items:
        name = str(name).strip().upper()
        if not name:
            raise ValueError("SCPI parameter name cannot be empty")
        parts.extend((name, scpi_value(value)))
    if not parts:
        raise ValueError("At least one SCPI parameter is required")
    return ",".join(parts)


def parse_scpi_pairs(response: str) -> dict[str, str]:
    """Parse the comma-separated parameter section of a SIGLENT response."""
    payload = response.strip()
    if " " in payload:
        payload = payload.split(" ", 1)[1]
    tokens = [token.strip() for token in payload.split(",") if token.strip()]
    result: dict[str, str] = {}
    index = 0
    while index + 1 < len(tokens):
        key = tokens[index].upper()
        value = tokens[index + 1]
        result[key] = value
        index += 2
    return result


def numeric_prefix(value: str) -> float:
    """Return the leading numeric portion of a SCPI response value."""
    match = re.match(r"\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)", value)
    if not match:
        raise ValueError(f"Response does not begin with a number: {value!r}")
    return float(match.group(1))


def validate_channel(channel: int) -> int:
    if channel not in (1, 2):
        raise ValueError("Channel must be 1 or 2")
    return channel


@dataclass(frozen=True)
class SineConfiguration:
    channel: int
    frequency_hz: float
    amplitude_vpp: float
    offset_v: float = 0.0
    phase_deg: float = 0.0

    def validate(
        self,
        max_frequency_hz: float = 120e6,
        model: str = "SIGLENT SDG",
    ) -> None:
        validate_channel(self.channel)
        if not 1e-6 <= self.frequency_hz <= max_frequency_hz:
            raise ValueError(
                f"{model} SINE frequency must be between 1 uHz and "
                f"{format_frequency_limit(max_frequency_hz)}"
            )
        if not 0.0 < self.amplitude_vpp <= 20.0:
            raise ValueError("Amplitude must be greater than 0 and at most 20 Vpp")
        if not -10.0 <= self.offset_v <= 10.0:
            raise ValueError("Offset must be between -10 V and +10 V")
        if not 0.0 <= self.phase_deg <= 360.0:
            raise ValueError("Phase must be between 0 and 360 degrees")


class SDG2122X:
    """LAN SCPI client for supported SIGLENT SDG generators."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._socket: socket.socket | None = None
        self._receive_buffer = bytearray()
        self._lock = threading.RLock()
        self.identity: str | None = None
        self.model_profile = SDGModelProfile(
            model="未知 SDG",
            family="SIGLENT SDG",
        )

    def connect(self) -> "SDG2122X":
        with self._lock:
            if self._socket is not None:
                return self
            try:
                self._socket = socket.create_connection(
                    (self.host, self.port), timeout=self.timeout
                )
                self._socket.settimeout(self.timeout)
            except OSError as exc:
                raise InstrumentError(
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

    @property
    def is_connected(self) -> bool:
        return self._socket is not None

    def reconnect(self) -> "SDG2122X":
        self.close()
        return self.connect()

    def __enter__(self) -> "SDG2122X":
        return self.connect()

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _connected_socket(self) -> socket.socket:
        if self._socket is None:
            raise InstrumentError("Instrument is not connected")
        return self._socket

    def write(self, command: str) -> None:
        command = command.strip()
        if not command:
            raise ValueError("SCPI command cannot be empty")
        with self._lock:
            try:
                self._connected_socket().sendall(command.encode("ascii") + b"\n")
            except (OSError, UnicodeEncodeError) as exc:
                raise InstrumentError(f"Failed to send {command!r}: {exc}") from exc

    def _read_line(self) -> str:
        connection = self._connected_socket()
        while b"\n" not in self._receive_buffer:
            try:
                chunk = connection.recv(4096)
            except socket.timeout as exc:
                raise InstrumentError("Timed out waiting for instrument response") from exc
            except OSError as exc:
                raise InstrumentError(f"Failed to read instrument response: {exc}") from exc
            if not chunk:
                raise InstrumentError("Instrument closed the connection")
            self._receive_buffer.extend(chunk)

        line, _, remaining = self._receive_buffer.partition(b"\n")
        self._receive_buffer = bytearray(remaining)
        return line.rstrip(b"\r").decode("ascii", errors="replace").strip()

    def query(self, command: str) -> str:
        with self._lock:
            self.write(command)
            return self._read_line()

    def identify(self) -> str:
        identity = self.query("*IDN?")
        self.identity = identity
        profile = get_sdg_model_profile(identity)
        if profile is not None:
            self.model_profile = profile
        return identity

    def _ensure_model_profile(self) -> SDGModelProfile:
        if self.identity is None:
            self.identify()
        assert self.identity is not None
        profile = get_sdg_model_profile(self.identity)
        if profile is None:
            raise InstrumentCommandError(
                f"{self.identity!r} is not a recognized SIGLENT SDG generator"
            )
        self.model_profile = profile
        return profile

    def _require_feature(self, feature: str, command_family: str) -> SDGModelProfile:
        profile = self._ensure_model_profile()
        if not profile.supports(feature):
            raise InstrumentCommandError(
                f"{profile.model} ({profile.family}) does not support "
                f"{command_family} through its documented SCPI command set"
            )
        return profile

    @staticmethod
    def _frequency_value(value: Any) -> float | None:
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return float(value)
        text = str(value).strip()
        text = re.sub(r"(?i)\s*HZ\s*$", "", text)
        try:
            return parse_engineering_number(text)
        except argparse.ArgumentTypeError:
            return None

    @staticmethod
    def _validate_waveform_frequency(
        profile: SDGModelProfile,
        waveform: str,
        frequency_hz: float,
    ) -> None:
        limit = profile.frequency_limit(waveform)
        if limit is None:
            return
        if not 1e-6 <= frequency_hz <= limit:
            name = waveform.strip().upper()
            raise ValueError(
                f"{profile.model} {name} frequency must be between 1 uHz and "
                f"{format_frequency_limit(limit)}"
            )

    @staticmethod
    def _response_parameter(response: str, parameter: str) -> str | None:
        payload = response.split(" ", 1)[1] if " " in response else response
        tokens = [token.strip() for token in payload.split(",")]
        target = parameter.strip().upper()
        for index, token in enumerate(tokens[:-1]):
            if token.upper() == target:
                return tokens[index + 1]
        return None

    def _current_waveform_context(
        self, channel: int, header: str = "BSWV"
    ) -> tuple[str, float | None]:
        response = self.query_channel(channel, header)
        waveform = self._response_parameter(response, "WVTP")
        frequency = self._response_parameter(response, "FRQ")
        if not waveform or frequency is None:
            basic = self.basic_waveform(channel) if header != "BSWV" else response
            waveform = waveform or self._response_parameter(basic, "WVTP")
            frequency = frequency or self._response_parameter(basic, "FRQ")
        return waveform or "SINE", self._frequency_value(frequency) if frequency else None

    def _validate_basic_parameters(
        self,
        channel: int,
        items: Sequence[tuple[str, Any]],
    ) -> None:
        normalized = {str(name).strip().upper(): value for name, value in items}
        if not {"WVTP", "FRQ"}.intersection(normalized):
            return
        profile = self._ensure_model_profile()
        current_waveform, current_frequency = self._current_waveform_context(channel)
        waveform = str(normalized.get("WVTP", current_waveform))
        frequency = self._frequency_value(normalized.get("FRQ"))
        if frequency is None:
            frequency = current_frequency
        if frequency is not None:
            self._validate_waveform_frequency(profile, waveform, frequency)

    def _validate_carrier_tokens(
        self,
        channel: int,
        header: str,
        tokens: Sequence[Any],
    ) -> None:
        names = [str(token).strip() for token in tokens]
        upper = [token.upper() for token in names]
        if "CARR" not in upper or not {"WVTP", "FRQ"}.intersection(upper):
            return
        profile = self._ensure_model_profile()
        current_waveform, current_frequency = self._current_waveform_context(
            channel, header
        )

        def value_after(parameter: str) -> Any | None:
            try:
                index = upper.index(parameter)
            except ValueError:
                return None
            return tokens[index + 1] if index + 1 < len(tokens) else None

        waveform = str(value_after("WVTP") or current_waveform)
        frequency = self._frequency_value(value_after("FRQ"))
        if frequency is None:
            frequency = current_frequency
        if frequency is not None:
            self._validate_waveform_frequency(profile, waveform, frequency)

    def _validate_sweep_frequencies(
        self,
        channel: int,
        items: Sequence[tuple[str, Any]],
    ) -> None:
        frequency_parameters = {"START", "STOP", "CENTER"}
        requested = [
            (str(name).strip().upper(), value)
            for name, value in items
            if str(name).strip().upper() in frequency_parameters
        ]
        if not requested:
            return
        profile = self._ensure_model_profile()
        waveform, _ = self._current_waveform_context(channel, "SWWV")
        for name, value in requested:
            frequency = self._frequency_value(value)
            if frequency is not None:
                try:
                    self._validate_waveform_frequency(profile, waveform, frequency)
                except ValueError as exc:
                    raise ValueError(f"Sweep {name}: {exc}") from exc

    def operation_complete(self) -> bool:
        return self.query("*OPC?") == "1"

    def system_error(self) -> str:
        return self.query("SYST:ERR?")

    def basic_waveform(self, channel: int) -> str:
        validate_channel(channel)
        return self.query(f"C{channel}:BSWV?")

    def output_status(self, channel: int) -> str:
        validate_channel(channel)
        return self.query(f"C{channel}:OUTP?")

    def set_output(self, channel: int, enabled: bool) -> str:
        validate_channel(channel)
        state = "ON" if enabled else "OFF"
        self.write(f"C{channel}:OUTP {state}")
        return self.output_status(channel)

    def set_all_outputs(self, enabled: bool) -> tuple[str, str]:
        """Set both channel outputs with the shared SDG command and verify each channel."""
        self._ensure_model_profile()
        self.write(f"OUT_BOTHCH {'ON' if enabled else 'OFF'}")
        return self.output_status(1), self.output_status(2)

    def set_sine(self, configuration: SineConfiguration) -> str:
        profile = self._ensure_model_profile()
        maximum = profile.frequency_limit("SINE") or 120e6
        configuration.validate(maximum, profile.model)
        channel = configuration.channel
        command = (
            f"C{channel}:BSWV "
            f"WVTP,SINE,"
            f"FRQ,{scpi_number(configuration.frequency_hz)},"
            f"AMP,{scpi_number(configuration.amplitude_vpp)},"
            f"OFST,{scpi_number(configuration.offset_v)},"
            f"PHSE,{scpi_number(configuration.phase_deg)}"
        )
        self.write(command)
        return self.basic_waveform(channel)

    def channel_status(self, channel: int) -> tuple[str, str]:
        return self.basic_waveform(channel), self.output_status(channel)

    # ---- Generic SCPI subsystem helpers ---------------------------------

    def query_channel(self, channel: int, header: str) -> str:
        validate_channel(channel)
        return self.query(f"C{channel}:{header.strip().upper()}?")

    def set_channel_tokens(
        self,
        channel: int,
        header: str,
        tokens: Sequence[Any],
        *,
        verify: bool = True,
    ) -> str | None:
        validate_channel(channel)
        if not tokens:
            raise ValueError("At least one SCPI token is required")
        header = header.strip().upper()
        token_items = list(tokens)
        if header in {"MDWV", "SWWV", "BTWV"}:
            self._validate_carrier_tokens(channel, header, token_items)
        payload = ",".join(scpi_value(token) for token in token_items)
        self.write(f"C{channel}:{header} {payload}")
        return self.query_channel(channel, header) if verify else None

    def set_channel_parameters(
        self,
        channel: int,
        header: str,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]],
        *,
        verify: bool = True,
    ) -> str | None:
        validate_channel(channel)
        header = header.strip().upper()
        items = list(parameters.items()) if isinstance(parameters, Mapping) else list(parameters)
        if header == "BSWV":
            self._validate_basic_parameters(channel, items)
        elif header == "SWWV":
            self._validate_sweep_frequencies(channel, items)
        feature_headers = {
            "HARM": ("harmonic", "HARM"),
            "CMBN": ("combine", "CMBN"),
            "SRATE": ("sample_rate", "SRATE"),
        }
        if header in feature_headers:
            self._require_feature(*feature_headers[header])
        self.write(f"C{channel}:{header} {scpi_pairs(items)}")
        return self.query_channel(channel, header) if verify else None

    def query_global(self, header: str) -> str:
        return self.query(f"{header.strip()}?")

    def set_global_parameters(
        self,
        header: str,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]],
        *,
        verify: bool = True,
    ) -> str | None:
        header = header.strip()
        self.write(f"{header} {scpi_pairs(parameters)}")
        return self.query_global(header) if verify else None

    # ---- Output and basic waveforms -------------------------------------

    def configure_output(
        self,
        channel: int,
        *,
        enabled: bool | None = None,
        load: str | float | int | None = None,
        polarity: str | None = None,
    ) -> str:
        validate_channel(channel)
        parts: list[str] = []
        if enabled is not None:
            parts.append("ON" if enabled else "OFF")
        if load is not None:
            load_value = str(load).strip().upper()
            if load_value in {"HI-Z", "HIZ", "HIGHZ", "HIGH-Z"}:
                load_value = "HZ"
            parts.extend(("LOAD", load_value))
        if polarity is not None:
            polarity_value = polarity.strip().upper()
            if polarity_value not in {"NOR", "INVT"}:
                raise ValueError("Polarity must be NOR or INVT")
            parts.extend(("PLRT", polarity_value))
        if not parts:
            raise ValueError("At least one output setting is required")
        self.write(f"C{channel}:OUTP " + ",".join(parts))
        return self.output_status(channel)

    def set_basic_wave(
        self,
        channel: int,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]],
    ) -> str:
        response = self.set_channel_parameters(channel, "BSWV", parameters)
        assert response is not None
        return response

    def set_noise_add(
        self,
        channel: int,
        enabled: bool,
        *,
        ratio: float | None = None,
        ratio_db: float | None = None,
    ) -> str:
        validate_channel(channel)
        self._require_feature("noise_add", "NOISE_ADD")
        parameters: list[tuple[str, Any]] = [("STATE", enabled)]
        if ratio is not None and ratio_db is not None:
            raise ValueError("Specify ratio or ratio_db, not both")
        if ratio is not None:
            parameters.append(("RATIO", ratio))
        if ratio_db is not None:
            parameters.append(("RATIO_DB", ratio_db))
        self.write(f"C{channel}:NOISE_ADD {scpi_pairs(parameters)}")
        return self.query(f"C{channel}:NOISE_ADD?")

    # ---- Modulation, sweep and burst ------------------------------------

    def modulation_status(self, channel: int) -> str:
        return self.query_channel(channel, "MDWV")

    def set_modulation(
        self,
        channel: int,
        modulation_type: str | None = None,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]] | None = None,
    ) -> str:
        validate_channel(channel)
        parts: list[str] = []
        if modulation_type:
            modulation_type = modulation_type.strip().upper()
            allowed = {"AM", "DSBAM", "FM", "PM", "PWM", "ASK", "FSK", "PSK"}
            if modulation_type not in allowed:
                profile = self._ensure_model_profile()
                raise ValueError(
                    f"Unsupported {profile.model} modulation type: {modulation_type}"
                )
            parts.append(modulation_type)
        if parameters:
            parts.append(scpi_pairs(parameters))
        if not parts:
            raise ValueError("A modulation type or parameter is required")
        self.write(f"C{channel}:MDWV " + ",".join(parts))
        return self.modulation_status(channel)

    def sweep_status(self, channel: int) -> str:
        return self.query_channel(channel, "SWWV")

    def set_sweep(
        self,
        channel: int,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]],
    ) -> str:
        response = self.set_channel_parameters(channel, "SWWV", parameters)
        assert response is not None
        return response

    def trigger_sweep(self, channel: int) -> str:
        validate_channel(channel)
        self.write(f"C{channel}:SWWV MTRIG")
        return self.sweep_status(channel)

    def burst_status(self, channel: int) -> str:
        return self.query_channel(channel, "BTWV")

    def set_burst(
        self,
        channel: int,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]],
    ) -> str:
        response = self.set_channel_parameters(channel, "BTWV", parameters)
        assert response is not None
        return response

    def trigger_burst(self, channel: int) -> str:
        validate_channel(channel)
        self.write(f"C{channel}:BTWV MTRIG")
        return self.burst_status(channel)

    # ---- Arbitrary waveforms --------------------------------------------

    def copy_channel(self, source: int, target: int) -> None:
        validate_channel(source)
        validate_channel(target)
        if source == target:
            raise ValueError("Source and target channels must be different")
        self.write(f"PACP C{target},C{source}")

    def arbitrary_status(self, channel: int) -> str:
        return self.query_channel(channel, "ARWV")

    def select_arbitrary(
        self,
        channel: int,
        *,
        index: int | None = None,
        name: str | None = None,
        path: str | None = None,
    ) -> str:
        validate_channel(channel)
        profile = self._ensure_model_profile()
        supplied = sum(value is not None for value in (index, name, path))
        if supplied != 1:
            raise ValueError("Specify exactly one of index, name, or path")
        if index is not None:
            if profile.arbitrary_index_range is not None:
                minimum, maximum = profile.arbitrary_index_range
                if not minimum <= index <= maximum:
                    raise ValueError(
                        f"{profile.model} built-in arbitrary waveform index must be "
                        f"between {minimum} and {maximum}"
                    )
            command = f"C{channel}:ARWV INDEX,{index}"
        else:
            command = f"C{channel}:ARWV NAME,{scpi_quote(name or path or '')}"
        self.write(command)
        return self.arbitrary_status(channel)

    def set_arbitrary_marker(self, channel: int, enabled: bool) -> str:
        validate_channel(channel)
        self._require_feature("arbitrary_marker", "arbitrary waveform Marker (MSW)")
        self.write(f"C{channel}:MSW {'ON' if enabled else 'OFF'}")
        self.operation_complete()
        return "ON" if enabled else "OFF"

    def list_waveforms(self, scope: str | None = None, path: str | None = None) -> str:
        if path is not None:
            return self.query(f"STL? USER,{scpi_quote(path)}")
        if scope is None:
            return self.query("STL?")
        scope_value = scope.strip().upper()
        if scope_value not in {"BUILDIN", "USER"}:
            raise ValueError("Waveform scope must be BUILDIN or USER")
        return self.query(f"STL? {scope_value}")

    def upload_waveform_hex(self, channel: int, name: str, hex_data: str) -> None:
        validate_channel(channel)
        profile = self._ensure_model_profile()
        clean = re.sub(r"[^0-9A-Fa-f]", "", hex_data)
        if not clean or len(clean) % 4:
            raise ValueError("Waveform hex data must contain complete 16-bit samples")
        sample_count = len(clean) // 4
        if (
            profile.max_arbitrary_points is not None
            and sample_count > profile.max_arbitrary_points
        ):
            raise ValueError(
                f"{profile.model} arbitrary waveform memory accepts at most "
                f"{profile.max_arbitrary_points} samples; received {sample_count}"
            )
        self.write(
            f"C{channel}:WVDT WVNM,{scpi_quote(name)},WAVEDATA,b'0x{clean.lower()}'"
        )
        self.operation_complete()

    def upload_waveform_samples(
        self,
        channel: int,
        name: str,
        samples: Iterable[float | int],
        *,
        normalized: bool = True,
    ) -> None:
        encoded: list[str] = []
        for sample in samples:
            if normalized:
                value = float(sample)
                if not -1.0 <= value <= 1.0:
                    raise ValueError("Normalized waveform samples must be between -1 and 1")
                integer = int(round(value * 32767.0))
            else:
                integer = int(sample)
                if not -32768 <= integer <= 32767:
                    raise ValueError("Integer waveform samples must fit signed 16-bit range")
            encoded.append(f"{integer & 0xFFFF:04x}")
        if not encoded:
            raise ValueError("Waveform must contain at least one sample")
        self.upload_waveform_hex(channel, name, "".join(encoded))

    def upload_waveform_csv(
        self,
        channel: int,
        name: str,
        file_path: str | os.PathLike[str],
        *,
        normalized: bool = True,
    ) -> None:
        values: list[float] = []
        with Path(file_path).open("r", encoding="utf-8-sig", newline="") as stream:
            for row in csv.reader(stream):
                for cell in row:
                    cell = cell.strip()
                    if cell:
                        values.append(float(cell))
        self.upload_waveform_samples(channel, name, values, normalized=normalized)

    def waveform_data(self, name: str, path: str | None = None) -> str:
        if path is None:
            return self.query(f"WVDT? USER,{scpi_quote(name)}")
        return self.query(f"WVDT? USER,{scpi_quote(path)},{scpi_quote(name)}")

    # ---- Synchronization and channel relationships ----------------------

    def sync_status(self, channel: int) -> str:
        return self.query_channel(channel, "SYNC")

    def set_sync(self, channel: int, enabled: bool) -> str:
        validate_channel(channel)
        self.write(f"C{channel}:SYNC {'ON' if enabled else 'OFF'}")
        return self.sync_status(channel)

    def equal_phase(self) -> None:
        self.write("EQPHASE")

    def coupling_status(self) -> str:
        self._require_feature("coupling", "COUP")
        return self.query("COUP?")

    def set_coupling(
        self, parameters: Mapping[str, Any] | Sequence[tuple[str, Any]]
    ) -> str:
        self._require_feature("coupling", "COUP")
        self.write(f"COUP {scpi_pairs(parameters)}")
        return self.coupling_status()

    def phase_mode(self) -> str:
        self._require_feature("phase_mode", "MODE")
        return self.query("MODE?")

    def set_phase_mode(self, mode: str) -> str:
        self._require_feature("phase_mode", "MODE")
        mode_value = mode.strip().upper()
        if mode_value not in {"PHASELOCKED", "INDEPENDENT"}:
            raise ValueError("Phase mode must be PHASELOCKED or INDEPENDENT")
        self.write(f"MODE {mode_value}")
        return self.phase_mode()

    def cascade_status(self) -> str:
        self._require_feature("cascade", "CASCADE")
        return self.query("CASCADE?")

    def set_cascade(
        self,
        enabled: bool,
        *,
        mode: str = "MASTER",
        delay_s: float | None = None,
    ) -> str:
        self._require_feature("cascade", "CASCADE")
        mode_value = mode.strip().upper()
        if mode_value not in {"MASTER", "SLAVE"}:
            raise ValueError("Cascade mode must be MASTER or SLAVE")
        parameters: list[tuple[str, Any]] = [("STATE", enabled), ("MODE", mode_value)]
        if delay_s is not None:
            if mode_value != "SLAVE":
                raise ValueError("Cascade delay is only valid in SLAVE mode")
            parameters.append(("DELAY", delay_s))
        self.write(f"CASCADE {scpi_pairs(parameters)}")
        return self.cascade_status()

    # ---- Sampling, harmonics and waveform combination -------------------

    def sample_rate_status(self, channel: int) -> str:
        self._require_feature("sample_rate", "SRATE")
        return self.query_channel(channel, "SRATE")

    def set_sample_rate(
        self,
        channel: int,
        *,
        mode: str | None = None,
        value: float | None = None,
    ) -> str:
        profile = self._require_feature("sample_rate", "SRATE")
        parameters: list[tuple[str, Any]] = []
        if mode is not None:
            mode_value = mode.strip().upper()
            if mode_value not in {"DDS", "TARB"}:
                raise ValueError(f"{profile.model} sample-rate mode must be DDS or TARB")
            parameters.append(("MODE", mode_value))
        if value is not None:
            parameters.append(("VALUE", value))
        response = self.set_channel_parameters(channel, "SRATE", parameters)
        assert response is not None
        return response

    def harmonic_status(self, channel: int) -> str:
        self._require_feature("harmonic", "HARM")
        return self.query_channel(channel, "HARM")

    def set_harmonic(
        self,
        channel: int,
        parameters: Mapping[str, Any] | Sequence[tuple[str, Any]],
    ) -> str:
        self._require_feature("harmonic", "HARM")
        response = self.set_channel_parameters(channel, "HARM", parameters)
        assert response is not None
        return response

    def combine_status(self, channel: int) -> str:
        self._require_feature("combine", "CMBN")
        return self.query_channel(channel, "CMBN")

    def set_combine(self, channel: int, enabled: bool) -> str:
        self._require_feature("combine", "CMBN")
        validate_channel(channel)
        self.write(f"C{channel}:CMBN {'ON' if enabled else 'OFF'}")
        return self.combine_status(channel)

    # ---- Frequency counter and protection -------------------------------

    def frequency_counter_status(self) -> str:
        self._require_feature("frequency_counter", "FCNT")
        return self.query("FCNT?")

    def set_frequency_counter(
        self, parameters: Mapping[str, Any] | Sequence[tuple[str, Any]]
    ) -> str:
        self._require_feature("frequency_counter", "FCNT")
        self.write(f"FCNT {scpi_pairs(parameters)}")
        return self.frequency_counter_status()

    def invert_status(self, channel: int) -> str:
        return self.query_channel(channel, "INVT")

    def set_invert(self, channel: int, enabled: bool) -> str:
        validate_channel(channel)
        self.write(f"C{channel}:INVT {'ON' if enabled else 'OFF'}")
        return self.invert_status(channel)

    def overvoltage_protection(self) -> str:
        return self.query("VOLTPRT?")

    def set_overvoltage_protection(self, enabled: bool) -> str:
        self.write(f"VOLTPRT {'ON' if enabled else 'OFF'}")
        return self.overvoltage_protection()

    def overcurrent_protection(self) -> str:
        return self.query("CURRPRT?")

    def set_overcurrent_protection(self, enabled: bool) -> str:
        self.write(f"CURRPRT {'ON' if enabled else 'OFF'}")
        return self.overcurrent_protection()

    def overload_status(self) -> str:
        return self.query("VOLTSTAT?")

    # ---- System configuration -------------------------------------------

    def set_number_format(self, decimal: str, separator: str) -> str:
        decimal_value = decimal.strip().upper()
        separator_value = separator.strip().upper()
        if decimal_value not in {"DOT", "COMMA"}:
            raise ValueError("Decimal marker must be DOT or COMMA")
        if separator_value not in {"SPACE", "NONE", "COMMA", "DOT"}:
            raise ValueError("Unsupported digit separator")
        self.write(f"NBFM PNT,{decimal_value},SEPT,{separator_value}")
        return self.query("NBFM?")

    def language(self) -> str:
        return self.query("LAGG?")

    def set_language(self, language: str) -> str:
        value = language.strip().upper()
        profile = self._ensure_model_profile()
        if value not in profile.languages:
            raise ValueError(
                f"{profile.model} language must be one of {', '.join(profile.languages)}"
            )
        self.write(f"LAGG {value}")
        return self.language()

    def startup_config(self) -> str:
        return self.query("SCFG?")

    def set_startup_config(self, mode: str) -> str:
        value = mode.strip().upper()
        if value not in {"DEFAULT", "LAST", "USER"}:
            raise ValueError("Startup config must be DEFAULT, LAST, or USER")
        self.write(f"SCFG {value}")
        return self.startup_config()

    def set_date(self, value: str) -> None:
        if not re.fullmatch(r"\d{4}/\d{1,2}/\d{1,2}", value.strip()):
            raise ValueError("Date must use YYYY/MM/DD format")
        self.write(f"SYST:DATE {value.strip()}")

    def set_time(self, value: str) -> None:
        if not re.fullmatch(r"\d{1,2}:\d{1,2}:\d{1,2}", value.strip()):
            raise ValueError("Time must use HH:MM:SS format")
        self.write(f"SYST:TIME {value.strip()}")

    def set_power_on_mode(self, direct_power_on: bool) -> None:
        self._require_feature("power_on_mode", "POWER:ON:MODE")
        self.write(f"POWER:ON:MODE {1 if direct_power_on else 2}")

    def set_front_panel_keys(self, enabled: bool) -> None:
        self._require_feature("front_panel_keys", "KEY")
        self.write(f"KEY {'ON' if enabled else 'OFF'}")

    def buzzer_status(self) -> str:
        return self.query("BUZZ?")

    def set_buzzer(self, enabled: bool) -> str:
        self.write(f"BUZZ {'ON' if enabled else 'OFF'}")
        return self.buzzer_status()

    def set_manual_trigger_coupling(self, enabled: bool) -> str:
        self._require_feature("manual_trigger_coupling", "COUP TRDUCH")
        self.write(f"COUP TRDUCH,{'ON' if enabled else 'OFF'}")
        return self.coupling_status()

    def screen_saver_status(self) -> str:
        return self.query("SCSV?")

    def set_screen_saver(self, minutes: int | None) -> str:
        allowed = {1, 5, 15, 30, 60, 120, 300}
        if minutes is None or minutes == 0:
            value = "OFF"
        elif minutes in allowed:
            value = str(minutes)
        else:
            raise ValueError(f"Screen saver must be OFF or one of {sorted(allowed)} minutes")
        self.write(f"SCSV {value}")
        return self.screen_saver_status()

    def clock_status(self) -> str:
        return self.query("ROSC?")

    def set_clock_source(self, source: str) -> str:
        value = source.strip().upper()
        if value not in {"INT", "EXT"}:
            raise ValueError("Clock source must be INT or EXT")
        self.write(f"ROSC {value}")
        return self.clock_status()

    def virtual_key(self, value: str | int, pressed: bool = True) -> None:
        self.write(f"VKEY VALUE,{value},STATE,{'ON' if pressed else 'OFF'}")

    def network_status(self) -> dict[str, str]:
        return {
            "ip": self.query("SYST:COMM:LAN:IPAD?"),
            "mask": self.query("SYST:COMM:LAN:SMAS?"),
            "gateway": self.query("SYST:COMM:LAN:GAT?"),
        }

    def set_network(
        self,
        *,
        ip: str | None = None,
        mask: str | None = None,
        gateway: str | None = None,
    ) -> None:
        values = {"IPAD": ip, "SMAS": mask, "GAT": gateway}
        if not any(values.values()):
            raise ValueError("At least one network value is required")
        for header, value in values.items():
            if value is None:
                continue
            try:
                socket.inet_aton(value)
            except OSError as exc:
                raise ValueError(f"Invalid IPv4 address: {value}") from exc
            self.write(f"SYST:COMM:LAN:{header} {scpi_quote(value)}")

    def reset(self) -> None:
        self.write("*RST")


SiglentSDG = SDG2122X


def print_lines(lines: Iterable[str]) -> None:
    for line in lines:
        print(line)


# ---------------------------------------------------------------------------
# Full-screen terminal user interface (standard library only)
# ---------------------------------------------------------------------------


ANSI_RESET = "\x1b[0m"
ANSI_BOLD = "\x1b[1m"
ANSI_DIM = "\x1b[2m"
ANSI_REVERSE = "\x1b[7m"
ANSI_CYAN = "\x1b[36m"
ANSI_GREEN = "\x1b[32m"
ANSI_YELLOW = "\x1b[33m"
ANSI_RED = "\x1b[31m"
ANSI_BLUE_BG = "\x1b[44m"


@dataclass(frozen=True)
class TerminalEvent:
    kind: str
    key: str = ""
    text: str = ""
    x: int = 0
    y: int = 0
    button: int = 0


@dataclass
class MenuAction:
    label: str
    handler: Callable[[], None]
    hint: str = ""
    dangerous: bool = False


class TerminalSession:
    """ANSI full-screen terminal with keyboard and SGR mouse support."""

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
            "\x1b[?1049h"  # alternate screen
            "\x1b[?25l"    # hide cursor
            "\x1b[?1000h"  # mouse button events
            "\x1b[?1006h"  # SGR mouse coordinates
            "\x1b[2J\x1b[H"
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
        size = shutil.get_terminal_size((120, 36))
        return max(60, size.columns), max(20, size.lines)

    def draw(self, content: str) -> None:
        sys.stdout.write("\x1b[H" + content)
        sys.stdout.flush()

    def read_event(self, timeout: float = 0.1) -> TerminalEvent:
        if self._windows:
            return self._read_windows_event(timeout)
        return self._read_posix_event(timeout)

    def _enable_windows_vt(self) -> None:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        stdin_handle = kernel32.GetStdHandle(-10)
        stdout_handle = kernel32.GetStdHandle(-11)
        input_mode = ctypes.c_uint()
        output_mode = ctypes.c_uint()
        if kernel32.GetConsoleMode(stdin_handle, ctypes.byref(input_mode)):
            self._original_input_mode = input_mode.value
            # ENABLE_EXTENDED_FLAGS | ENABLE_WINDOW_INPUT | ENABLE_MOUSE_INPUT |
            # ENABLE_VIRTUAL_TERMINAL_INPUT
            kernel32.SetConsoleMode(stdin_handle, input_mode.value | 0x0080 | 0x0008 | 0x0010 | 0x0200)
        if kernel32.GetConsoleMode(stdout_handle, ctypes.byref(output_mode)):
            self._original_output_mode = output_mode.value
            # ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING
            kernel32.SetConsoleMode(stdout_handle, output_mode.value | 0x0001 | 0x0004)

    def _restore_windows_modes(self) -> None:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        if self._original_input_mode is not None:
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-10), self._original_input_mode)
        if self._original_output_mode is not None:
            kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), self._original_output_mode)

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
                        "H": "up", "P": "down", "K": "left", "M": "right",
                        "G": "home", "O": "end", "I": "pageup", "Q": "pagedown",
                        "?": "f5", "<": "f2", "S": "delete",
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
            "\x1b[A": "up", "\x1b[B": "down", "\x1b[C": "right", "\x1b[D": "left",
            "\x1b[H": "home", "\x1b[F": "end", "\x1b[5~": "pageup", "\x1b[6~": "pagedown",
            "\x1b[15~": "f5", "\x1b[12~": "f2", "\x1b": "escape",
        }
        for prefix, key in mappings.items():
            if sequence.startswith(prefix):
                return TerminalEvent("key", key=key)
        return TerminalEvent("key", key="escape")


def waveform_preview(waveform: str, width: int = 52, height: int = 9) -> list[str]:
    """Render a small local ASCII preview for the selected waveform type."""
    waveform = waveform.upper()
    width = max(16, width)
    height = max(5, height)
    canvas = [[" " for _ in range(width)] for _ in range(height)]
    middle = (height - 1) / 2
    for x in range(width):
        phase = 2 * math.pi * x / max(1, width - 1) * 2
        if waveform == "SQUARE":
            value = 1.0 if math.sin(phase) >= 0 else -1.0
        elif waveform in {"RAMP", "UPRAMP"}:
            value = 2 * ((x / max(1, width - 1) * 2) % 1) - 1
        elif waveform == "DNRAMP":
            value = 1 - 2 * ((x / max(1, width - 1) * 2) % 1)
        elif waveform == "PULSE":
            value = 1.0 if (x % max(2, width // 2)) < max(1, width // 10) else -1.0
        elif waveform == "DC":
            value = 0.45
        elif waveform == "NOISE":
            # deterministic pseudo-noise keeps redraws stable
            value = math.sin(x * 12.9898) * 0.85
        else:
            value = math.sin(phase)
        y = int(round(middle - value * (height - 2) / 2))
        y = max(0, min(height - 1, y))
        canvas[y][x] = "*"
    return ["".join(row) for row in canvas]


def display_width(text: str) -> int:
    plain = re.sub(r"\x1b\[[0-9;?]*[A-Za-z]", "", text)
    return sum(2 if unicodedata.east_asian_width(char) in {"W", "F"} else 1 for char in plain)


def fit_display(text: str, width: int) -> str:
    """Truncate/pad ANSI text to an approximate terminal cell width."""
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


class SDGTerminalApp:
    """Keyboard/mouse TUI for discovered SCPI instruments and SIGLENT SDG control."""

    TABS = (
        "总览", "通道", "调制", "扫频", "Burst", "任意波", "同步/高级", "系统", "SCPI控制台"
    )

    def __init__(
        self,
        host: str | None,
        port: int,
        timeout: float,
        *,
        discovery_subnets: Sequence[str] = (),
        scan_timeout: float = DEFAULT_SCAN_TIMEOUT,
        cache_path: Path | str | None = DEFAULT_CACHE_PATH,
        auto_discover: bool = True,
    ) -> None:
        self.requested_host = host
        self.host = host or DEFAULT_HOST
        self.port = port
        self.timeout = timeout
        self.discovery_subnets = tuple(discovery_subnets)
        self.scan_timeout = scan_timeout
        self.cache_path = cache_path
        self.auto_discover = auto_discover
        self.instrument = SDG2122X(self.host, self.port, timeout)
        self.terminal: TerminalSession | None = None
        self.running = True
        self.tab_index = 0
        self.channel = 1
        self.selected = 0
        self.scroll = 0
        self.identity = "未连接"
        self.sdg_compatible = True
        self.waveform: dict[int, dict[str, str]] = {1: {}, 2: {}}
        self.waveform_raw: dict[int, str] = {1: "", 2: ""}
        self.output: dict[int, dict[str, str]] = {
            1: {"STATE": "OFF", "LOAD": "?", "PLRT": "?"},
            2: {"STATE": "OFF", "LOAD": "?", "PLRT": "?"},
        }
        self.mode_cache: dict[str, dict[int, dict[str, str]]] = {
            "mod": {1: {}, 2: {}},
            "sweep": {1: {}, 2: {}},
            "burst": {1: {}, 2: {}},
        }
        self.modulation_type: dict[int, str] = {1: "AM", 2: "AM"}
        self.logs: list[str] = []
        self.status = "方向键/鼠标选择，Enter 执行，1/2 切换通道，F5 刷新，Esc/Q 退出"
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
            if os.environ.get("SDG_TUI_TEST") == "1":
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

    def _choose_discovered_device(
        self, candidates: Sequence[DiscoveredInstrument]
    ) -> DiscoveredInstrument | None:
        if not candidates:
            return None
        if self.terminal is None:
            return None
        last = load_last_device(self.cache_path)
        labels = [
            (
                "★ 上次设备  "
                if last is not None
                and instrument_identity_key(candidate.identity)
                == instrument_identity_key(last.identity)
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
        if selected is None:
            return None
        return candidates[labels.index(selected)]

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
        visible_devices = max(1, height - 13)
        device_lines = [
            f"  {index:02d}. {candidate.display_name()}"
            for index, candidate in enumerate(progress.candidates[-visible_devices:], 1)
        ]
        self._overlay_title = "正在扫描 SCPI 设备"
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
        if not self._scan_overlay_active:
            return
        self._scan_overlay_active = False
        self._scan_last_render = 0.0
        self._scan_last_found = -1
        self._overlay_title = None
        self._overlay_lines = []
        self._overlay_selected = 0
        self._overlay_wide = False

    def _connect_candidate(
        self, candidate: DiscoveredInstrument, source: str = "discovery"
    ) -> bool:
        replacement = self.instrument
        same_endpoint = (
            self.instrument.is_connected
            and self.instrument.host == candidate.host
            and self.instrument.port == candidate.port
        )
        if same_endpoint:
            try:
                # Discovery probes may invalidate an existing control socket on
                # instruments that only keep one LAN session.  Re-selecting the
                # current endpoint must therefore create a fresh session.
                replacement.reconnect()
                identity = replacement.identify()
            except Exception as exc:
                replacement.close()
                self.identity = "未连接"
                self._log(
                    f"重新连接 {candidate.host}:{candidate.port} 失败: {exc}；"
                    "不会自动重新扫描",
                    error=True,
                )
                return False
        else:
            replacement = SDG2122X(candidate.host, candidate.port, self.timeout)
            try:
                replacement.connect()
                identity = replacement.identify()
            except Exception as exc:
                replacement.close()
                self._log(
                    f"连接 {candidate.host}:{candidate.port} 失败: {exc}；"
                    "不会自动重新扫描",
                    error=True,
                )
                return False
            self.instrument.close()
        self.instrument = replacement
        self.host = candidate.host
        self.port = candidate.port
        self.identity = identity
        self.sdg_compatible = is_siglent_sdg_identity(identity)
        save_last_device(
            DiscoveredInstrument(self.host, self.port, self.identity), self.cache_path
        )
        source_names = {
            "explicit": "指定地址",
            "last": "上次设备",
            "default": "默认地址",
            "reconnect": "当前地址",
            "manual-selection": "扫描选择",
            "discovery": "自动发现",
        }
        self._log(
            f"已通过{source_names.get(source, source)}连接 "
            f"{self.host}:{self.port} {self.identity}"
        )
        if self.sdg_compatible:
            try:
                self._refresh_all()
            except Exception as exc:
                self._log(f"设备已连接，但 SDG 状态读取失败: {exc}", error=True)
        else:
            self.tab_index = len(self.TABS) - 1
            self.selected = self.scroll = 0
            self._log("该设备不是已识别的 SIGLENT SDG，已切换到通用 SCPI 控制台")
        return True

    def _connect_with_discovery(self, *, force_scan: bool = False) -> bool:
        scan_started = False

        def progress_callback(progress: DiscoveryProgress) -> None:
            nonlocal scan_started
            scan_started = True
            self._show_scan_progress(progress)

        try:
            location = locate_instrument(
                None if force_scan else self.requested_host,
                self.port,
                self.timeout,
                subnets=self.discovery_subnets,
                scan_timeout=self.scan_timeout,
                cache_path=self.cache_path,
                allow_scan=self.auto_discover or force_scan,
                force_scan=force_scan,
                progress_callback=progress_callback,
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
            candidate = self._choose_discovered_device(location.candidates)
            source = "manual-selection"
        if candidate is None:
            if location.candidates:
                self._log(
                    f"扫描发现 {len(location.candidates)} 台设备，但用户未选择连接目标",
                    error=True,
                )
            elif location.source == "explicit-not-found":
                self._log(
                    f"指定地址 {self.requested_host}:{self.port} 未响应 *IDN?；"
                    "显式地址不会回退到其他设备",
                    error=True,
                )
            else:
                networks = ", ".join(location.networks) or "未生成可扫描网段"
                self._log(f"未发现 SCPI 设备；扫描范围: {networks}", error=True)
            return False
        if force_scan:
            self.requested_host = None
        return self._connect_candidate(candidate, source)

    def _log(self, message: str, *, error: bool = False) -> None:
        stamp = time.strftime("%H:%M:%S")
        prefix = "ERR" if error else "OK "
        clean = " ".join(str(message).split())
        if len(clean) > 600:
            clean = clean[:597] + "..."
        self.logs.append(f"{stamp} {prefix} {clean}")
        self.logs = self.logs[-100:]
        self.status = clean

    def _safe(
        self,
        description: str,
        callback: Callable[[], Any],
        *,
        refresh: bool = False,
        refresh_channel: int | None = None,
        allow_generic: bool = False,
    ) -> Any:
        try:
            if not self.instrument.is_connected:
                raise InstrumentError(
                    "No SCPI instrument is connected; use 连接/断开设备 or "
                    "扫描并选择 SCPI 设备"
                )
            if not self.sdg_compatible and not allow_generic:
                raise InstrumentError(
                    "The connected device is not a recognized SIGLENT SDG; use the SCPI console"
                )
            result = callback()
            if result is not None:
                self._log(f"{description}: {result}")
            else:
                self._log(f"{description}: 完成")
            if refresh_channel is not None:
                self._refresh_channel(refresh_channel)
            elif refresh:
                self._refresh_channel(self.channel)
            return result
        except Exception as exc:
            self._log(f"{description}失败: {exc}", error=True)
            return None

    @staticmethod
    def _parse_output_response(response: str) -> dict[str, str]:
        payload = response.split(" ", 1)[1] if " " in response else response
        tokens = [token.strip() for token in payload.split(",") if token.strip()]
        result = {"STATE": tokens[0] if tokens else "?", "LOAD": "?", "PLRT": "?"}
        index = 1
        while index + 1 < len(tokens):
            result[tokens[index].upper()] = tokens[index + 1]
            index += 2
        return result

    def _refresh_channel(self, channel: int) -> None:
        wave = self.instrument.basic_waveform(channel)
        output = self.instrument.output_status(channel)
        self.waveform_raw[channel] = wave
        self.waveform[channel] = parse_scpi_pairs(wave)
        self.output[channel] = self._parse_output_response(output)

    def _refresh_all(self) -> None:
        if not self.instrument.is_connected:
            self.instrument.connect()
        self.identity = self.instrument.identify()
        self.sdg_compatible = is_siglent_sdg_identity(self.identity)
        save_last_device(
            DiscoveredInstrument(self.host, self.port, self.identity), self.cache_path
        )
        for channel in (1, 2):
            self._refresh_channel(channel)
        self._log("设备状态已刷新")

    def _refresh_device_status(self) -> None:
        if self.sdg_compatible:
            self._refresh_all()
            return
        self.identity = self.instrument.identify()
        save_last_device(
            DiscoveredInstrument(self.host, self.port, self.identity), self.cache_path
        )
        self._log(f"设备身份: {self.identity}")

    def _refresh_current_mode(self) -> None:
        if self.instrument.is_connected and not self.sdg_compatible:
            self._refresh_device_status()
            return
        tab = self.TABS[self.tab_index]
        if tab == "调制":
            response = self.instrument.modulation_status(self.channel)
            self.mode_cache["mod"][self.channel] = parse_scpi_pairs(response)
            first = response.split(" ", 1)[-1].split(",", 1)[0].strip().upper()
            if first in {"AM", "DSBAM", "FM", "PM", "PWM", "ASK", "FSK", "PSK"}:
                self.modulation_type[self.channel] = first
            self._log(f"CH{self.channel} 调制状态: {response}")
        elif tab == "扫频":
            response = self.instrument.sweep_status(self.channel)
            self.mode_cache["sweep"][self.channel] = parse_scpi_pairs(response)
            self._log(f"CH{self.channel} 扫频状态: {response}")
        elif tab == "Burst":
            response = self.instrument.burst_status(self.channel)
            self.mode_cache["burst"][self.channel] = parse_scpi_pairs(response)
            self._log(f"CH{self.channel} Burst 状态: {response}")
        else:
            self._refresh_channel(self.channel)
            self._log(f"CH{self.channel} 状态已刷新")

    def _render(self) -> None:
        assert self.terminal is not None
        width, height = self.terminal.size()
        actions = self._actions()
        self.selected = max(0, min(self.selected, max(0, len(actions) - 1)))

        connected = self.instrument.is_connected
        connection_text = f"{ANSI_GREEN}ONLINE{ANSI_RESET}" if connected else f"{ANSI_RED}OFFLINE{ANSI_RESET}"
        model_text = (
            self.instrument.model_profile.model
            if connected and self.sdg_compatible
            else "SCPI"
        )
        header = (
            f"{ANSI_BLUE_BG}{ANSI_BOLD} SCPI TERMINAL CONTROL {ANSI_RESET} "
            f"{connection_text}  {model_text}  {self.host}:{self.port}  当前 CH{self.channel}"
        )
        lines = [fit_display(header, width)]

        tab_line = ""
        self._tab_ranges.clear()
        cursor = 1
        for index, tab in enumerate(self.TABS):
            label = f" {tab} "
            styled = ANSI_REVERSE + label + ANSI_RESET if index == self.tab_index else label
            start = cursor
            tab_line += styled
            cursor += display_width(label)
            self._tab_ranges.append((start, cursor - 1, index))
        lines.append(fit_display(tab_line, width))

        lines.extend(self._channel_summary_lines(width))
        content_top = len(lines) + 1
        log_height = 5
        content_height = max(6, height - content_top - log_height - 2)
        left_width = width if width < 100 else min(66, width // 2 + 4)
        right_width = max(0, width - left_width - 2)
        visible_count = max(1, content_height)
        if self.selected < self.scroll:
            self.scroll = self.selected
        if self.selected >= self.scroll + visible_count:
            self.scroll = self.selected - visible_count + 1

        menu_lines: list[str] = []
        self._menu_rows.clear()
        for row_offset in range(visible_count):
            action_index = self.scroll + row_offset
            if action_index < len(actions):
                action = actions[action_index]
                marker = "▶" if action_index == self.selected else " "
                danger = f"{ANSI_RED}!{ANSI_RESET}" if action.dangerous else " "
                text = f"{marker}{danger} {action_index + 1:02d}. {action.label}"
                if action.hint:
                    text += f"  {ANSI_DIM}{action.hint}{ANSI_RESET}"
                if action_index == self.selected:
                    text = ANSI_REVERSE + text + ANSI_RESET
                menu_lines.append(fit_display(text, left_width))
                self._menu_rows[content_top + row_offset] = action_index
            else:
                menu_lines.append(" " * left_width)

        side_lines = self._side_panel(right_width, visible_count) if right_width else []
        for index in range(visible_count):
            if right_width:
                lines.append(menu_lines[index] + "  " + fit_display(side_lines[index], right_width))
            else:
                lines.append(menu_lines[index])

        lines.append(fit_display("─" * width, width))
        for log_line in self.logs[-log_height:]:
            color = ANSI_RED if " ERR " in log_line else ANSI_DIM
            lines.append(fit_display(color + log_line + ANSI_RESET, width))
        while len(lines) < height - 1:
            lines.append(" " * width)
        footer = f"{ANSI_BLUE_BG} ←/→页签  ↑/↓选择  Enter执行  鼠标可点击  1/2通道  F5刷新  C控制台  Q退出 | {self.status} {ANSI_RESET}"
        lines.append(fit_display(footer, width))

        if self._overlay_title is not None:
            lines = self._render_overlay(lines, width, height)
        self.terminal.draw("\x1b[2J" + "\n".join(lines[:height]))

    def _channel_summary_lines(self, width: int) -> list[str]:
        if self.instrument.is_connected and not self.sdg_compatible:
            return [
                fit_display(f"设备: {self.identity}", width),
                fit_display("模式: 通用 SCPI（SIGLENT SDG 专用页已停用）", width),
            ]
        summaries: list[str] = [
            fit_display(f"设备档案: {self.instrument.model_profile.summary()}", width)
        ]
        for channel in (1, 2):
            wave = self.waveform[channel]
            output = self.output[channel]
            state = output.get("STATE", "?")
            state_color = ANSI_GREEN if state == "ON" else ANSI_YELLOW
            text = (
                f"CH{channel}: {state_color}{state}{ANSI_RESET}  "
                f"{wave.get('WVTP', '?')}  FRQ={wave.get('FRQ', '?')}  "
                f"AMP={wave.get('AMP', wave.get('AMPVRMS', '?'))}  "
                f"OFST={wave.get('OFST', '?')}  LOAD={output.get('LOAD', '?')}"
            )
            if channel == self.channel:
                text = ANSI_BOLD + text + ANSI_RESET
            summaries.append(fit_display(text, width))
        return summaries

    def _side_panel(self, width: int, height: int) -> list[str]:
        if self.instrument.is_connected and not self.sdg_compatible:
            lines = [
                f"{ANSI_BOLD}通用 SCPI 设备{ANSI_RESET}",
                self.identity,
                "",
                "请使用 SCPI控制台发送该设备支持的命令。",
                "SIGLENT SDG 专用通道菜单已停用。",
            ]
            return (lines + [""] * height)[:height]
        wave = self.waveform[self.channel]
        output = self.output[self.channel]
        wave_type = wave.get("WVTP", "SINE")
        lines = [f"{ANSI_BOLD}CH{self.channel} 本地波形预览 ({wave_type}){ANSI_RESET}"]
        lines.extend(waveform_preview(wave_type, max(16, width - 1), min(9, max(5, height - 8))))
        lines.extend(
            [
                f"频率: {wave.get('FRQ', '?')}",
                f"幅度: {wave.get('AMP', '?')} / {wave.get('AMPVRMS', '?')}",
                f"偏置: {wave.get('OFST', '?')}  相位: {wave.get('PHSE', '?')}",
                f"输出: {output.get('STATE', '?')}  负载: {output.get('LOAD', '?')}",
                f"型号: {self.instrument.model_profile.summary()}",
            ]
        )
        return (lines + [""] * height)[:height]

    def _render_overlay(self, lines: list[str], width: int, height: int) -> list[str]:
        box_width = (
            max(4, width - 4)
            if self._overlay_wide
            else min(max(44, width * 2 // 3), width - 4)
        )
        content = list(self._overlay_lines)
        if self._prompt_buffer is not None:
            before = self._prompt_buffer[: self._prompt_cursor]
            current = self._prompt_buffer[self._prompt_cursor : self._prompt_cursor + 1] or " "
            after = self._prompt_buffer[self._prompt_cursor + (1 if self._prompt_cursor < len(self._prompt_buffer) else 0) :]
            content.append("> " + before + ANSI_REVERSE + current + ANSI_RESET + after)
        box_height = min(len(content) + 4, height - 4)
        top = max(1, (height - box_height) // 2)
        left = max(1, (width - box_width) // 2)
        border = "┌" + "─" * (box_width - 2) + "┐"
        lines[top] = fit_display(" " * left + border, width)
        title = f" {self._overlay_title or ''} "
        lines[top + 1] = fit_display(" " * left + "│" + fit_display(ANSI_BOLD + title + ANSI_RESET, box_width - 2) + "│", width)
        self._overlay_rows.clear()
        for index in range(box_height - 3):
            source = content[index] if index < len(content) else ""
            if self._prompt_buffer is None and index < len(self._overlay_lines):
                self._overlay_rows[top + 2 + index] = index
                if index == self._overlay_selected:
                    source = ANSI_REVERSE + source + ANSI_RESET
            lines[top + 2 + index] = fit_display(
                " " * left + "│" + fit_display(source, box_width - 2) + "│", width
            )
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
                if event.y == 2 and start <= event.x <= end:
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
            elif key in {"1", "2"}:
                self.channel = int(key)
                self._log(f"切换到 CH{self.channel}")
            elif key == "r":
                self._safe("刷新", self._refresh_current_mode, allow_generic=True)
            elif key == "c":
                self.tab_index = len(self.TABS) - 1
                self.selected = self.scroll = 0
            elif key in {"j", "s"}:
                self.selected = min(len(actions) - 1, self.selected + 1)
            elif key in {"k", "w"}:
                self.selected = max(0, self.selected - 1)
            return
        if event.kind != "key":
            return
        if event.key in {"escape", "ctrl_c"}:
            self.running = False
        elif event.key in {"up"}:
            self.selected = max(0, self.selected - 1)
        elif event.key in {"down"}:
            self.selected = min(max(0, len(actions) - 1), self.selected + 1)
        elif event.key == "pageup":
            self.selected = max(0, self.selected - 8)
        elif event.key == "pagedown":
            self.selected = min(max(0, len(actions) - 1), self.selected + 8)
        elif event.key in {"left"}:
            self.tab_index = (self.tab_index - 1) % len(self.TABS)
            self.selected = self.scroll = 0
        elif event.key in {"right", "tab"}:
            self.tab_index = (self.tab_index + 1) % len(self.TABS)
            self.selected = self.scroll = 0
        elif event.key in {"enter", "f2"} and actions:
            actions[self.selected].handler()
        elif event.key in {"f5", "ctrl_l"}:
            self._safe("刷新", self._refresh_current_mode, allow_generic=True)

    def prompt_text(self, title: str, initial: str = "", *, password: bool = False) -> str | None:
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
                self._prompt_buffer = (
                    self._prompt_buffer[: self._prompt_cursor]
                    + event.text
                    + self._prompt_buffer[self._prompt_cursor :]
                )
                self._prompt_cursor += len(event.text)
            elif event.kind == "key":
                if event.key == "enter":
                    result = self._prompt_buffer
                    break
                if event.key in {"escape", "ctrl_c"}:
                    result = None
                    break
                if event.key == "backspace" and self._prompt_cursor > 0:
                    self._prompt_buffer = (
                        self._prompt_buffer[: self._prompt_cursor - 1]
                        + self._prompt_buffer[self._prompt_cursor :]
                    )
                    self._prompt_cursor -= 1
                elif event.key == "delete" and self._prompt_cursor < len(self._prompt_buffer):
                    self._prompt_buffer = (
                        self._prompt_buffer[: self._prompt_cursor]
                        + self._prompt_buffer[self._prompt_cursor + 1 :]
                    )
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
        all_lines = [
            f" {index + 1:0{number_width}d}. {choice}"
            for index, choice in enumerate(choices)
        ]
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
            self._overlay_title = (
                f"{title}  [{selected + 1}/{len(choices)}]  "
                "(↑↓/PgUp/PgDn/滚轮，Enter确认)"
            )
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

    def show_text(self, title: str, text: str) -> None:
        assert self.terminal is not None
        self._overlay_wide = False
        width, height = self.terminal.size()
        wrap_width = max(30, min(100, width - 12))
        source_lines: list[str] = []
        for raw_line in str(text).splitlines() or [""]:
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
            elif event.kind == "mouse":
                if event.button in {64, 65}:
                    delta = -3 if event.button == 64 else 3
                    offset = min(max(0, offset + delta), max(0, len(source_lines) - page_size))
        self._overlay_title = None
        self._overlay_lines = []
        self._overlay_selected = 0

    def prompt_number(self, title: str, initial: str = "") -> float | None:
        value = self.prompt_text(title, initial)
        if value is None:
            return None
        try:
            return parse_engineering_number(value)
        except argparse.ArgumentTypeError as exc:
            self._log(str(exc), error=True)
            return None

    def _actions(self) -> list[MenuAction]:
        tab = self.TABS[self.tab_index]
        if not self.sdg_compatible and tab not in {"总览", "SCPI控制台"}:
            def open_console() -> None:
                self.tab_index = len(self.TABS) - 1
                self.selected = self.scroll = 0

            return [
                MenuAction("当前设备为通用 SCPI 设备，打开 SCPI 控制台", open_console),
                MenuAction(
                    "读取设备身份",
                    lambda: self._safe(
                        "设备身份", self.instrument.identify, allow_generic=True
                    ),
                ),
            ]
        builders: dict[str, Callable[[], list[MenuAction]]] = {
            "总览": self._dashboard_actions,
            "通道": self._channel_actions,
            "调制": self._modulation_actions,
            "扫频": self._sweep_actions,
            "Burst": self._burst_actions,
            "任意波": self._arbitrary_actions,
            "同步/高级": self._advanced_actions,
            "系统": self._system_actions,
            "SCPI控制台": self._console_actions,
        }
        return builders[tab]()

    def _run_and_refresh(self, description: str, callback: Callable[[], Any]) -> None:
        self._safe(description, callback, refresh=True)

    def _choose_then(
        self,
        title: str,
        choices: Sequence[str],
        callback: Callable[[str], Any],
        current: str | None = None,
    ) -> None:
        value = self.prompt_choice(title, choices, current)
        if value is not None:
            self._safe(title, lambda: callback(value), refresh=True)

    def _number_then(
        self,
        title: str,
        callback: Callable[[float], Any],
        initial: str = "",
        *,
        refresh: bool = True,
    ) -> None:
        value = self.prompt_number(title, initial)
        if value is not None:
            self._safe(title, lambda: callback(value), refresh=refresh)

    def _text_then(
        self,
        title: str,
        callback: Callable[[str], Any],
        initial: str = "",
        *,
        refresh: bool = True,
    ) -> None:
        value = self.prompt_text(title, initial)
        if value is not None and value.strip():
            self._safe(title, lambda: callback(value.strip()), refresh=refresh)

    def _toggle_output(self, channel: int) -> None:
        enabled = self.output[channel].get("STATE", "OFF") != "ON"
        self._safe(
            f"CH{channel} 输出 {'ON' if enabled else 'OFF'}",
            lambda: self.instrument.set_output(channel, enabled),
            refresh_channel=channel,
        )

    def _set_basic_number(self, parameter: str, title: str) -> None:
        current = self.waveform[self.channel].get(parameter, "")
        value = self.prompt_number(title, re.sub(r"[^0-9eE+\-.]", "", current))
        if value is not None:
            self._safe(
                title,
                lambda: self.instrument.set_basic_wave(self.channel, [(parameter, value)]),
                refresh=True,
            )

    def _set_basic_choice(self, parameter: str, title: str, choices: Sequence[str]) -> None:
        current = self.waveform[self.channel].get(parameter, choices[0])
        self._choose_then(
            title,
            choices,
            lambda value: self.instrument.set_basic_wave(self.channel, [(parameter, value)]),
            current,
        )

    def _toggle_channel_query(
        self,
        description: str,
        query: Callable[[], str],
        setter: Callable[[bool], str],
    ) -> None:
        response = self._safe(description + "状态查询", query)
        if response is None:
            return
        enabled = "ON" not in str(response).upper()
        self._safe(description, lambda: setter(enabled), refresh=True)

    def _set_family_number(self, family: str, parameter: str, title: str) -> None:
        cache_name = {"MDWV": "mod", "SWWV": "sweep", "BTWV": "burst"}[family]
        current = self.mode_cache[cache_name][self.channel].get(parameter, "")
        initial = re.sub(r"[^0-9eE+\-.]", "", current)
        value = self.prompt_number(title, initial)
        if value is None:
            return
        if family == "MDWV":
            modulation_type = self.modulation_type[self.channel]
            callback = lambda: self.instrument.set_modulation(
                self.channel, modulation_type, [(parameter, value)]
            )
        elif family == "SWWV":
            callback = lambda: self.instrument.set_sweep(self.channel, [(parameter, value)])
        else:
            callback = lambda: self.instrument.set_burst(self.channel, [(parameter, value)])
        self._safe(title, callback)
        self._safe("刷新模式状态", self._refresh_current_mode)

    def _set_family_choice(
        self,
        family: str,
        parameter: str,
        title: str,
        choices: Sequence[str],
        *,
        include_mod_type: bool = True,
    ) -> None:
        cache_name = {"MDWV": "mod", "SWWV": "sweep", "BTWV": "burst"}[family]
        current = self.mode_cache[cache_name][self.channel].get(parameter, choices[0])
        value = self.prompt_choice(title, choices, current)
        if value is None:
            return
        if family == "MDWV":
            modulation_type = self.modulation_type[self.channel] if include_mod_type else None
            callback = lambda: self.instrument.set_modulation(
                self.channel, modulation_type, [(parameter, value)]
            )
        elif family == "SWWV":
            callback = lambda: self.instrument.set_sweep(self.channel, [(parameter, value)])
        else:
            callback = lambda: self.instrument.set_burst(self.channel, [(parameter, value)])
        self._safe(title, callback)
        self._safe("刷新模式状态", self._refresh_current_mode)

    def _set_carrier_value(self, family: str, parameter: str, title: str, value: Any) -> Any:
        response = self.instrument.set_channel_tokens(
            self.channel, family, ("CARR", parameter, value)
        )
        return response

    def _carrier_number(self, family: str, parameter: str, title: str) -> None:
        value = self.prompt_number(title)
        if value is not None:
            self._safe(title, lambda: self._set_carrier_value(family, parameter, title, value))
            self._safe("刷新模式状态", self._refresh_current_mode)

    def _carrier_choice(
        self, family: str, parameter: str, title: str, choices: Sequence[str]
    ) -> None:
        value = self.prompt_choice(title, choices)
        if value is not None:
            self._safe(title, lambda: self._set_carrier_value(family, parameter, title, value))
            self._safe("刷新模式状态", self._refresh_current_mode)

    def _dashboard_actions(self) -> list[MenuAction]:
        def connect_toggle() -> None:
            if self.instrument.is_connected:
                self.instrument.close()
                self.identity = "未连接"
                self._log("连接已关闭")
            else:
                candidate = DiscoveredInstrument(
                    self.host,
                    self.port,
                    "" if self.identity == "未连接" else self.identity,
                )
                self._connect_candidate(candidate, source="reconnect")

        def scan_and_select() -> None:
            self._connect_with_discovery(force_scan=True)

        def open_console() -> None:
            self.tab_index = len(self.TABS) - 1
            self.selected = self.scroll = 0

        def reset_device() -> None:
            if self.confirm("恢复仪器默认设置？该操作会改变所有通道状态"):
                self._safe("恢复默认设置", self.instrument.reset)

        connection_actions = [
            MenuAction("连接/断开设备", connect_toggle, self.identity),
            MenuAction("扫描并选择 SCPI 设备", scan_and_select),
        ]
        if not self.sdg_compatible:
            return connection_actions + [
                MenuAction(
                    "读取设备身份",
                    lambda: self._safe(
                        "设备身份", self.instrument.identify, allow_generic=True
                    ),
                ),
                MenuAction(
                    "读取操作完成状态",
                    lambda: self._safe(
                        "OPC", self.instrument.operation_complete, allow_generic=True
                    ),
                ),
                MenuAction(
                    "读取系统错误",
                    lambda: self._safe(
                        "系统错误", self.instrument.system_error, allow_generic=True
                    ),
                ),
                MenuAction("打开通用 SCPI 控制台", open_console),
            ]

        return connection_actions + [
            MenuAction("刷新全部状态", lambda: self._safe("刷新全部状态", self._refresh_all)),
            MenuAction("切换 CH1 输出", lambda: self._toggle_output(1), self.output[1].get("STATE", "?")),
            MenuAction("切换 CH2 输出", lambda: self._toggle_output(2), self.output[2].get("STATE", "?")),
            MenuAction("两通道同相位", lambda: self._safe("同相位", self.instrument.equal_phase)),
            MenuAction("复制 CH1 -> CH2", lambda: self._safe("复制通道", lambda: self.instrument.copy_channel(1, 2))),
            MenuAction("复制 CH2 -> CH1", lambda: self._safe("复制通道", lambda: self.instrument.copy_channel(2, 1))),
            MenuAction("读取操作完成状态", lambda: self._safe("OPC", self.instrument.operation_complete)),
            MenuAction("读取系统错误", lambda: self._safe("系统错误", self.instrument.system_error)),
            MenuAction("恢复默认设置 (*RST)", reset_device, dangerous=True),
        ]

    def _channel_actions(self) -> list[MenuAction]:
        channel = self.channel
        wave = self.waveform[channel]
        output = self.output[channel]

        def set_load() -> None:
            value = self.prompt_choice("输出负载", ("HZ", "50", "75", "600"), output.get("LOAD", "HZ"))
            if value is not None:
                self._safe(
                    "设置负载",
                    lambda: self.instrument.configure_output(channel, load=value),
                    refresh=True,
                )

        def set_polarity() -> None:
            value = self.prompt_choice("输出极性", ("NOR", "INVT"), output.get("PLRT", "NOR"))
            if value is not None:
                self._safe(
                    "设置极性",
                    lambda: self.instrument.configure_output(channel, polarity=value),
                    refresh=True,
                )

        actions = [
            MenuAction("刷新通道状态", lambda: self._safe("刷新通道", lambda: self._refresh_channel(channel))),
            MenuAction("切换输出 ON/OFF", lambda: self._toggle_output(channel), output.get("STATE", "?")),
            MenuAction("波形类型 WVTP", lambda: self._set_basic_choice("WVTP", "波形类型", ("SINE", "SQUARE", "RAMP", "PULSE", "NOISE", "ARB", "DC")), wave.get("WVTP", "?")),
            MenuAction("频率 FRQ", lambda: self._set_basic_number("FRQ", "频率 (Hz，支持 10k/1M)"), wave.get("FRQ", "?")),
            MenuAction("幅度 AMP (Vpp)", lambda: self._set_basic_number("AMP", "幅度 Vpp"), wave.get("AMP", "?")),
            MenuAction("幅度 AMPVRMS (Vrms)", lambda: self._set_basic_number("AMPVRMS", "幅度 Vrms"), wave.get("AMPVRMS", "?")),
            MenuAction("幅度 AMPDBM (dBm)", lambda: self._set_basic_number("AMPDBM", "幅度 dBm"), wave.get("AMPDBM", "?")),
            MenuAction("直流偏置 OFST", lambda: self._set_basic_number("OFST", "偏置 V"), wave.get("OFST", "?")),
            MenuAction("相位 PHSE", lambda: self._set_basic_number("PHSE", "相位 0-360 度"), wave.get("PHSE", "?")),
            MenuAction("三角波对称度 SYM", lambda: self._set_basic_number("SYM", "对称度 %"), wave.get("SYM", "?")),
            MenuAction("方波/脉冲占空比 DUTY", lambda: self._set_basic_number("DUTY", "占空比 %"), wave.get("DUTY", "?")),
            MenuAction("脉冲宽度 WIDTH", lambda: self._set_basic_number("WIDTH", "脉冲宽度 s"), wave.get("WIDTH", "?")),
            MenuAction("上升时间 RISE", lambda: self._set_basic_number("RISE", "上升时间 s"), wave.get("RISE", "?")),
            MenuAction("下降时间 FALL", lambda: self._set_basic_number("FALL", "下降时间 s"), wave.get("FALL", "?")),
            MenuAction("波形延迟 DLY", lambda: self._set_basic_number("DLY", "延迟 s"), wave.get("DLY", "?")),
            MenuAction("高电平 HLEV", lambda: self._set_basic_number("HLEV", "高电平 V"), wave.get("HLEV", "?")),
            MenuAction("低电平 LLEV", lambda: self._set_basic_number("LLEV", "低电平 V"), wave.get("LLEV", "?")),
            MenuAction("噪声标准差 STDEV", lambda: self._set_basic_number("STDEV", "噪声标准差 V"), wave.get("STDEV", "?")),
            MenuAction("噪声均值 MEAN", lambda: self._set_basic_number("MEAN", "噪声均值 V"), wave.get("MEAN", "?")),
            MenuAction("噪声带宽开关 BANDSTATE", lambda: self._set_basic_choice("BANDSTATE", "噪声带宽", ("ON", "OFF")), wave.get("BANDSTATE", "?")),
            MenuAction("噪声带宽 BANDWIDTH", lambda: self._set_basic_number("BANDWIDTH", "噪声带宽 Hz"), wave.get("BANDWIDTH", "?")),
            MenuAction("最大幅度限制", lambda: self._set_basic_number("MAX_OUTPUT_AMP", "最大输出幅度 Vpp"), wave.get("MAX_OUTPUT_AMP", "?")),
            MenuAction("输出负载 LOAD", set_load, output.get("LOAD", "?")),
            MenuAction("输出极性 PLRT", set_polarity, output.get("PLRT", "?")),
            MenuAction("通道反相 INVT", lambda: self._toggle_channel_query("通道反相", lambda: self.instrument.invert_status(channel), lambda enabled: self.instrument.set_invert(channel, enabled))),
            MenuAction("同步输出 SYNC", lambda: self._toggle_channel_query("同步输出", lambda: self.instrument.sync_status(channel), lambda enabled: self.instrument.set_sync(channel, enabled))),
            MenuAction("波形合并 CMBN", lambda: self._toggle_channel_query("波形合并", lambda: self.instrument.combine_status(channel), lambda enabled: self.instrument.set_combine(channel, enabled))),
            MenuAction("采样模式 DDS/TARB", lambda: self._choose_then("采样模式", ("DDS", "TARB"), lambda value: self.instrument.set_sample_rate(channel, mode=value))),
            MenuAction("TrueArb 采样率", lambda: self._number_then("采样率 Sa/s", lambda value: self.instrument.set_sample_rate(channel, value=value))),
        ]
        if not self.instrument.model_profile.supports("sample_rate"):
            actions = [
                action
                for action in actions
                if action.label not in {"采样模式 DDS/TARB", "TrueArb 采样率"}
            ]
        return actions

    def _modulation_actions(self) -> list[MenuAction]:
        channel = self.channel
        modulation_type = self.modulation_type[channel]
        cache = self.mode_cache["mod"][channel]

        def set_type() -> None:
            value = self.prompt_choice("调制类型", ("AM", "DSBAM", "FM", "PM", "PWM", "ASK", "FSK", "PSK"), modulation_type)
            if value is not None:
                self.modulation_type[channel] = value
                self._safe("设置调制类型", lambda: self.instrument.set_modulation(channel, value))
                self._safe("刷新调制", self._refresh_current_mode)

        def toggle_state() -> None:
            enabled = cache.get("STATE", "OFF") != "ON"
            self._safe(
                "调制开关",
                lambda: self.instrument.set_modulation(channel, None, [("STATE", enabled)]),
            )
            self._safe("刷新调制", self._refresh_current_mode)

        return [
            MenuAction("刷新调制状态", lambda: self._safe("刷新调制", self._refresh_current_mode)),
            MenuAction("调制类型", set_type, modulation_type),
            MenuAction("调制开关 STATE", toggle_state, cache.get("STATE", "?")),
            MenuAction("调制源 SRC", lambda: self._set_family_choice("MDWV", "SRC", "调制源", ("INT", "EXT", "CH1", "CH2"))),
            MenuAction("内部调制波形 MDSP", lambda: self._set_family_choice("MDWV", "MDSP", "调制波形", ("SINE", "SQUARE", "TRIANGLE", "UPRAMP", "DNRAMP", "NOISE", "ARB"))),
            MenuAction("调制频率 FRQ", lambda: self._set_family_number("MDWV", "FRQ", "调制频率 Hz")),
            MenuAction("AM 深度 DEPTH", lambda: self._set_family_number("MDWV", "DEPTH", "AM 深度 %")),
            MenuAction("FM/PM/PWM 偏差 DEVI", lambda: self._set_family_number("MDWV", "DEVI", "调制偏差")),
            MenuAction("键控频率 KFRQ", lambda: self._set_family_number("MDWV", "KFRQ", "键控频率 Hz")),
            MenuAction("FSK 跳频频率 HFRQ", lambda: self._set_family_number("MDWV", "HFRQ", "FSK 跳频频率 Hz")),
            MenuAction("载波类型", lambda: self._carrier_choice("MDWV", "WVTP", "载波类型", ("SINE", "SQUARE", "RAMP", "ARB", "PULSE"))),
            MenuAction("载波频率", lambda: self._carrier_number("MDWV", "FRQ", "载波频率 Hz")),
            MenuAction("载波幅度 Vpp", lambda: self._carrier_number("MDWV", "AMP", "载波幅度 Vpp")),
            MenuAction("载波偏置", lambda: self._carrier_number("MDWV", "OFST", "载波偏置 V")),
            MenuAction("载波相位", lambda: self._carrier_number("MDWV", "PHSE", "载波相位 度")),
            MenuAction("载波对称度", lambda: self._carrier_number("MDWV", "SYM", "载波对称度 %")),
            MenuAction("载波占空比", lambda: self._carrier_number("MDWV", "DUTY", "载波占空比 %")),
            MenuAction("载波上升时间", lambda: self._carrier_number("MDWV", "RISE", "载波上升时间 s")),
            MenuAction("载波下降时间", lambda: self._carrier_number("MDWV", "FALL", "载波下降时间 s")),
            MenuAction("载波延迟", lambda: self._carrier_number("MDWV", "DLY", "载波延迟 s")),
        ]

    def _sweep_actions(self) -> list[MenuAction]:
        channel = self.channel
        cache = self.mode_cache["sweep"][channel]

        def toggle_state() -> None:
            enabled = cache.get("STATE", "OFF") != "ON"
            self._safe("扫频开关", lambda: self.instrument.set_sweep(channel, [("STATE", enabled)]))
            self._safe("刷新扫频", self._refresh_current_mode)

        return [
            MenuAction("刷新扫频状态", lambda: self._safe("刷新扫频", self._refresh_current_mode)),
            MenuAction("扫频开关 STATE", toggle_state, cache.get("STATE", "?")),
            MenuAction("扫频时间 TIME", lambda: self._set_family_number("SWWV", "TIME", "扫频时间 s"), cache.get("TIME", "?")),
            MenuAction("起始保持 STARTTIME", lambda: self._set_family_number("SWWV", "STARTTIME", "起始保持 s")),
            MenuAction("结束保持 ENDTIME", lambda: self._set_family_number("SWWV", "ENDTIME", "结束保持 s")),
            MenuAction("返回时间 BACKTIME", lambda: self._set_family_number("SWWV", "BACKTIME", "返回时间 s")),
            MenuAction("起始频率 START", lambda: self._set_family_number("SWWV", "START", "起始频率 Hz"), cache.get("START", "?")),
            MenuAction("终止频率 STOP", lambda: self._set_family_number("SWWV", "STOP", "终止频率 Hz"), cache.get("STOP", "?")),
            MenuAction("中心频率 CENTER", lambda: self._set_family_number("SWWV", "CENTER", "中心频率 Hz"), cache.get("CENTER", "?")),
            MenuAction("频率跨度 SPAN", lambda: self._set_family_number("SWWV", "SPAN", "频率跨度 Hz"), cache.get("SPAN", "?")),
            MenuAction("扫描模式 SWMD", lambda: self._set_family_choice("SWWV", "SWMD", "扫描模式", ("LINE", "LOG", "STEP")), cache.get("SWMD", "?")),
            MenuAction("扫描方向 DIR", lambda: self._set_family_choice("SWWV", "DIR", "扫描方向", ("UP", "DOWN")), cache.get("DIR", "?")),
            MenuAction("触发源 TRSR", lambda: self._set_family_choice("SWWV", "TRSR", "触发源", ("INT", "EXT", "MAN")), cache.get("TRSR", "?")),
            MenuAction("触发输出 TRMD", lambda: self._set_family_choice("SWWV", "TRMD", "触发输出", ("ON", "OFF")), cache.get("TRMD", "?")),
            MenuAction("触发沿 EDGE", lambda: self._set_family_choice("SWWV", "EDGE", "触发沿", ("RISE", "FALL"))),
            MenuAction("步进数量 STEPNUM", lambda: self._set_family_number("SWWV", "STEPNUM", "步进数量 2-64")),
            MenuAction("手动触发 MTRIG", lambda: self._safe("手动扫频触发", lambda: self.instrument.trigger_sweep(channel))),
            MenuAction("载波类型", lambda: self._carrier_choice("SWWV", "WVTP", "扫频载波类型", ("SINE", "SQUARE", "RAMP", "ARB"))),
            MenuAction("载波频率", lambda: self._carrier_number("SWWV", "FRQ", "载波频率 Hz")),
            MenuAction("载波幅度 Vpp", lambda: self._carrier_number("SWWV", "AMP", "载波幅度 Vpp")),
            MenuAction("载波偏置", lambda: self._carrier_number("SWWV", "OFST", "载波偏置 V")),
            MenuAction("载波相位", lambda: self._carrier_number("SWWV", "PHSE", "载波相位 度")),
            MenuAction("载波对称度", lambda: self._carrier_number("SWWV", "SYM", "载波对称度 %")),
            MenuAction("载波占空比", lambda: self._carrier_number("SWWV", "DUTY", "载波占空比 %")),
        ]

    def _burst_actions(self) -> list[MenuAction]:
        channel = self.channel
        cache = self.mode_cache["burst"][channel]

        def toggle_state() -> None:
            enabled = cache.get("STATE", "OFF") != "ON"
            self._safe("Burst 开关", lambda: self.instrument.set_burst(channel, [("STATE", enabled)]))
            self._safe("刷新 Burst", self._refresh_current_mode)

        def set_cycles() -> None:
            value = self.prompt_text("Burst 周期数 TIME (INF 或整数)", cache.get("TIME", "1"))
            if value is not None and value.strip():
                self._safe("设置周期数", lambda: self.instrument.set_burst(channel, [("TIME", value.strip().upper())]))
                self._safe("刷新 Burst", self._refresh_current_mode)

        return [
            MenuAction("刷新 Burst 状态", lambda: self._safe("刷新 Burst", self._refresh_current_mode)),
            MenuAction("Burst 开关 STATE", toggle_state, cache.get("STATE", "?")),
            MenuAction("Burst 周期 PRD", lambda: self._set_family_number("BTWV", "PRD", "Burst 周期 s"), cache.get("PRD", "?")),
            MenuAction("起始相位 STPS", lambda: self._set_family_number("BTWV", "STPS", "起始相位 度"), cache.get("STPS", "?")),
            MenuAction("Burst 模式 GATE/NCYC", lambda: self._set_family_choice("BTWV", "GATE_NCYC", "Burst 模式", ("GATE", "NCYC")), cache.get("GATE_NCYC", "?")),
            MenuAction("触发源 TRSR", lambda: self._set_family_choice("BTWV", "TRSR", "触发源", ("INT", "EXT", "MAN")), cache.get("TRSR", "?")),
            MenuAction("触发延迟 DLAY", lambda: self._set_family_number("BTWV", "DLAY", "触发延迟 s"), cache.get("DLAY", "?")),
            MenuAction("门控极性 PLRT", lambda: self._set_family_choice("BTWV", "PLRT", "门控极性", ("POS", "NEG"))),
            MenuAction("触发输出 TRMD", lambda: self._set_family_choice("BTWV", "TRMD", "触发输出", ("RISE", "FALL", "OFF")), cache.get("TRMD", "?")),
            MenuAction("触发沿 EDGE", lambda: self._set_family_choice("BTWV", "EDGE", "触发沿", ("RISE", "FALL"))),
            MenuAction("周期数 TIME", set_cycles, cache.get("TIME", "?")),
            MenuAction("Burst 次数 COUNT", lambda: self._set_family_number("BTWV", "COUNT", "Burst 次数")),
            MenuAction("手动触发 MTRIG", lambda: self._safe("手动 Burst 触发", lambda: self.instrument.trigger_burst(channel))),
            MenuAction("载波类型", lambda: self._carrier_choice("BTWV", "WVTP", "Burst 载波类型", ("SINE", "SQUARE", "RAMP", "ARB", "PULSE", "NOISE"))),
            MenuAction("载波频率", lambda: self._carrier_number("BTWV", "FRQ", "载波频率 Hz")),
            MenuAction("载波幅度 Vpp", lambda: self._carrier_number("BTWV", "AMP", "载波幅度 Vpp")),
            MenuAction("载波偏置", lambda: self._carrier_number("BTWV", "OFST", "载波偏置 V")),
            MenuAction("载波相位", lambda: self._carrier_number("BTWV", "PHSE", "载波相位 度")),
            MenuAction("载波对称度", lambda: self._carrier_number("BTWV", "SYM", "载波对称度 %")),
            MenuAction("载波占空比", lambda: self._carrier_number("BTWV", "DUTY", "载波占空比 %")),
            MenuAction("载波上升时间", lambda: self._carrier_number("BTWV", "RISE", "载波上升时间 s")),
            MenuAction("载波下降时间", lambda: self._carrier_number("BTWV", "FALL", "载波下降时间 s")),
            MenuAction("载波延迟", lambda: self._carrier_number("BTWV", "DLY", "载波延迟 s")),
            MenuAction("噪声标准差", lambda: self._carrier_number("BTWV", "STDEV", "噪声标准差 V")),
            MenuAction("噪声均值", lambda: self._carrier_number("BTWV", "MEAN", "噪声均值 V")),
        ]

    def _arbitrary_actions(self) -> list[MenuAction]:
        channel = self.channel

        def show_query(title: str, callback: Callable[[], str]) -> None:
            response = self._safe(title, callback)
            if response is not None:
                self.show_text(title, str(response))

        def select_index() -> None:
            value = self.prompt_text("内建任意波索引")
            if value is not None:
                try:
                    index = int(value)
                except ValueError:
                    self._log("索引必须是整数", error=True)
                    return
                self._safe("选择任意波", lambda: self.instrument.select_arbitrary(channel, index=index), refresh=True)

        def select_name() -> None:
            self._text_then(
                "任意波名称",
                lambda value: self.instrument.select_arbitrary(channel, name=value),
            )

        def select_path() -> None:
            self._text_then(
                "仪器内部/网络/U盘波形路径",
                lambda value: self.instrument.select_arbitrary(channel, path=value),
            )

        def marker_toggle() -> None:
            state = self.prompt_choice("任意波 Marker", ("OFF", "ON"), "OFF")
            if state is not None:
                self._safe("Marker 设置", lambda: self.instrument.set_arbitrary_marker(channel, state == "ON"))

        def upload_csv() -> None:
            path = self.prompt_text("本机 CSV 文件绝对路径")
            if not path:
                return
            name = self.prompt_text("保存到仪器的波形名", Path(path).stem)
            if not name:
                return
            mode = self.prompt_choice("CSV 数据格式", ("归一化 -1..1", "有符号16位整数"), "归一化 -1..1")
            if mode is None:
                return
            if not self.confirm(f"上传 {path} 到 CH{channel}，波形名 {name}？"):
                return
            self._safe(
                "上传 CSV 任意波",
                lambda: self.instrument.upload_waveform_csv(
                    channel, name, path, normalized=mode.startswith("归一化")
                ),
            )

        def upload_hex() -> None:
            name = self.prompt_text("波形名", "wave1")
            if not name:
                return
            data = self.prompt_text("16位样本十六进制数据，例如 6000c0006000")
            if data:
                self._safe("上传十六进制任意波", lambda: self.instrument.upload_waveform_hex(channel, name, data))

        def query_wave_data() -> None:
            name = self.prompt_text("用户波形名")
            if not name:
                return
            path = self.prompt_text("仪器路径（留空表示本地）", "")
            response = self._safe("读取波形数据", lambda: self.instrument.waveform_data(name, path or None))
            if response is not None:
                self.show_text("波形数据", str(response))

        actions = [
            MenuAction("查询当前任意波", lambda: show_query("当前任意波", lambda: self.instrument.arbitrary_status(channel))),
            MenuAction("按索引选择内建任意波", select_index),
            MenuAction("按名称选择任意波", select_name),
            MenuAction("按仪器路径选择任意波", select_path),
            MenuAction("列出全部波形", lambda: show_query("全部波形", self.instrument.list_waveforms)),
            MenuAction("列出内建波形", lambda: show_query("内建波形", lambda: self.instrument.list_waveforms("BUILDIN"))),
            MenuAction("列出用户波形", lambda: show_query("用户波形", lambda: self.instrument.list_waveforms("USER"))),
            MenuAction("任意波 Marker 开关", marker_toggle),
            MenuAction("上传 CSV 任意波", upload_csv, dangerous=True),
            MenuAction("上传十六进制任意波", upload_hex, dangerous=True),
            MenuAction("读取用户任意波数据", query_wave_data),
        ]
        if not self.instrument.model_profile.supports("arbitrary_marker"):
            actions = [
                action for action in actions if action.label != "任意波 Marker 开关"
            ]
        return actions

    def _advanced_actions(self) -> list[MenuAction]:
        channel = self.channel

        def coupling_choice(parameter: str, title: str, choices: Sequence[str]) -> None:
            self._choose_then(title, choices, lambda value: self.instrument.set_coupling([(parameter, value)]), refresh=False)

        def coupling_number(parameter: str, title: str) -> None:
            self._number_then(title, lambda value: self.instrument.set_coupling([(parameter, value)]), refresh=False)

        def harmonic_choice(parameter: str, title: str, choices: Sequence[str]) -> None:
            self._choose_then(title, choices, lambda value: self.instrument.set_harmonic(channel, [(parameter, value)]), refresh=False)

        def harmonic_number(parameter: str, title: str) -> None:
            self._number_then(title, lambda value: self.instrument.set_harmonic(channel, [(parameter, value)]), refresh=False)

        def set_cascade() -> None:
            state = self.prompt_choice("多机同步状态", ("OFF", "ON"), "OFF")
            if state is None:
                return
            mode = self.prompt_choice("多机同步角色", ("MASTER", "SLAVE"), "MASTER")
            if mode is None:
                return
            delay = None
            if mode == "SLAVE":
                delay = self.prompt_number("从机延迟 s", "0")
                if delay is None:
                    return
            self._safe("设置多机同步", lambda: self.instrument.set_cascade(state == "ON", mode=mode, delay_s=delay))

        actions = [
            MenuAction("查询 SYNC", lambda: self._safe("SYNC", lambda: self.instrument.sync_status(channel))),
            MenuAction("切换 SYNC", lambda: self._toggle_channel_query("SYNC", lambda: self.instrument.sync_status(channel), lambda enabled: self.instrument.set_sync(channel, enabled))),
            MenuAction("两通道同相位 EQPHASE", lambda: self._safe("同相位", self.instrument.equal_phase)),
            MenuAction("复制另一通道到当前通道", lambda: self._safe("复制通道", lambda: self.instrument.copy_channel(2 if channel == 1 else 1, channel))),
            MenuAction("查询耦合参数", lambda: self._safe("耦合参数", self.instrument.coupling_status)),
            MenuAction("通道跟踪 TRACE", lambda: coupling_choice("TRACE", "通道跟踪", ("ON", "OFF"))),
            MenuAction("频率耦合 FCOUP", lambda: coupling_choice("FCOUP", "频率耦合", ("ON", "OFF"))),
            MenuAction("频率偏差 FDEV", lambda: coupling_number("FDEV", "频率偏差 Hz")),
            MenuAction("频率比例 FRAT", lambda: coupling_number("FRAT", "频率比例")),
            MenuAction("相位耦合 PCOUP", lambda: coupling_choice("PCOUP", "相位耦合", ("ON", "OFF"))),
            MenuAction("相位偏差 PDEV", lambda: coupling_number("PDEV", "相位偏差 度")),
            MenuAction("相位比例 PRAT", lambda: coupling_number("PRAT", "相位比例")),
            MenuAction("幅度耦合 ACOUP", lambda: coupling_choice("ACOUP", "幅度耦合", ("ON", "OFF"))),
            MenuAction("幅度比例 ARAT", lambda: coupling_number("ARAT", "幅度比例")),
            MenuAction("幅度偏差 ADEV", lambda: coupling_number("ADEV", "幅度偏差 Vpp")),
            MenuAction("相位模式", lambda: self._choose_then("相位模式", ("PHASELOCKED", "INDEPENDENT"), self.instrument.set_phase_mode, refresh=False)),
            MenuAction("查询谐波参数", lambda: self._safe("谐波参数", lambda: self.instrument.harmonic_status(channel))),
            MenuAction("谐波开关 HARMSTATE", lambda: harmonic_choice("HARMSTATE", "谐波开关", ("ON", "OFF"))),
            MenuAction("谐波类型 HARMTYPE", lambda: harmonic_choice("HARMTYPE", "谐波类型", ("EVEN", "ODD", "ALL"))),
            MenuAction("谐波级数 HARMORDER", lambda: harmonic_number("HARMORDER", "谐波级数")),
            MenuAction("谐波幅度 HARMAMP", lambda: harmonic_number("HARMAMP", "谐波幅度 Vpp")),
            MenuAction("谐波幅度 HARMDBC", lambda: harmonic_number("HARMDBC", "谐波幅度 dBc")),
            MenuAction("谐波相位 HARMPHASE", lambda: harmonic_number("HARMPHASE", "谐波相位 度")),
            MenuAction("波形合并开关", lambda: self._toggle_channel_query("波形合并", lambda: self.instrument.combine_status(channel), lambda enabled: self.instrument.set_combine(channel, enabled))),
            MenuAction("查询多机同步", lambda: self._safe("多机同步", self.instrument.cascade_status)),
            MenuAction("设置多机同步", set_cascade),
        ]
        if not self.instrument.model_profile.supports("cascade"):
            actions = [
                action
                for action in actions
                if action.label not in {"查询多机同步", "设置多机同步"}
            ]
        return actions

    def _system_actions(self) -> list[MenuAction]:
        channel = self.channel

        def toggle_query_set(title: str, query: Callable[[], str], setter: Callable[[bool], Any]) -> None:
            response = self._safe(title + "状态", query)
            if response is not None:
                self._safe(title, lambda: setter("ON" not in str(response).upper()))

        def noise_add() -> None:
            state = self.prompt_choice("噪声叠加", ("OFF", "ON"), "OFF")
            if state is None:
                return
            mode = self.prompt_choice("信噪比单位", ("RATIO", "RATIO_DB"), "RATIO")
            if mode is None:
                return
            value = self.prompt_number("信噪比数值")
            if value is None:
                return
            kwargs = {"ratio": value} if mode == "RATIO" else {"ratio_db": value}
            self._safe("噪声叠加", lambda: self.instrument.set_noise_add(channel, state == "ON", **kwargs))

        def set_counter_value(parameter: str, title: str) -> None:
            value = self.prompt_number(title)
            if value is not None:
                self._safe(title, lambda: self.instrument.set_frequency_counter([(parameter, value)]))

        def set_counter_choice(parameter: str, title: str, choices: Sequence[str]) -> None:
            value = self.prompt_choice(title, choices)
            if value is not None:
                self._safe(title, lambda: self.instrument.set_frequency_counter([(parameter, value)]))

        def set_screen_saver() -> None:
            value = self.prompt_choice("屏幕保护分钟", ("OFF", "1", "5", "15", "30", "60", "120", "300"), "OFF")
            if value is not None:
                self._safe("屏幕保护", lambda: self.instrument.set_screen_saver(None if value == "OFF" else int(value)))

        def set_clock_output() -> None:
            value = self.prompt_choice("10 MHz 时钟输出", ("OFF", "ON"), "OFF")
            if value is not None:
                self._safe("10 MHz 输出", lambda: self.instrument.set_global_parameters("ROSC", [("10MOUT", value)]))

        def show_network() -> None:
            status = self._safe("网络状态", self.instrument.network_status)
            if status is not None:
                self.show_text("网络状态", "\n".join(f"{key}: {value}" for key, value in status.items()))

        def set_network_value(kind: str, label: str) -> None:
            value = self.prompt_text(label)
            if not value:
                return
            if not self.confirm(f"修改 {label} 为 {value}？连接可能立即中断"):
                return
            kwargs = {kind: value}
            self._safe(label, lambda: self.instrument.set_network(**kwargs))
            if kind == "ip":
                self.host = value
                if self.requested_host is not None:
                    self.requested_host = value
                self.instrument.host = value
                save_last_device(
                    DiscoveredInstrument(value, self.port, self.identity), self.cache_path
                )

        def set_date() -> None:
            value = self.prompt_text("仪器日期 YYYY/MM/DD", time.strftime("%Y/%m/%d"))
            if value:
                self._safe("设置日期", lambda: self.instrument.set_date(value))

        def set_time_value() -> None:
            value = self.prompt_text("仪器时间 HH:MM:SS", time.strftime("%H:%M:%S"))
            if value:
                self._safe("设置时间", lambda: self.instrument.set_time(value))

        def virtual_key() -> None:
            value = self.prompt_text("虚拟按键名称或索引")
            if value:
                self._safe("虚拟按键按下", lambda: self.instrument.virtual_key(value, True))
                self._safe("虚拟按键释放", lambda: self.instrument.virtual_key(value, False))

        def reset_device() -> None:
            if self.confirm("执行 *RST 恢复默认设置？"):
                self._safe("恢复默认设置", self.instrument.reset)

        def unsupported() -> None:
            profile = self.instrument.model_profile
            feature_names = {
                "noise_add": "噪声叠加 NOISE_ADD",
                "arbitrary_marker": "任意波 Marker MSW",
                "sample_rate": "远程采样率 SRATE",
                "cascade": "多机同步 CASCADE",
                "manual_trigger_coupling": "手动触发双通道耦合 TRDUCH",
                "power_on_mode": "上电开机模式 POWER:ON:MODE",
                "front_panel_keys": "前面板按键开关 KEY",
            }
            unavailable = [
                label
                for feature, label in feature_names.items()
                if not profile.supports(feature)
            ]
            limits = [
                f"{waveform}: {format_frequency_limit(limit)}"
                for waveform, limit in profile.frequency_limits_hz
            ]
            self.show_text(
                f"{profile.model} 型号能力边界",
                f"档案: {profile.summary()}\n\n"
                + ("频率上限:\n- " + "\n- ".join(limits) + "\n\n" if limits else "")
                + (
                    "TUI/API 已停用的命令族:\n- "
                    + "\n- ".join(unavailable)
                    + "\n\n"
                    if unavailable
                    else ""
                )
                + "序列波形、16 路数字通道、FHOP、SDG7000A 计数器统计、"
                "文件管理及 IQ/数字接口等平台专属功能也不会由 TUI 发送。\n\n"
                "原始 SCPI 控制台不会绕过仪器自身校验，实验性命令请以目标固件手册为准。",
            )

        actions = [
            MenuAction("噪声叠加 NOISE_ADD", noise_add),
            MenuAction("查询频率计 FCNT", lambda: self._safe("频率计", self.instrument.frequency_counter_status)),
            MenuAction("频率计开关", lambda: set_counter_choice("STATE", "频率计开关", ("ON", "OFF"))),
            MenuAction("频率计参考频率 REFQ", lambda: set_counter_value("REFQ", "参考频率 Hz")),
            MenuAction("频率计触发电平 TRG", lambda: set_counter_value("TRG", "触发电平 V")),
            MenuAction("频率计耦合 MODE", lambda: set_counter_choice("MODE", "频率计耦合", ("AC", "DC"))),
            MenuAction("频率计高频抑制 HFR", lambda: set_counter_choice("HFR", "高频抑制", ("ON", "OFF"))),
            MenuAction("过压保护", lambda: toggle_query_set("过压保护", self.instrument.overvoltage_protection, self.instrument.set_overvoltage_protection)),
            MenuAction("蜂鸣器", lambda: toggle_query_set("蜂鸣器", self.instrument.buzzer_status, self.instrument.set_buzzer)),
            MenuAction("屏幕保护", set_screen_saver),
            MenuAction("参考时钟源", lambda: self._choose_then("参考时钟", ("INT", "EXT"), self.instrument.set_clock_source, refresh=False)),
            MenuAction("10 MHz 时钟输出", set_clock_output),
            MenuAction("数字格式", lambda: self._choose_then("小数点格式", ("DOT", "COMMA"), lambda decimal: self.instrument.set_number_format(decimal, "SPACE"), refresh=False)),
            MenuAction("系统语言", lambda: self._choose_then("系统语言", self.instrument.model_profile.languages, self.instrument.set_language, refresh=False)),
            MenuAction("启动配置", lambda: self._choose_then("启动配置", ("DEFAULT", "LAST", "USER"), self.instrument.set_startup_config, refresh=False)),
            MenuAction("直接上电开机模式", lambda: self._choose_then("开机模式", ("直接上电", "按键开机"), lambda value: self.instrument.set_power_on_mode(value == "直接上电"), refresh=False)),
            MenuAction("前面板按键开关", lambda: self._choose_then("前面板按键", ("ON", "OFF"), lambda value: self.instrument.set_front_panel_keys(value == "ON"), refresh=False)),
            MenuAction("设置日期", set_date),
            MenuAction("设置时间", set_time_value),
            MenuAction("虚拟按键", virtual_key),
            MenuAction("查看网络配置", show_network),
            MenuAction("修改 IP 地址", lambda: set_network_value("ip", "IP 地址"), dangerous=True),
            MenuAction("修改子网掩码", lambda: set_network_value("mask", "子网掩码"), dangerous=True),
            MenuAction("修改网关", lambda: set_network_value("gateway", "网关"), dangerous=True),
            MenuAction("两通道同时开启输出", lambda: self._safe("两通道输出 ON", lambda: self.instrument.set_all_outputs(True), refresh=True), dangerous=True),
            MenuAction("两通道同时关闭输出", lambda: self._safe("两通道输出 OFF", lambda: self.instrument.set_all_outputs(False), refresh=True)),
            MenuAction("恢复默认设置 (*RST)", reset_device, dangerous=True),
            MenuAction("查看当前型号能力边界", unsupported),
        ]
        hidden_labels: set[str] = set()
        if not self.instrument.model_profile.supports("noise_add"):
            hidden_labels.add("噪声叠加 NOISE_ADD")
        if not self.instrument.model_profile.supports("power_on_mode"):
            hidden_labels.add("直接上电开机模式")
        if not self.instrument.model_profile.supports("front_panel_keys"):
            hidden_labels.add("前面板按键开关")
        return [action for action in actions if action.label not in hidden_labels]

    def _console_actions(self) -> list[MenuAction]:
        def send_command() -> None:
            command = self.prompt_text("SCPI 命令")
            if not command:
                return
            command = command.strip()
            self._history.append(command)
            self._history = self._history[-100:]
            if command.endswith("?") or "? " in command:
                response = self._safe(
                    "SCPI 查询", lambda: self.instrument.query(command), allow_generic=True
                )
                if response is not None:
                    self.show_text(command, str(response))
            else:
                if command.upper().startswith("*RST") and not self.confirm("发送 *RST？"):
                    return
                self._safe(
                    "SCPI 写入",
                    lambda: (self.instrument.write(command), self.instrument.operation_complete())[1],
                    allow_generic=True,
                )

        def history() -> None:
            self.show_text("SCPI 历史", "\n".join(self._history) if self._history else "暂无历史")

        def common_queries() -> None:
            queries = (
                ("*IDN?", "*OPC?", "SYST:ERR?")
                if not self.sdg_compatible
                else ("*IDN?", "C1:BSWV?", "C2:BSWV?", "C1:OUTP?", "C2:OUTP?", "C1:MDWV?", "C1:SWWV?", "C1:BTWV?", "COUP?", "FCNT?", "SYST:ERR?")
            )
            command = self.prompt_choice(
                "常用查询",
                queries,
            )
            if command:
                response = self._safe(
                    command, lambda: self.instrument.query(command), allow_generic=True
                )
                if response is not None:
                    self.show_text(command, str(response))

        return [
            MenuAction("输入并发送 SCPI 命令", send_command),
            MenuAction("常用查询菜单", common_queries),
            MenuAction("查看命令历史", history),
            MenuAction(
                "读取系统错误",
                lambda: self._safe(
                    "系统错误", self.instrument.system_error, allow_generic=True
                ),
            ),
            MenuAction(
                "刷新设备状态",
                lambda: self._safe(
                    "刷新", self._refresh_device_status, allow_generic=True
                ),
            ),
        ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Discover SCPI instruments and control profiled SIGLENT SDG generators "
            "including SDG1032X and SDG2122X over TCP."
        )
    )
    parser.add_argument(
        "--host",
        default=None,
        help="Instrument IP/hostname; otherwise try the last device and auto-discovery",
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="SCPI TCP port")
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT, help="Socket timeout in seconds"
    )
    parser.add_argument(
        "--subnet",
        action="append",
        default=[],
        help="IPv4 discovery network such as 192.0.2.0/24; may be repeated",
    )
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=DEFAULT_SCAN_TIMEOUT,
        help="Per-address discovery timeout in seconds",
    )
    parser.add_argument(
        "--no-discovery",
        action="store_true",
        help="Do not scan when explicit/last/default endpoints are unavailable",
    )

    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("tui", help="Open the full-screen terminal control interface")
    subparsers.add_parser("catalog", help="List all terminal control sections and actions")
    discover_parser = subparsers.add_parser(
        "discover",
        help="Scan, list all TCP SCPI instruments, and manually select one",
    )
    discover_parser.add_argument(
        "--json",
        action="store_true",
        dest="json_output",
        help="Print all results as JSON; manual device selection is still required",
    )
    subparsers.add_parser("idn", help="Read the instrument identity")

    status_parser = subparsers.add_parser("status", help="Read waveform and output status")
    status_parser.add_argument(
        "--channel", choices=("1", "2", "all"), default="all"
    )

    sine_parser = subparsers.add_parser("sine", help="Configure a sine wave")
    sine_parser.add_argument("--channel", type=int, choices=(1, 2), default=1)
    sine_parser.add_argument(
        "--frequency",
        type=parse_engineering_number,
        required=True,
        help="Frequency in Hz; suffixes such as 10k and 1M are accepted",
    )
    sine_parser.add_argument(
        "--amplitude",
        type=parse_engineering_number,
        required=True,
        help="Amplitude in Vpp",
    )
    sine_parser.add_argument(
        "--offset", type=parse_engineering_number, default=0.0, help="DC offset in V"
    )
    sine_parser.add_argument(
        "--phase", type=parse_engineering_number, default=0.0, help="Phase in degrees"
    )
    sine_parser.add_argument(
        "--enable", action="store_true", help="Enable the channel after configuration"
    )

    output_parser = subparsers.add_parser("output", help="Turn a channel on or off")
    output_parser.add_argument("--channel", type=int, choices=(1, 2), required=True)
    output_parser.add_argument("state", choices=("on", "off"))

    subparsers.add_parser("error", help="Read one entry from the system error queue")

    raw_parser = subparsers.add_parser("raw", help="Send a raw SCPI command")
    raw_parser.add_argument("command", help="SCPI command; commands ending in ? are queried")

    return parser


def run(args: argparse.Namespace) -> int:
    if args.action == "tui":
        return SDGTerminalApp(
            args.host,
            args.port,
            args.timeout,
            discovery_subnets=args.subnet,
            scan_timeout=args.scan_timeout,
            auto_discover=not args.no_discovery,
        ).run()
    if args.action == "catalog":
        app = SDGTerminalApp(
            args.host,
            args.port,
            args.timeout,
            discovery_subnets=args.subnet,
            scan_timeout=args.scan_timeout,
            auto_discover=not args.no_discovery,
        )
        for index, tab in enumerate(app.TABS):
            app.tab_index = index
            print(f"[{tab}]")
            for action in app._actions():
                print(f"  - {action.label}")
        return 0

    if args.action == "discover":
        last = load_last_device()
        preferred_hosts = (args.host, last.host if last else None, DEFAULT_HOST)
        ports = {args.port}
        if last is not None:
            ports.add(last.port)
        networks = discovery_networks(args.subnet, preferred_hosts=preferred_hosts)
        candidates = discover_instruments(
            [str(network) for network in networks],
            ports=tuple(ports),
            timeout=args.scan_timeout,
            preferred_hosts=preferred_hosts,
            progress_callback=make_console_progress_reporter(),
        )
        if args.json_output:
            print(
                json.dumps(
                    [
                        {
                            "host": candidate.host,
                            "port": candidate.port,
                            "identity": candidate.identity,
                            "manufacturer": candidate.manufacturer,
                            "model": candidate.model,
                            "serial": candidate.serial,
                            "siglent_sdg_compatible": is_siglent_sdg_identity(candidate.identity),
                            "sdg_family": (
                                get_sdg_model_profile(candidate.identity).family
                                if get_sdg_model_profile(candidate.identity) is not None
                                else None
                            ),
                            "max_sine_frequency_hz": (
                                get_sdg_model_profile(candidate.identity).frequency_limit("SINE")
                                if get_sdg_model_profile(candidate.identity) is not None
                                else None
                            ),
                        }
                        for candidate in candidates
                    ],
                    ensure_ascii=False,
                    indent=2,
                )
            )
        else:
            print("Scan networks: " + (", ".join(map(str, networks)) or "none"))
            if not candidates:
                print("No SCPI instruments found.")
            for index, candidate in enumerate(candidates, 1):
                kind = "SIGLENT SDG" if is_siglent_sdg_identity(candidate.identity) else "Generic SCPI"
                print(
                    f"{index:02d}. {candidate.host}:{candidate.port} "
                    f"[{kind}] {candidate.identity}"
                )
        if candidates:
            selection_output = sys.stderr if args.json_output else sys.stdout
            selected = choose_console_device(
                candidates,
                show_list=args.json_output,
                output=selection_output,
            )
            if selected is None:
                print(
                    "扫描结果必须在交互终端中手动选择；未保存任何设备。",
                    file=sys.stderr,
                )
                return 2
            save_last_device(selected)
            print(f"Selected: {selected.display_name()}", file=selection_output)
        return 0

    progress_reporter = make_console_progress_reporter()
    location = locate_instrument(
        args.host,
        args.port,
        args.timeout,
        subnets=args.subnet,
        scan_timeout=args.scan_timeout,
        allow_scan=not args.no_discovery,
        progress_callback=progress_reporter,
    )
    selected = location.selected
    if selected is None:
        if location.source == "explicit-not-found":
            raise InstrumentError(
                f"No SCPI instrument answered *IDN? at {args.host}:{args.port}; "
                "explicit --host never falls back to another device"
            )
        if location.candidates:
            selected = choose_console_device(location.candidates)
            if selected is None:
                raise InstrumentError(
                    "A scan was performed, but no device was manually selected"
                )
        else:
            networks = ", ".join(location.networks) or "none"
            raise InstrumentError(f"No SCPI instrument found; scan networks: {networks}")

    assert selected is not None
    with SDG2122X(selected.host, selected.port, args.timeout) as instrument:
        identity = instrument.identify()
        selected = DiscoveredInstrument(selected.host, selected.port, identity)
        save_last_device(selected)
        if args.action in {"status", "sine", "output"} and not is_siglent_sdg_identity(identity):
            raise InstrumentError(
                f"{identity!r} is not a recognized SIGLENT SDG generator; "
                "use idn/raw or choose another device with --host"
            )

        if args.action == "idn":
            print(identity)

        elif args.action == "status":
            channels = (1, 2) if args.channel == "all" else (int(args.channel),)
            print(f"Device: {identity}")
            print(f"Profile: {instrument.model_profile.summary()}")
            for channel in channels:
                waveform, output = instrument.channel_status(channel)
                print(f"CH{channel} waveform: {waveform}")
                print(f"CH{channel} output:   {output}")

        elif args.action == "sine":
            configuration = SineConfiguration(
                channel=args.channel,
                frequency_hz=args.frequency,
                amplitude_vpp=args.amplitude,
                offset_v=args.offset,
                phase_deg=args.phase,
            )
            waveform = instrument.set_sine(configuration)
            print(f"CH{args.channel} waveform: {waveform}")
            if args.enable:
                output = instrument.set_output(args.channel, True)
                print(f"CH{args.channel} output:   {output}")
            else:
                print(
                    f"CH{args.channel} output was not changed. "
                    "Use --enable to turn it on."
                )

        elif args.action == "output":
            output = instrument.set_output(args.channel, args.state == "on")
            print(output)

        elif args.action == "error":
            print(instrument.system_error())

        elif args.action == "raw":
            command = args.command.strip()
            if command.endswith("?"):
                print(instrument.query(command))
            else:
                instrument.write(command)
                if instrument.operation_complete():
                    print("OK")

    return 0


def main() -> int:
    with contextlib.suppress(Exception):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    with contextlib.suppress(Exception):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    if len(sys.argv) == 1:
        try:
            return SDGTerminalApp(None, DEFAULT_PORT, DEFAULT_TIMEOUT).run()
        except KeyboardInterrupt:
            return 130
    parser = build_parser()
    args = parser.parse_args()
    try:
        return run(args)
    except (InstrumentError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
