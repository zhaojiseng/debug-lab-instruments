import contextlib
import io
import socketserver
import struct
import tempfile
import threading
import unittest
from pathlib import Path

from dho924_control import (
    ACQUISITION_SPECS,
    DHO924TerminalApp,
    DiscoveredScope,
    DiscoveryProgress,
    MEASUREMENT_ITEMS,
    MHO_ACQUISITION_SPECS,
    RigolDHO,
    ScopeError,
    TerminalEvent,
    TerminalSession,
    TIMEBASE_SPECS,
    TRIGGER_SPECS,
    WaveformPreamble,
    build_parser,
    choose_console_scope,
    discover_scopes,
    identity_key,
    invalid_measurement,
    is_mho_scope_identity,
    is_supported_scope_identity,
    load_last_scope,
    locate_scope,
    main,
    parse_engineering_number,
    save_last_scope,
    scope_family_from_identity,
    waveform_ascii_preview,
)


class FakeDHOHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while True:
            raw = self.rfile.readline()
            if not raw:
                return
            command = raw.decode("ascii").strip()
            self.server.commands.append(command)
            upper = command.upper()
            if upper.startswith(":WAVEFORM:DATA?"):
                self.send_block(self.server.waveform_payload)
                continue
            if upper.startswith(":DISPLAY:DATA?") or upper == ":SAVE:IMAGE:DATA?":
                self.send_block(self.server.screenshot_payload)
                continue
            if "?" in command:
                response = self.response_for(upper)
                self.wfile.write(response.encode("ascii") + b"\n")
                self.wfile.flush()
            else:
                self.apply_write(command, upper)

    def send_block(self, payload):
        length = str(len(payload)).encode("ascii")
        self.wfile.write(b"#" + str(len(length)).encode("ascii") + length + payload + b"\n")
        self.wfile.flush()

    def response_for(self, upper):
        if upper == "*IDN?":
            return self.server.identity
        if upper == "*OPC?":
            return "0"
        if upper == ":SYSTEM:ERROR?":
            if self.server.errors:
                return self.server.errors.pop(0)
            return '0,"No error"'
        if upper.startswith(":MEASURE:ITEM?"):
            return "9.9000E+37" if "CHAN1" in upper else "6.1867E-04"
        if upper == ":WAVEFORM:PREAMBLE?":
            return self.server.preamble
        if upper == ":COUNTER:CURRENT?":
            return "1.000000E+06"
        if upper == ":DVM:CURRENT?":
            return "1.250000E+00"
        if upper == ":BUS1:DATA?":
            return "AA,55"
        if upper == ":SEARCH:COUNT?":
            return "2"
        header, _, arguments = upper.partition("?")
        if arguments.strip():
            header = header.strip()
        return self.server.state.get(header, "0")

    def apply_write(self, command, upper):
        if upper == ":RUN":
            self.server.state[":TRIGGER:STATUS"] = "RUN"
            return
        if upper == ":STOP":
            self.server.state[":TRIGGER:STATUS"] = "STOP"
            return
        if upper == ":SINGLE":
            self.server.state[":TRIGGER:STATUS"] = "WAIT"
            return
        if upper.startswith(":WAVEFORM:SOURCE "):
            source = upper.split(" ", 1)[1].strip()
            if source.startswith("CHANNEL") and source[7:].isdigit():
                source = "CHAN" + source[7:]
            if source.startswith("CHAN") and source[4:].isdigit():
                display = self.server.state.get(
                    f":CHANNEL{int(source[4:])}:DISPLAY", "0"
                )
                if display == "0":
                    self.server.errors.append('-200,"Command execute failed"')
                    return
        if " " in command:
            header, value = command.split(" ", 1)
            self.server.state[header.upper()] = value.strip().strip('"')


class FakeDHOServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    block_on_close = False

    def __init__(self, identity="RIGOL TECHNOLOGIES,DHO924,TEST-DHO-0001,00.01.14"):
        super().__init__(("127.0.0.1", 0), FakeDHOHandler)
        self.identity = identity
        self.commands = []
        self.errors = []
        self.state = self.default_state()
        samples = [32768 + 2074 + int(180 * ((index % 50) / 49 - 0.5)) for index in range(1000)]
        self.waveform_payload = struct.pack("<1000H", *samples)
        self.preamble = "1,0,1000,1,1.000000E-10,-5.000000E-8,0,1.3333E-4,2074,32768"
        self.screenshot_payload = b"\x89PNG\r\n\x1a\nFAKE_DHO_SCREENSHOT"
        self.thread = threading.Thread(target=self.serve_forever, daemon=True)

    @staticmethod
    def default_state():
        state = {
            ":SYSTEM:VERSION": "3.0",
            ":SYSTEM:ERROR": '0,"No error"',
            ":SYSTEM:MODULES": "DHO924",
            ":SYSTEM:RAMOUNT": "1073741824",
            ":SYSTEM:GAMOUNT": "268435456",
            ":ACQUIRE:TYPE": "NORM",
            ":ACQUIRE:SRATE": "6.250000E+8",
            ":ACQUIRE:MDEPTH": "1.0000E+04",
            ":ACQUIRE:AVERAGES": "2",
            ":ACQUIRE:BITS": "14",
            ":ACQUIRE:ULTRA:MODE": "ADJ",
            ":ACQUIRE:ULTRA:TIMEOUT": "1.0E-1",
            ":ACQUIRE:ULTRA:MAXFRAME": "100",
            ":TIMEBASE:MAIN:SCALE": "1.000000E-8",
            ":TIMEBASE:MAIN:OFFSET": "0",
            ":TIMEBASE:MODE": "MAIN",
            ":TIMEBASE:HREFERENCE:MODE": "CENT",
            ":TIMEBASE:HREFERENCE:POSITION": "0",
            ":TIMEBASE:VERNIER": "0",
            ":TIMEBASE:ROLL": "0",
            ":TIMEBASE:DELAY:ENABLE": "0",
            ":TIMEBASE:DELAY:SCALE": "1.0E-9",
            ":TIMEBASE:DELAY:OFFSET": "0",
            ":TRIGGER:STATUS": "STOP",
            ":TRIGGER:MODE": "EDGE",
            ":TRIGGER:SWEEP": "AUTO",
            ":TRIGGER:COUPLING": "DC",
            ":TRIGGER:HOLDOFF": "1.6E-8",
            ":TRIGGER:NREJECT": "0",
            ":TRIGGER:EDGE:SOURCE": "CHAN1",
            ":TRIGGER:EDGE:SLOPE": "POS",
            ":TRIGGER:EDGE:LEVEL": "0.5",
            ":COUNTER:ENABLE": "0",
            ":COUNTER:SOURCE": "CHAN1",
            ":COUNTER:MODE": "FREQ",
            ":COUNTER:NDIGITS": "6",
            ":DVM:ENABLE": "0",
            ":DVM:SOURCE": "CHAN1",
            ":DVM:MODE": "DC",
            ":LA:ENABLE": "0",
            ":LA:ACTIVE": "D0",
            ":LA:SIZE": "MED",
            ":LA:POD1:DISPLAY": "0",
            ":LA:POD1:THRESHOLD": "1.4",
            ":LA:POD2:DISPLAY": "0",
            ":LA:POD2:THRESHOLD": "1.4",
            ":DISPLAY:TYPE": "VECT",
            ":DISPLAY:WBRIGHTNESS": "60",
            ":DISPLAY:GRID": "FULL",
            ":DISPLAY:GBRIGHTNESS": "40",
            ":DISPLAY:RULERS": "1",
            ":DISPLAY:COLOR": "0",
            ":SYSTEM:OPTION:STATUS": "1",
            ":WAVEFORM:SOURCE": "CHAN1",
            ":WAVEFORM:MODE": "NORM",
            ":WAVEFORM:FORMAT": "WORD",
            ":WAVEFORM:POINTS": "1000",
            ":WAVEFORM:START": "1",
            ":WAVEFORM:STOP": "1000",
        }
        for channel in range(1, 5):
            prefix = f":CHANNEL{channel}"
            state.update(
                {
                    prefix + ":DISPLAY": "1" if channel in {1, 3} else "0",
                    prefix + ":SCALE": "1.0" if channel == 1 else "0.02",
                    prefix + ":OFFSET": "0",
                    prefix + ":COUPLING": "DC",
                    prefix + ":PROBE": "10" if channel == 1 else "1",
                    prefix + ":BWLIMIT": "OFF",
                    prefix + ":INVERT": "0",
                    prefix + ":UNITS": "VOLT",
                    prefix + ":VERNIER": "0",
                    prefix + ":POSITION": "0",
                    prefix + ":TCALIBRATE": "0",
                    prefix + ":LABEL:SHOW": "0",
                    prefix + ":LABEL:CONTENT": f"CH{channel}",
                }
            )
        for channel in range(1, 3):
            prefix = f":SOURCE{channel}"
            state.update(
                {
                    prefix + ":OUTPUT:STATE": "0",
                    prefix + ":FUNCTION": "SIN",
                    prefix + ":LOAD:ARBITRARY": "C:/default.csv",
                    prefix + ":FREQUENCY": "1.000000E+3",
                    prefix + ":PERIOD": "1.000000E-3",
                    prefix + ":PHASE": "0",
                    prefix + ":FUNCTION:RAMP:SYMMETRY": "50",
                    prefix + ":FUNCTION:SQUARE:DUTY": "50",
                    prefix + ":VOLTAGE:AMPLITUDE": "5",
                    prefix + ":VOLTAGE:OFFSET": "0",
                    prefix + ":VOLTAGE:HIGH": "2.5",
                    prefix + ":VOLTAGE:LOW": "-2.5",
                    prefix + ":IMPEDANCE": "OMEG",
                    prefix + ":MOD:STATE": "0",
                    prefix + ":MOD:TYPE": "AM",
                    prefix + ":MOD:AM:DEPTH": "100",
                    prefix + ":MOD:AM:INTERNAL:FREQUENCY": "100",
                    prefix + ":MOD:AM:INTERNAL:FUNCTION": "SIN",
                    prefix + ":MOD:FM:DEVIATION": "1000",
                    prefix + ":MOD:FM:INTERNAL:FREQUENCY": "100",
                    prefix + ":MOD:FM:INTERNAL:FUNCTION": "SIN",
                    prefix + ":MOD:PM:DEVIATION": "90",
                    prefix + ":MOD:PM:INTERNAL:FREQUENCY": "100",
                    prefix + ":MOD:PM:INTERNAL:FUNCTION": "SIN",
                }
            )
        return state

    def start(self):
        self.thread.start()

    def stop(self):
        self.shutdown()
        self.server_close()
        self.thread.join(timeout=2)


class FakeTerminal:
    def __init__(self, events=(), size=(120, 28)):
        self.events = list(events)
        self._size = size
        self.frames = []

    def size(self):
        return self._size

    def draw(self, frame):
        self.frames.append(frame)

    def read_event(self, timeout=None):
        if self.events:
            return self.events.pop(0)
        return TerminalEvent("key", key="escape")


class HelperTests(unittest.TestCase):
    def test_engineering_numbers_and_invalid_measurement(self):
        self.assertEqual(parse_engineering_number("10n"), 1e-8)
        self.assertEqual(parse_engineering_number("500m"), 0.5)
        self.assertEqual(parse_engineering_number("1.25G"), 1.25e9)
        self.assertTrue(invalid_measurement(9.9e37))
        self.assertFalse(invalid_measurement(1.0))

    def test_identity_and_display_name(self):
        candidate = DiscoveredScope(
            "192.0.2.120",
            5555,
            "RIGOL TECHNOLOGIES,DHO924,PUBLIC-DHO-001,00.01.14",
        )
        self.assertTrue(candidate.display_name().startswith("DHO924"))
        self.assertTrue(is_supported_scope_identity(candidate.identity))
        self.assertEqual(identity_key(candidate.identity)[2], "PUBLIC-DHO-001")
        mho = DiscoveredScope(
            "192.0.2.149",
            5555,
            "RIGOL TECHNOLOGIES,MHO934,PUBLIC-MHO-001,00.01.00",
        )
        self.assertTrue(mho.display_name().startswith("MHO934"))
        self.assertTrue(is_supported_scope_identity(mho.identity))
        self.assertTrue(is_mho_scope_identity(mho.identity))
        self.assertEqual(scope_family_from_identity(mho.identity), "MHO")

    def test_terminal_parser_and_waveform_preview(self):
        self.assertEqual(TerminalSession._parse_escape("\x1b[A").key, "up")
        mouse = TerminalSession._parse_escape("\x1b[<0;12;8M")
        self.assertEqual((mouse.kind, mouse.x, mouse.y), ("mouse", 12, 8))
        preview = waveform_ascii_preview([0, 1, 0, -1] * 20, 30, 7)
        self.assertEqual(len(preview), 7)
        self.assertTrue(any("*" in line for line in preview))

    def test_preamble_parser(self):
        preamble = WaveformPreamble.parse(
            "1,0,1000,1,1e-10,-5e-8,0,1.3333e-4,2074,32768"
        )
        self.assertEqual(preamble.format_name, "WORD")
        self.assertEqual(preamble.mode_name, "NORMAL")
        self.assertEqual(preamble.points, 1000)

    def test_official_dho_enumerations(self):
        acquisition = {spec.label: spec.choices for spec in ACQUISITION_SPECS}
        mho_acquisition = {spec.label: spec.choices for spec in MHO_ACQUISITION_SPECS}
        timebase = {spec.label: spec.choices for spec in TIMEBASE_SPECS}
        trigger = {spec.label: spec.choices for spec in TRIGGER_SPECS}
        self.assertIn("ULTRa", acquisition["采集类型"])
        self.assertIn("5M", acquisition["存储深度"])
        self.assertIn("HRESolution", mho_acquisition["采集类型"])
        self.assertEqual(mho_acquisition["高分辨率位数"], ("14", "16"))
        self.assertEqual(timebase["时基模式"], ("MAIN", "XY", "ROLL"))
        self.assertIn("CENTer", timebase["水平参考模式"])
        self.assertIn("SETup", trigger["触发类型"])
        self.assertNotIn("SHOLd", trigger["触发类型"])
        self.assertIn("RRDELAY", MEASUREMENT_ITEMS)
        parser = build_parser()
        args = parser.parse_args(
            ["timebase", "--mode", "MAIN", "--reference-mode", "CENT"]
        )
        self.assertEqual((args.mode, args.reference_mode), ("MAIN", "CENT"))
        args = parser.parse_args(["acquisition", "--type", "HRES", "--bits", "16"])
        self.assertEqual((args.acquisition_type, args.resolution_bits), ("HRES", 16))

    def test_long_choice_list_scrolls_and_multidigit_selects(self):
        app = DHO924TerminalApp("127.0.0.1", 5555, 0.1, cache_path=None)
        choices = tuple(f"Device {index}" for index in range(15))
        app.terminal = FakeTerminal(
            (TerminalEvent("key", key="pagedown"), TerminalEvent("key", key="enter")),
            size=(100, 12),
        )
        self.assertEqual(app.prompt_choice("选择", choices), choices[4])
        app.terminal = FakeTerminal(
            (
                TerminalEvent("text", text="1"),
                TerminalEvent("text", text="2"),
                TerminalEvent("key", key="enter"),
            ),
            size=(100, 12),
        )
        self.assertEqual(app.prompt_choice("选择", choices), choices[11])


class InstrumentTests(unittest.TestCase):
    def setUp(self):
        self.server = FakeDHOServer()
        self.server.start()
        self.host, self.port = self.server.server_address

    def tearDown(self):
        self.server.stop()

    def device(self):
        return RigolDHO(self.host, self.port, timeout=1.0)

    def test_identity_status_and_channel_queries(self):
        with self.device() as scope:
            self.assertIn("DHO924", scope.identify())
            self.assertEqual(scope.scpi_version(), "3.0")
            status = scope.status()
            self.assertEqual(status["trigger"]["mode"], "EDGE")
            self.assertTrue(status["channels"]["1"]["display"])
            self.assertFalse(status["channels"]["2"]["display"])
            self.assertIsNone(scope.digital_state()["active"])
            self.assertFalse(scope.display_state()["color_temperature"])

    def test_channel_configuration(self):
        with self.device() as scope:
            state = scope.configure_channel(
                2,
                display=True,
                scale=0.5,
                offset=-0.1,
                coupling="AC",
                probe=10,
                bandwidth="20M",
                invert=True,
                label="INPUT",
            )
            self.assertTrue(state["display"])
            self.assertEqual(state["scale"], 0.5)
            self.assertEqual(state["coupling"], "AC")
        commands = "\n".join(self.server.commands)
        self.assertIn(":CHANnel2:SCALe 0.5", commands)
        self.assertIn(':CHANnel2:LABel:CONTent "INPUT"', commands)

    def test_acquisition_timebase_and_trigger_configuration(self):
        with self.device() as scope:
            acquisition = scope.configure_acquisition(
                acquisition_type="AVER", memory_depth="1M", averages=16
            )
            timebase = scope.configure_timebase(scale=1e-6, offset=2e-6)
            trigger = scope.configure_trigger(
                mode="EDGE", source="CHAN3", level=0.2, slope="NEG", sweep="NORM"
            )
            self.assertEqual(acquisition["type"], "AVER")
            self.assertEqual(timebase["scale"], 1e-6)
            self.assertEqual(trigger["source"], "CHAN3")
            self.assertAlmostEqual(trigger["level"], 0.2)

    def test_mho_acquisition_afg_and_optional_state(self):
        self.server.identity = "RIGOL TECHNOLOGIES,MHO934,TEST-MHO-0001,00.01.00"
        with self.device() as scope:
            self.assertIn("MHO934", scope.identify())
            acquisition = scope.acquisition_state()
            self.assertEqual(acquisition["bits"], 14)
            self.assertNotIn("ultra_mode", acquisition)
            acquisition = scope.configure_acquisition(
                acquisition_type="HRES", memory_depth="100M", resolution_bits=16
            )
            self.assertEqual(acquisition["type"], "HRES")
            self.assertEqual(acquisition["bits"], 16)
            self.assertTrue(scope.afg_option_state()["afg100"])
            initial = scope.afg_state(1)
            self.assertFalse(initial["output"])
            configured = scope.configure_afg(
                1,
                function="SQU",
                frequency=10_000,
                duty=40,
                amplitude=1,
                offset=0,
            )
            self.assertEqual(configured["function"], "SQU")
            self.assertEqual(configured["frequency"], 10_000)
            self.assertEqual(configured["duty"], 40)
            self.assertFalse(configured["output"])
            self.assertIsNone(scope.dvm_state()["current"])
            self.assertIsNone(scope.digital_state()["active"])
        commands = "\n".join(self.server.commands).upper()
        self.assertIn(":ACQUIRE:BITS 16", commands)
        self.assertNotIn(":ACQUIRE:ULTRA", commands)
        self.assertIn(":SOURCE1:FUNCTION SQU", commands)
        self.assertNotIn(":SOURCE1:OUTPUT:STATE ON", commands)

    def test_measurement_counter_and_dvm(self):
        with self.device() as scope:
            self.assertTrue(invalid_measurement(scope.measure("VPP", "CHAN1")))
            self.assertAlmostEqual(scope.measure("VPP", "CHAN3"), 6.1867e-4)
            counter = scope.configure_counter(enabled=True, source="CHAN3", mode="FREQ")
            dvm = scope.configure_dvm(enabled=True, source="CHAN1", mode="DC")
            self.assertTrue(counter["enabled"])
            self.assertEqual(counter["source"], "CHAN3")
            self.assertTrue(dvm["enabled"])

    def test_waveform_decode_and_followup_query(self):
        with self.device() as scope:
            waveform = scope.read_waveform(
                source="CHAN1", mode="NORM", data_format="WORD", points=1000
            )
            self.assertEqual(len(waveform.volts), 1000)
            self.assertEqual(len(waveform.raw), 2000)
            self.assertGreater(waveform.peak_to_peak, 0)
            self.assertIn("DHO924", scope.identify())

    def test_waveform_skips_redundant_hidden_source_write(self):
        self.server.state[":CHANNEL1:DISPLAY"] = "0"
        self.server.state[":WAVEFORM:SOURCE"] = "CHAN1"
        with self.device() as scope:
            waveform = scope.read_waveform(
                source="CHAN1", mode="NORM", data_format="WORD", points=1000
            )
            self.assertEqual(waveform.source, "CHAN1")
            self.assertEqual(scope.system_error(), '0,"No error"')
        source_writes = [
            command
            for command in self.server.commands
            if command.upper().startswith(":WAVEFORM:SOURCE ")
        ]
        self.assertEqual(source_writes, [])

    def test_waveform_rejects_new_hidden_source_before_remote_error(self):
        self.server.state[":CHANNEL1:DISPLAY"] = "0"
        self.server.state[":WAVEFORM:SOURCE"] = "CHAN3"
        with self.device() as scope:
            with self.assertRaisesRegex(ScopeError, "CHAN1.*OFF"):
                scope.read_waveform(
                    source="CHAN1", mode="NORM", data_format="WORD", points=1000
                )
            self.assertEqual(scope.system_error(), '0,"No error"')
        self.assertNotIn(":WAVeform:SOURce CHAN1", self.server.commands)

    def test_tui_preview_uses_the_only_displayed_channel(self):
        for channel in range(1, 5):
            self.server.state[f":CHANNEL{channel}:DISPLAY"] = (
                "1" if channel == 3 else "0"
            )
        app = DHO924TerminalApp(self.host, self.port, 1.0, cache_path=None)
        app.instrument.connect()
        app.identity = app.instrument.identify()
        try:
            app._capture_preview()
            self.assertEqual(app.channel, 3)
            self.assertIsNotNone(app.last_waveform)
            self.assertEqual(app.last_waveform.source, "CHAN3")
            self.assertTrue(any("CH3" in message for message in app.logs))
            self.assertEqual(app.instrument.system_error(), '0,"No error"')
        finally:
            app.instrument.close()

    def test_screenshot_binary_block(self):
        with self.device() as scope:
            payload = scope.screenshot("PNG")
            self.assertTrue(payload.startswith(b"\x89PNG"))
            self.assertEqual(scope.query("*OPC?"), "0")

    def test_discovery_and_cache(self):
        candidates = discover_scopes(
            ("127.0.0.1/32",), ports=(self.port,), timeout=0.2, workers=2
        )
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].host, self.host)
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory) / "scope.json"
            self.assertTrue(save_last_scope(candidates[0], cache))
            self.assertEqual(load_last_scope(cache), candidates[0])
            location = locate_scope(
                None,
                self.port,
                0.5,
                subnets=("192.0.2.1/32",),
                cache_path=cache,
                scan_timeout=0.1,
            )
            self.assertEqual(location.source, "last")

    def test_explicit_host_is_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory) / "scope.json"
            save_last_scope(DiscoveredScope(self.host, self.port, self.server.identity), cache)
            location = locate_scope(
                "127.0.0.2",
                self.port,
                0.05,
                subnets=("127.0.0.1/32",),
                cache_path=cache,
            )
            self.assertEqual(location.source, "explicit-not-found")
            self.assertIsNone(location.selected)

    def test_manual_selection_even_for_one_scope(self):
        candidate = DiscoveredScope(self.host, self.port, self.server.identity)
        prompts = []

        def answer(prompt):
            prompts.append(prompt)
            return "1"

        selected = choose_console_scope(
            (candidate,), input_func=answer, output=io.StringIO(), require_tty=False
        )
        self.assertEqual(selected, candidate)
        self.assertEqual(len(prompts), 1)

    def test_tui_catalog_and_keyboard(self):
        app = DHO924TerminalApp(self.host, self.port, 1.0, cache_path=None)
        self.assertEqual(len(app.TABS), 11)
        total = 0
        for index in range(len(app.TABS)):
            app.tab_index = index
            total += len(app._actions())
        self.assertGreaterEqual(total, 120)
        app._handle_event(TerminalEvent("text", text="4"))
        self.assertEqual(app.channel, 4)
        app._handle_event(TerminalEvent("key", key="escape"))
        self.assertFalse(app.running)
        app.running = True
        app._handle_event(TerminalEvent("text", text="q"))
        self.assertFalse(app.running)

    def test_mho_tui_exposes_hres_and_afg(self):
        app = DHO924TerminalApp(self.host, self.port, 1.0, cache_path=None)
        app.identity = "RIGOL TECHNOLOGIES,MHO934,TEST-MHO-0001,00.01.00"
        app.scope_compatible = True
        app.tab_index = app.TABS.index("采集")
        labels = [action.label for action in app._actions()]
        self.assertIn("高分辨率位数", labels)
        self.assertNotIn("凝时模式", labels)
        app.tab_index = app.TABS.index("MHO/AFG")
        labels = [action.label for action in app._actions()]
        self.assertIn("查询 MHO AFG 选件", labels)
        self.assertIn("AFG1 输出开关", labels)

    def test_tui_scan_reselects_connected_scope_with_fresh_socket(self):
        app = DHO924TerminalApp(
            self.host,
            self.port,
            1.0,
            discovery_subnets=("127.0.0.1/32",),
            scan_timeout=0.2,
            cache_path=None,
        )
        app.instrument.connect()
        old_socket = app.instrument._socket
        app.terminal = FakeTerminal((TerminalEvent("key", key="enter"),))
        self.assertTrue(app._connect_with_discovery(force_scan=True))
        self.assertTrue(app.instrument.is_connected)
        self.assertIsNot(app.instrument._socket, old_socket)
        self.assertEqual(old_socket.fileno(), -1)
        app.instrument.close()

    def test_tui_disconnected_operation_does_not_scan(self):
        app = DHO924TerminalApp(self.host, self.port, 1.0, cache_path=None)
        calls = []
        app._connect_with_discovery = lambda **kwargs: calls.append(kwargs) or False
        self.assertIsNone(app._safe("测试", lambda: "不应执行"))
        self.assertEqual(calls, [])

    def test_tui_render(self):
        app = DHO924TerminalApp(self.host, self.port, 1.0, cache_path=None)
        app.terminal = FakeTerminal()
        app.identity = self.server.identity
        app._render()
        self.assertTrue(app.terminal.frames)
        self.assertIn("RIGOL DHO924", app.terminal.frames[-1])

        app.identity = "RIGOL TECHNOLOGIES,MHO934,TEST-MHO-0001,00.01.00"
        app._render()
        self.assertIn("RIGOL MHO934", app.terminal.frames[-1])

    def test_cli_idn_channel_waveform_and_screenshot(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            self.assertEqual(main(["--host", self.host, "--port", str(self.port), "idn"]), 0)
            self.assertEqual(
                main(
                    [
                        "--host", self.host, "--port", str(self.port),
                        "channel", "--channel", "1", "--scale", "500m",
                    ]
                ),
                0,
            )
            with tempfile.TemporaryDirectory() as directory:
                csv_path = Path(directory) / "wave.csv"
                png_path = Path(directory) / "screen.png"
                self.assertEqual(
                    main(
                        [
                            "--host", self.host, "--port", str(self.port),
                            "waveform", "--source", "CHAN1", "--csv", str(csv_path),
                        ]
                    ),
                    0,
                )
                self.assertTrue(csv_path.exists())
                self.assertEqual(
                    main(
                        [
                            "--host", self.host, "--port", str(self.port),
                            "screenshot", str(png_path),
                        ]
                    ),
                    0,
                )
                self.assertTrue(png_path.read_bytes().startswith(b"\x89PNG"))
        self.assertIn("DHO924", stdout.getvalue())

    def test_mho_cli_status_acquisition_and_afg_query(self):
        self.server.identity = "RIGOL TECHNOLOGIES,MHO934,TEST-MHO-0001,00.01.00"
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            self.assertEqual(
                main(["--host", self.host, "--port", str(self.port), "status"]),
                0,
            )
            self.assertEqual(
                main(
                    [
                        "--host", self.host, "--port", str(self.port),
                        "acquisition", "--type", "HRES", "--bits", "16",
                    ]
                ),
                0,
            )
            self.assertEqual(
                main(
                    [
                        "--host", self.host, "--port", str(self.port),
                        "afg", "--channel", "1",
                    ]
                ),
                0,
            )
        self.assertEqual(stderr.getvalue(), "")
        self.assertIn('"bits": 16', stdout.getvalue())
        self.assertIn('"output": false', stdout.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
