import io
import socketserver
import tempfile
import threading
import unittest
from pathlib import Path

from sdg2122x_control import (
    DiscoveredInstrument,
    DiscoveryProgress,
    InstrumentCommandError,
    SDG2122X,
    SDGModelProfile,
    SDGTerminalApp,
    SineConfiguration,
    TerminalEvent,
    TerminalSession,
    choose_console_device,
    discover_instruments,
    get_sdg_model_profile,
    is_siglent_sdg_identity,
    load_last_device,
    locate_instrument,
    parse_engineering_number,
    parse_scpi_pairs,
    save_last_device,
    scpi_pairs,
    waveform_preview,
)


class FakeInstrumentHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while True:
            raw = self.rfile.readline()
            if not raw:
                return
            command = raw.decode("ascii").strip()
            self.server.commands.append(command)
            if "?" not in command:
                self.apply_write(command)
                continue
            response = self.response_for(command)
            self.wfile.write(response.encode("ascii") + b"\n")
            self.wfile.flush()

    def apply_write(self, command):
        upper = command.upper()
        if upper.startswith("OUT_BOTHCH "):
            state = upper.split(None, 1)[1]
            if state in {"ON", "OFF"}:
                with self.server.state_lock:
                    self.server.output_states = {1: state, 2: state}
            return
        if upper.startswith(("C1:OUTP ", "C2:OUTP ")):
            channel = int(upper[1])
            state = upper.split(None, 1)[1].split(",", 1)[0]
            if state in {"ON", "OFF"}:
                with self.server.state_lock:
                    self.server.output_states[channel] = state

    def response_for(self, command):
        upper = command.upper()
        if upper == "*IDN?":
            return self.server.identity
        if upper == "*OPC?":
            return "1"
        if "BSWV?" in upper:
            channel = upper.split(":", 1)[0]
            return f"{channel}:BSWV WVTP,SINE,FRQ,1000HZ,AMP,1V,OFST,0V,PHSE,0"
        if "OUTP?" in upper:
            channel = upper.split(":", 1)[0]
            with self.server.state_lock:
                state = self.server.output_states[int(channel[1:])]
            return f"{channel}:OUTP {state},LOAD,HZ,PLRT,NOR"
        if "MDWV?" in upper:
            return "C1:MDWV STATE,OFF"
        if "SWWV?" in upper:
            return "C1:SWWV STATE,OFF"
        if "BTWV?" in upper:
            return "C1:BTWV STATE,OFF"
        if "ARWV?" in upper:
            return "C1:ARWV INDEX,2,NAME,StairUp"
        if "SYNC?" in upper:
            return "C1:SYNC OFF,TYPE,CH1"
        if "SRATE?" in upper:
            return "C1:SRATE MODE,DDS"
        if "HARM?" in upper:
            return "C1:HARM HARMSTATE,OFF"
        if "CMBN?" in upper:
            return "C1:CMBN OFF"
        if "INVT?" in upper:
            return "C1:INVT OFF"
        if upper == "COUP?":
            return "COUP TRACE,OFF,FCOUP,OFF,PCOUP,OFF,ACOUP,OFF"
        if upper == "MODE?":
            return "MODE PHASE-LOCKED"
        if upper == "CASCADE?":
            return "CASCADE STATE,OFF,MODE,MASTER"
        if upper == "FCNT?":
            return "FCNT STATE,OFF"
        if upper == "VOLTPRT?":
            return "ON"
        if upper == "BUZZ?":
            return "BUZZ ON"
        if upper == "SCSV?":
            return "SCSV OFF"
        if upper == "ROSC?":
            return "ROSC INT,10MOUT,OFF"
        if upper == "NBFM?":
            return "NBFM PNT,DOT,SEPT,SPACE"
        if upper == "LAGG?":
            return "LAGG CH"
        if upper == "SCFG?":
            return "SCFG USER"
        if "SYST:COMM:LAN:IPAD?" in upper:
            return '"127.0.0.1"'
        if "SYST:COMM:LAN:SMAS?" in upper:
            return '"255.0.0.0"'
        if "SYST:COMM:LAN:GAT?" in upper:
            return '"127.0.0.1"'
        if upper.startswith("STL?"):
            return "STL WVNM,wave1,wave2"
        if upper.startswith("WVDT?"):
            return "WVDT WVNM,wave1,WAVEDATA,0x00007fff"
        if upper == "SYST:ERR?":
            return '0,"No error"'
        return "1"


class FakeInstrumentServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

    def __init__(self, identity="Siglent Technologies,SDG2122X,TEST0001,2.01.01"):
        super().__init__(("127.0.0.1", 0), FakeInstrumentHandler)
        self.identity = identity
        self.commands = []
        self.output_states = {1: "OFF", 2: "OFF"}
        self.state_lock = threading.Lock()
        self.thread = threading.Thread(target=self.serve_forever, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.shutdown()
        self.server_close()
        self.thread.join(timeout=2)


class FakeTerminal:
    def __init__(self, events=(), size=(100, 16)):
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
    def test_engineering_numbers(self):
        self.assertEqual(parse_engineering_number("10k"), 10_000)
        self.assertEqual(parse_engineering_number("1.2M"), 1_200_000)
        self.assertAlmostEqual(parse_engineering_number("500m"), 0.5)

    def test_scpi_helpers(self):
        self.assertEqual(scpi_pairs([("STATE", True), ("FRQ", 1000.0)]), "STATE,ON,FRQ,1000")
        parsed = parse_scpi_pairs("C1:BSWV WVTP,SINE,FRQ,1000HZ,AMP,1V")
        self.assertEqual(parsed["WVTP"], "SINE")
        self.assertEqual(parsed["FRQ"], "1000HZ")

    def test_terminal_escape_parser(self):
        event = TerminalSession._parse_escape("\x1b[A")
        self.assertEqual(event.key, "up")
        mouse = TerminalSession._parse_escape("\x1b[<0;12;8M")
        self.assertEqual((mouse.kind, mouse.x, mouse.y), ("mouse", 12, 8))

    def test_waveform_preview(self):
        preview = waveform_preview("SINE", 30, 7)
        self.assertEqual(len(preview), 7)
        self.assertTrue(any("*" in line for line in preview))

    def test_long_terminal_choice_list_scrolls(self):
        app = SDGTerminalApp("127.0.0.1", 5025, 0.1, cache_path=None)
        app.terminal = FakeTerminal(
            (
                TerminalEvent("key", key="pagedown"),
                TerminalEvent("key", key="enter"),
            ),
            size=(100, 12),
        )
        choices = tuple(f"Device {index}" for index in range(12))
        self.assertEqual(app.prompt_choice("选择设备", choices), choices[4])

        app.terminal = FakeTerminal(
            (
                TerminalEvent("text", text="1"),
                TerminalEvent("text", text="2"),
                TerminalEvent("key", key="enter"),
            ),
            size=(100, 12),
        )
        self.assertEqual(app.prompt_choice("选择设备", choices), choices[11])

    def test_device_selection_prioritizes_model_and_uses_wide_overlay(self):
        candidate = DiscoveredInstrument(
            "192.0.2.213",
            5025,
            "Siglent Technologies,SDG2122X,TEST-SERIAL,2.01.01.35R3B2",
        )
        self.assertTrue(candidate.display_name().startswith("SDG2122X"))
        self.assertIn("192.0.2.213:5025", candidate.display_name())

        app = SDGTerminalApp("127.0.0.1", 5025, 0.1, cache_path=None)
        app._overlay_title = "设备选择"
        app._overlay_lines = [candidate.display_name()]
        app._overlay_wide = True
        rendered = app._render_overlay([" " * 100 for _ in range(16)], 100, 16)
        border = next(line for line in rendered if "┌" in line)
        self.assertEqual(border.count("─"), 94)

    def test_tui_renders_realtime_scan_progress(self):
        app = SDGTerminalApp("127.0.0.1", 5025, 0.1, cache_path=None)
        app.terminal = FakeTerminal()
        candidate = DiscoveredInstrument(
            "127.0.0.1", 5025, "Acme Instruments,SCOPE1000,A001,1.0"
        )
        app._show_scan_progress(
            DiscoveryProgress(50, 100, "127.0.0.50", 5025, (candidate,))
        )
        self.assertTrue(app._scan_overlay_active)
        self.assertTrue(any("50/100" in line for line in app._overlay_lines))
        self.assertTrue(any("已发现: 1" in line for line in app._overlay_lines))
        app._clear_scan_progress()
        self.assertFalse(app._scan_overlay_active)

    def test_sdg_identity_is_not_limited_to_one_model(self):
        self.assertTrue(
            is_siglent_sdg_identity("Siglent Technologies,SDG1032X,TEST0002,1.0")
        )
        self.assertFalse(is_siglent_sdg_identity("Acme Instruments,SCOPE1000,A001,1.0"))

    def test_sdg1032x_profile_uses_official_model_limits(self):
        profile = get_sdg_model_profile(
            "Siglent Technologies,SDG1032X,TEST0002,1.0"
        )
        self.assertIsInstance(profile, SDGModelProfile)
        assert profile is not None
        self.assertEqual(profile.model, "SDG1032X")
        self.assertEqual(profile.family, "SDG1000X")
        self.assertEqual(profile.frequency_limit("SINE"), 30_000_000)
        self.assertEqual(profile.frequency_limit("SQUARE"), 30_000_000)
        self.assertEqual(profile.frequency_limit("PULSE"), 12_500_000)
        self.assertEqual(profile.frequency_limit("RAMP"), 500_000)
        self.assertEqual(profile.frequency_limit("ARB"), 6_000_000)
        self.assertEqual(profile.max_arbitrary_points, 16_384)
        self.assertFalse(profile.supports("sample_rate"))
        self.assertFalse(profile.supports("cascade"))
        self.assertFalse(profile.supports("noise_add"))
        self.assertTrue(profile.supports("harmonic"))
        self.assertTrue(profile.supports("combine"))
        self.assertTrue(profile.supports("frequency_counter"))


class InstrumentApiTests(unittest.TestCase):
    def setUp(self):
        self.server = FakeInstrumentServer()
        self.server.start()
        self.host, self.port = self.server.server_address

    def tearDown(self):
        self.server.stop()

    def device(self):
        return SDG2122X(self.host, self.port, timeout=1.0)

    def test_identity_and_status(self):
        with self.device() as device:
            self.assertIn("SDG2122X", device.identify())
            self.assertIn("WVTP,SINE", device.basic_waveform(1))
            self.assertIn("OUTP OFF", device.output_status(1))

    def test_major_function_families(self):
        with self.device() as device:
            device.set_basic_wave(1, [("WVTP", "SINE"), ("FRQ", 10_000), ("AMP", 1)])
            device.configure_output(1, enabled=True, load="HZ", polarity="NOR")
            device.set_modulation(1, "AM", [("SRC", "INT"), ("FRQ", 1000), ("DEPTH", 50)])
            device.set_sweep(1, [("STATE", True), ("START", 100), ("STOP", 1000)])
            device.set_burst(1, [("STATE", True), ("TIME", 5)])
            device.select_arbitrary(1, index=2)
            device.set_sync(1, True)
            device.set_coupling([("TRACE", "ON")])
            device.set_sample_rate(1, mode="TARB", value=1_000_000)
            device.set_harmonic(1, [("HARMSTATE", "ON"), ("HARMORDER", 2)])
            device.set_combine(1, True)
            device.set_frequency_counter([("STATE", "ON"), ("REFQ", 1000)])
            device.set_overvoltage_protection(True)
            device.set_buzzer(True)
            device.set_screen_saver(15)
            device.set_clock_source("INT")
            device.set_language("CH")
            device.set_startup_config("USER")
            device.set_network(ip="127.0.0.2", mask="255.0.0.0", gateway="127.0.0.1")

        commands = "\n".join(self.server.commands)
        self.assertIn("C1:BSWV WVTP,SINE,FRQ,10000,AMP,1", commands)
        self.assertIn("C1:OUTP ON,LOAD,HZ,PLRT,NOR", commands)
        self.assertIn("C1:MDWV AM,SRC,INT,FRQ,1000,DEPTH,50", commands)
        self.assertIn("C1:SWWV STATE,ON,START,100,STOP,1000", commands)
        self.assertIn("C1:BTWV STATE,ON,TIME,5", commands)
        self.assertIn("C1:ARWV INDEX,2", commands)
        self.assertIn('SYST:COMM:LAN:IPAD "127.0.0.2"', commands)

    def test_arbitrary_upload_encoding(self):
        with self.device() as device:
            device.upload_waveform_samples(1, "test", [-1.0, 0.0, 1.0])
        command = next(cmd for cmd in self.server.commands if ":WVDT " in cmd)
        self.assertIn("800100007fff", command)

    def test_sdg1032x_enforces_frequency_and_waveform_memory_limits(self):
        self.server.identity = "Siglent Technologies,SDG1032X,TEST1032,1.01.01"
        with self.device() as device:
            device.identify()
            device.set_sine(SineConfiguration(1, 30_000_000, 1.0))
            with self.assertRaisesRegex(ValueError, "SDG1032X.*30 MHz"):
                device.set_sine(SineConfiguration(1, 30_000_001, 1.0))
            with self.assertRaisesRegex(ValueError, "PULSE.*12.5 MHz"):
                device.set_basic_wave(
                    1, [("WVTP", "PULSE"), ("FRQ", 12_500_001)]
                )
            with self.assertRaisesRegex(ValueError, "30 MHz"):
                device.set_channel_tokens(
                    1, "MDWV", ("CARR", "FRQ", 30_000_001)
                )
            with self.assertRaisesRegex(ValueError, "Sweep STOP.*30 MHz"):
                device.set_sweep(1, [("START", 100), ("STOP", 30_000_001)])
            with self.assertRaisesRegex(ValueError, "16384"):
                device.upload_waveform_samples(1, "too_long", [0.0] * 16_385)

        commands = "\n".join(self.server.commands)
        self.assertIn("C1:BSWV WVTP,SINE,FRQ,30000000", commands)
        self.assertNotIn("FRQ,30000001", commands)
        self.assertNotIn("FRQ,12500001", commands)
        self.assertNotIn("CARR,FRQ,30000001", commands)
        self.assertNotIn("STOP,30000001", commands)
        self.assertNotIn("too_long", commands)

    def test_sdg1032x_blocks_unsupported_command_families(self):
        self.server.identity = "Siglent Technologies,SDG1032X,TEST1032,1.01.01"
        with self.device() as device:
            device.identify()
            unsupported = (
                lambda: device.set_sample_rate(1, mode="TARB"),
                lambda: device.set_cascade(True),
                lambda: device.set_noise_add(1, True, ratio=20),
                lambda: device.set_arbitrary_marker(1, True),
            )
            for callback in unsupported:
                with self.assertRaises(InstrumentCommandError):
                    callback()

        commands = "\n".join(self.server.commands)
        self.assertNotIn(":SRATE ", commands)
        self.assertNotIn("CASCADE ", commands)
        self.assertNotIn("NOISE_ADD ", commands)
        self.assertNotIn(":MSW ", commands)

    def test_sdg1032x_keeps_supported_major_functions(self):
        self.server.identity = "Siglent Technologies,SDG1032X,TEST1032,1.01.01"
        with self.device() as device:
            device.identify()
            device.set_modulation(1, "AM", [("STATE", True)])
            device.set_sweep(1, [("STATE", True), ("START", 100), ("STOP", 1000)])
            device.set_burst(1, [("STATE", True), ("TIME", 5)])
            device.set_harmonic(1, [("HARMSTATE", True), ("HARMORDER", 2)])
            device.set_combine(1, True)
            device.set_frequency_counter([("STATE", True)])
            output_1, output_2 = device.set_all_outputs(True)
            self.assertIn("OUTP ON", output_1)
            self.assertIn("OUTP ON", output_2)

        commands = "\n".join(self.server.commands)
        self.assertIn("C1:MDWV AM,STATE,ON", commands)
        self.assertIn("C1:SWWV STATE,ON,START,100,STOP,1000", commands)
        self.assertIn("C1:BTWV STATE,ON,TIME,5", commands)
        self.assertIn("C1:HARM HARMSTATE,ON,HARMORDER,2", commands)
        self.assertIn("C1:CMBN ON", commands)
        self.assertIn("FCNT STATE,ON", commands)
        self.assertIn("OUT_BOTHCH ON", commands)

    def test_sdg1032x_tui_hides_unsupported_actions(self):
        self.server.identity = "Siglent Technologies,SDG1032X,TEST1032,1.01.01"
        app = SDGTerminalApp(self.host, self.port, 1.0, cache_path=None)
        app.instrument.connect()
        app._refresh_all()

        channel_labels = {action.label for action in app._channel_actions()}
        arbitrary_labels = {action.label for action in app._arbitrary_actions()}
        advanced_labels = {action.label for action in app._advanced_actions()}
        system_labels = {action.label for action in app._system_actions()}

        self.assertNotIn("采样模式 DDS/TARB", channel_labels)
        self.assertNotIn("TrueArb 采样率", channel_labels)
        self.assertNotIn("任意波 Marker 开关", arbitrary_labels)
        self.assertNotIn("查询多机同步", advanced_labels)
        self.assertNotIn("设置多机同步", advanced_labels)
        self.assertNotIn("噪声叠加 NOISE_ADD", system_labels)
        self.assertNotIn("直接上电开机模式", system_labels)
        self.assertNotIn("前面板按键开关", system_labels)
        self.assertIn("查询谐波参数", advanced_labels)
        self.assertIn("波形合并开关", advanced_labels)
        self.assertIn("SDG1032X", app._channel_summary_lines(120)[0])
        app.instrument.close()

    def test_tui_catalog_contains_all_major_sections(self):
        app = SDGTerminalApp(self.host, self.port, 1.0, cache_path=None)
        expected = {"总览", "通道", "调制", "扫频", "Burst", "任意波", "同步/高级", "系统", "SCPI控制台"}
        self.assertEqual(set(app.TABS), expected)
        self.assertGreater(len(app._channel_actions()), 20)
        self.assertGreater(len(app._modulation_actions()), 10)
        self.assertGreater(len(app._sweep_actions()), 10)
        self.assertGreater(len(app._burst_actions()), 10)

    def test_tui_keyboard_navigation(self):
        app = SDGTerminalApp(self.host, self.port, 1.0, cache_path=None)
        app._handle_event(TerminalEvent("text", text="2"))
        self.assertEqual(app.channel, 2)
        app._handle_event(TerminalEvent("key", key="right"))
        self.assertEqual(app.TABS[app.tab_index], "通道")
        app._handle_event(TerminalEvent("text", text="q"))
        self.assertFalse(app.running)

        escape_app = SDGTerminalApp(self.host, self.port, 1.0, cache_path=None)
        escape_app._handle_event(TerminalEvent("key", key="escape"))
        self.assertFalse(escape_app.running)

    def test_reselecting_connected_device_reopens_control_socket(self):
        app = SDGTerminalApp(
            self.host,
            self.port,
            1.0,
            discovery_subnets=("127.0.0.1/32",),
            scan_timeout=0.2,
            cache_path=None,
        )
        app.instrument.connect()
        app.terminal = FakeTerminal((TerminalEvent("key", key="enter"),))
        old_socket = app.instrument._socket

        self.assertTrue(app._connect_with_discovery(force_scan=True))
        self.assertTrue(app.instrument.is_connected)
        self.assertIsNot(app.instrument._socket, old_socket)
        self.assertEqual(old_socket.fileno(), -1)
        app.instrument.close()

    def test_tui_connection_failures_do_not_start_discovery(self):
        app = SDGTerminalApp(self.host, self.port, 1.0, cache_path=None)
        discovery_calls = []
        app._connect_with_discovery = (
            lambda **kwargs: discovery_calls.append(kwargs) or False
        )

        self.assertIsNone(app._safe("未连接操作", lambda: "不应执行"))
        self.assertEqual(discovery_calls, [])

        connection_calls = []
        app._connect_candidate = (
            lambda candidate, source="discovery":
            connection_calls.append((candidate, source)) or False
        )
        app._dashboard_actions()[0].handler()
        self.assertEqual(discovery_calls, [])
        self.assertEqual(len(connection_calls), 1)
        self.assertEqual(connection_calls[0][1], "reconnect")

    def test_dashboard_toggles_non_selected_channel(self):
        app = SDGTerminalApp(self.host, self.port, 1.0, cache_path=None)
        app.instrument.connect()
        app._refresh_all()
        app.channel = 1

        app._toggle_output(2)
        self.assertEqual(app.channel, 1)
        self.assertEqual(app.output[1]["STATE"], "OFF")
        self.assertEqual(app.output[2]["STATE"], "ON")

        app._toggle_output(2)
        self.assertEqual(app.channel, 1)
        self.assertEqual(app.output[1]["STATE"], "OFF")
        self.assertEqual(app.output[2]["STATE"], "OFF")
        app.instrument.close()

    def test_discovery_accepts_generic_scpi_devices(self):
        self.server.identity = "Acme Instruments,SCOPE1000,A001,1.0"
        progress_events = []
        candidates = discover_instruments(
            ("127.0.0.1/32",),
            ports=(self.port,),
            timeout=0.2,
            workers=4,
            progress_callback=progress_events.append,
        )
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].identity, self.server.identity)
        self.assertFalse(is_siglent_sdg_identity(candidates[0].identity))
        self.assertTrue(self.server.commands)
        self.assertEqual(set(self.server.commands), {"*IDN?"})
        self.assertEqual(progress_events[0].completed, 0)
        self.assertEqual(progress_events[-1].completed, progress_events[-1].total)
        self.assertEqual(progress_events[-1].candidates, tuple(candidates))

    def test_single_scan_result_still_requires_manual_selection(self):
        candidate = DiscoveredInstrument(self.host, self.port, self.server.identity)
        prompts = []

        def answer(prompt):
            prompts.append(prompt)
            return "1"

        selected = choose_console_device(
            (candidate,),
            input_func=answer,
            output=io.StringIO(),
            require_tty=False,
        )
        self.assertEqual(selected, candidate)
        self.assertEqual(len(prompts), 1)

    def test_last_device_is_used_before_scanning(self):
        with tempfile.TemporaryDirectory() as directory:
            cache_path = Path(directory) / "last_device.json"
            remembered = DiscoveredInstrument(self.host, self.port, self.server.identity)
            self.assertTrue(save_last_device(remembered, cache_path))
            self.assertEqual(load_last_device(cache_path), remembered)

            location = locate_instrument(
                None,
                5025,
                0.5,
                subnets=("192.0.2.1/32",),
                scan_timeout=0.1,
                cache_path=cache_path,
            )
            self.assertEqual(location.source, "last")
            self.assertEqual(location.selected, remembered)

            app = SDGTerminalApp(
                None,
                5025,
                0.5,
                cache_path=cache_path,
                discovery_subnets=("192.0.2.1/32",),
            )
            app._connect_initially()
            self.assertTrue(app.instrument.is_connected)
            self.assertEqual(app.host, self.host)
            self.assertIsNone(app.requested_host)
            app.instrument.close()

    def test_explicit_host_never_falls_back_to_last_device(self):
        with tempfile.TemporaryDirectory() as directory:
            cache_path = Path(directory) / "last_device.json"
            save_last_device(
                DiscoveredInstrument(self.host, self.port, self.server.identity),
                cache_path,
            )
            location = locate_instrument(
                "127.0.0.2",
                self.port,
                0.1,
                subnets=("127.0.0.1/32",),
                scan_timeout=0.1,
                cache_path=cache_path,
            )
            self.assertEqual(location.source, "explicit-not-found")
            self.assertIsNone(location.selected)
            self.assertEqual(self.server.commands, [])

    def test_scanning_matches_last_device_after_ip_change(self):
        with tempfile.TemporaryDirectory() as directory:
            cache_path = Path(directory) / "last_device.json"
            previous_identity = "Siglent Technologies,SDG2122X,TEST0001,1.00"
            save_last_device(
                DiscoveredInstrument("127.0.0.2", self.port, previous_identity),
                cache_path,
            )
            location = locate_instrument(
                None,
                self.port,
                0.2,
                subnets=("127.0.0.1/32",),
                scan_timeout=0.2,
                cache_path=cache_path,
            )
            self.assertEqual(location.source, "scan-results")
            self.assertIsNone(location.selected)
            self.assertEqual(len(location.candidates), 1)
            self.assertEqual(location.candidates[0].host, self.host)
            self.assertEqual(location.candidates[0].identity, self.server.identity)

            app = SDGTerminalApp(self.host, self.port, 0.5, cache_path=None)
            self.assertIsNone(app._choose_discovered_device(location.candidates))

    def test_tui_uses_console_only_mode_for_generic_scpi(self):
        self.server.identity = "Acme Instruments,SCOPE1000,A001,1.0"
        app = SDGTerminalApp(self.host, self.port, 0.5, cache_path=None)
        app._connect_initially()
        self.assertTrue(app.instrument.is_connected)
        self.assertFalse(app.sdg_compatible)
        self.assertEqual(app.TABS[app.tab_index], "SCPI控制台")
        self.assertIsNone(app._safe("SDG 专用操作", lambda: "should-not-run"))
        self.assertEqual(
            app._safe("通用操作", lambda: "ok", allow_generic=True),
            "ok",
        )
        app.instrument.close()

    def test_tui_force_scan_requires_confirmation_for_one_device(self):
        app = SDGTerminalApp(
            None,
            self.port,
            0.5,
            discovery_subnets=("127.0.0.1/32",),
            scan_timeout=0.2,
            cache_path=None,
        )
        app.terminal = FakeTerminal((TerminalEvent("key", key="enter"),))
        self.assertTrue(app._connect_with_discovery(force_scan=True))
        self.assertTrue(app.instrument.is_connected)
        self.assertEqual(app.host, self.host)
        self.assertTrue(app.terminal.frames)
        app.instrument.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
