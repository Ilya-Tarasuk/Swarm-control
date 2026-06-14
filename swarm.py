#!/usr/bin/env python3

import json
import math
import os
import pty
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil


BASE_PORT = 14550
GCS_IP = "192.168.1.137"

ARDUPILOT_DIR = Path("/home/orangepi/ardupilot")
PREBUILD_BEFORE_START = True

DEBUG_SITL_LOG_FILES = True
LOG_DIR = Path("sitl_logs")

ROUTE_DIR = Path("routes")
DEFAULT_ROUTE_FILE = "default_circle.json"
SAMPLE_ROUTE_FILE = "short_random_route.json"

BASE_LAT = 47.397742
BASE_LON = 8.545594
START_SPACING_DEG = 0.00009

SITL_STARTUP_GAP_SEC = 1.0
SITL_RESTART_DELAY_SEC = 2.0
HEARTBEAT_RETRIES = 5
HEARTBEAT_TIMEOUT_SEC = 20
CONNECT_RETRIES = 5
WARMUP_SECONDS = 40
AIRBORNE_STABILIZATION_SEC = 5
MISSION_FINISH_TIMEOUT_SEC = 900
LANDED_STABLE_FINISH_SEC = 5
MIN_LANDED_FINISH_ELAPSED_SEC = 20
MIN_AIRBORNE_BEFORE_FINISH_SEC = 5
IGNORE_STATUS_TEXT_AFTER_AUTO_SEC = 5

COPTER_MODE_AUTO = 3
COPTER_MODE_GUIDED = 4

sitl_processes: dict[int, subprocess.Popen] = {}
sitl_pty_masters: dict[int, int] = {}
connections: dict[int, Any] = {}

sitl_processes_lock = threading.Lock()
connections_lock = threading.Lock()
shutdown_event = threading.Event()


# status table
class StatusTable:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.rows: dict[int, dict[str, str]] = {}
        self.phase = "INIT"
        self.start_time = time.time()
        self.enabled = True
        self.first_render = True
        self.last_render_time = 0.0
        self.min_render_interval = 0.15
        self.columns = [
            ("ID", 4),
            ("SYS", 5),
            ("PORT", 7),
            ("SITL", 18),
            ("HEARTBEAT", 18),
            ("CONNECT", 18),
            ("MISSION", 22),
            ("FLIGHT", 22),
            ("ERROR / INFO", 45),
        ]
        self.min_column_widths = {
            "ID": 3,
            "SYS": 4,
            "PORT": 6,
            "SITL": 12,
            "HEARTBEAT": 12,
            "CONNECT": 12,
            "MISSION": 16,
            "FLIGHT": 14,
            "ERROR / INFO": 18,
        }
        self.shrink_priority = [
            "ERROR / INFO",
            "MISSION",
            "FLIGHT",
            "SITL",
            "HEARTBEAT",
            "CONNECT",
        ]

    def _default_row(self, i: int) -> dict[str, str]:
        return {
            "id": str(i),
            "sysid": str(i + 1),
            "port": str(BASE_PORT + i),
            "sitl": "-",
            "heartbeat": "-",
            "connect": "-",
            "mission": "-",
            "flight": "-",
            "error": "-",
        }

    def init_drone(self, i: int) -> None:
        with self.lock:
            self.rows.setdefault(i, self._default_row(i))

    def set_phase(self, phase: str) -> None:
        with self.lock:
            self.phase = phase
        self.render(force=True)

    def update(
        self,
        i: int,
        sysid: Any = None,
        port: Any = None,
        sitl: Any = None,
        heartbeat: Any = None,
        connect: Any = None,
        mission: Any = None,
        flight: Any = None,
        error: Any = None,
    ) -> None:
        with self.lock:
            row = self.rows.setdefault(i, self._default_row(i))
            updates = {
                "sysid": sysid,
                "port": port,
                "sitl": sitl,
                "heartbeat": heartbeat,
                "connect": connect,
                "mission": mission,
                "flight": flight,
                "error": error,
            }
            for key, value in updates.items():
                if value is not None:
                    row[key] = str(value)
        self.render()

    @staticmethod
    def truncate(value: Any, width: int) -> str:
        text = str(value)
        if len(text) <= width:
            return text
        if width <= 3:
            return text[:width]
        return text[: width - 3] + "..."

    def format_cell(self, value: Any, width: int) -> str:
        return self.truncate(value, width).ljust(width)

    @staticmethod
    def terminal_width(default: int = 200) -> int:
        try:
            return os.get_terminal_size().columns
        except OSError:
            return default

    @staticmethod
    def total_table_width(columns: list[tuple[str, int]]) -> int:
        if not columns:
            return 0
        return sum(width for _, width in columns) + 3 * (len(columns) - 1)

    def fitted_columns(self) -> list[tuple[str, int]]:
        columns = [(name, width) for name, width in self.columns]
        available_width = max(self.terminal_width(), 80)
        overflow = self.total_table_width(columns) - available_width

        if overflow <= 0:
            return columns

        width_by_name = dict(columns)

        for name in self.shrink_priority:
            if overflow <= 0:
                break

            current_width = width_by_name.get(name)
            if current_width is None:
                continue

            min_width = self.min_column_widths.get(name, 8)
            can_remove = max(0, current_width - min_width)
            if can_remove <= 0:
                continue

            removed = min(can_remove, overflow)
            width_by_name[name] = current_width - removed
            overflow -= removed

        return [(name, width_by_name[name]) for name, _ in columns]

    @staticmethod
    def disable_line_wrap() -> None:
        print("\033[?7l", end="")

    @staticmethod
    def enable_line_wrap() -> None:
        print("\033[?7h", end="")

    @staticmethod
    def clear_screen() -> None:
        print("\033[2J\033[H", end="")

    def render(self, force: bool = False) -> None:
        if not self.enabled:
            return

        now = time.time()
        if not force and now - self.last_render_time < self.min_render_interval:
            return
        self.last_render_time = now

        with self.lock:
            rows_copy = dict(sorted(self.rows.items()))
            phase = self.phase
            elapsed = int(time.time() - self.start_time)

        if self.first_render:
            self.hide_cursor()
            self.first_render = False

        columns = self.fitted_columns()

        output_lines = [
            "ArduPilot SITL swarm status table",
            f"Elapsed: {elapsed}s | Phase: {phase}",
            "",
        ]

        header_parts = [self.format_cell(name, width) for name, width in columns]
        separator_parts = ["-" * width for _, width in columns]
        output_lines.append(" | ".join(header_parts))
        output_lines.append("-+-".join(separator_parts))

        for _, row in rows_copy.items():
            values = [
                row["id"],
                row["sysid"],
                row["port"],
                row["sitl"],
                row["heartbeat"],
                row["connect"],
                row["mission"],
                row["flight"],
                row["error"],
            ]
            output_lines.append(
                " | ".join(
                    self.format_cell(value, width)
                    for value, (_, width) in zip(values, columns)
                )
            )

        output_lines.extend(["", "Ctrl+C -> stop all SITL"])

        self.disable_line_wrap()
        self.clear_screen()
        print("\n".join(output_lines), end="")
        print("\033[J", end="")
        sys.stdout.flush()

    @staticmethod
    def hide_cursor() -> None:
        print("\033[?25l", end="")
        sys.stdout.flush()

    @staticmethod
    def show_cursor() -> None:
        StatusTable.enable_line_wrap()
        print("\033[?25h", end="")
        sys.stdout.flush()


status_table = StatusTable()


# input
def pause_table_for_input() -> None:
    status_table.enabled = False
    status_table.show_cursor()
    print()


def resume_table_after_input(phase: str | None = None) -> None:
    status_table.enabled = True
    status_table.hide_cursor()
    if phase is None:
        status_table.render(force=True)
    else:
        status_table.set_phase(phase)


def get_drone_count() -> int:
    pause_table_for_input()
    while True:
        raw = input("Enter number of drones: ").strip()
        try:
            count = int(raw)
        except ValueError:
            print("Invalid input. Enter a positive integer.")
            continue

        if count > 0:
            resume_table_after_input()
            return count

        print("Invalid input. Enter a positive integer.")


def wait_for_enter(message: str) -> None:
    pause_table_for_input()
    input(message)
    resume_table_after_input()


def ask_yes_no(message: str, default: str = "y") -> bool:
    pause_table_for_input()

    default = default.lower().strip()
    if default not in {"y", "n"}:
        default = "y"

    suffix = "[Y/n]" if default == "y" else "[y/N]"
    while True:
        raw = input(f"{message} {suffix}: ").strip().lower()
        if raw == "":
            answer = default
            break
        if raw in {"y", "yes", "д", "да"}:
            answer = "y"
            break
        if raw in {"n", "no", "н", "нет"}:
            answer = "n"
            break
        print("Please enter y or n.")

    resume_table_after_input()
    return answer == "y"


def parse_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on", "да", "д"}:
            return True
        if normalized in {"0", "false", "no", "n", "off", "нет", "н"}:
            return False
    return default


# child process terminal
def close_sitl_pty(i: int) -> None:
    master_fd = sitl_pty_masters.pop(i, None)
    if master_fd is not None:
        try:
            os.close(master_fd)
        except OSError:
            pass


def popen_sitl_isolated(
    cmd: list[str],
    stdout_target: int | Any,
    stderr_target: int | Any,
) -> tuple[subprocess.Popen, int]:
    master_fd, slave_fd = pty.openpty()
    try:
        process = subprocess.Popen(
            cmd,
            stdin=slave_fd,
            stdout=stdout_target,
            stderr=stderr_target,
            close_fds=True,
        )
    except Exception:
        try:
            os.close(slave_fd)
        finally:
            os.close(master_fd)
        raise
    finally:
        try:
            os.close(slave_fd)
        except OSError:
            pass

    return process, master_fd


# SITL process control
def kill_process(process: subprocess.Popen, timeout: float = 5.0) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=timeout)


def kill_all_sitl() -> None:
    shutdown_event.set()
    status_table.set_phase("Killing all SITL")

    with sitl_processes_lock:
        local_processes = list(sitl_processes.items())

    for i, process in local_processes:
        status_table.update(i, sitl="STOPPING")
        try:
            kill_process(process)
        except Exception as exc:
            status_table.update(i, error=f"kill failed: {exc}")

    # cleanup old processes if something was left from previous runs
    subprocess.run(["pkill", "-f", "ArduCopter"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "sim_vehicle.py"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "MAVProxy"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    with sitl_processes_lock:
        for drone_id in list(sitl_pty_masters.keys()):
            close_sitl_pty(drone_id)
        sitl_processes.clear()

    with connections_lock:
        for conn in list(connections.values()):
            try:
                conn.close()
            except Exception:
                pass
        connections.clear()

    time.sleep(2)


def signal_handler(sig: int, frame: Any) -> None:
    shutdown_event.set()
    status_table.set_phase("Interrupted by user")

    with connections_lock:
        interrupted_ids = list(connections.keys())
    for drone_id in interrupted_ids:
        status_table.update(drone_id, flight="STOPPING", error="Interrupted")

    kill_all_sitl()
    status_table.show_cursor()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


# route files
def default_circle_route_config() -> dict[str, Any]:
    return {
        "name": "Default circle mission",
        "type": "circle",
        "description": "Built-in circle route. Drones split into left/right groups and fly circle-like missions in opposite directions.",
        "altitude": 10,
        "num_points": 20,
        "spacing": START_SPACING_DEG,
        "offset": START_SPACING_DEG,
        "takeoff": True,
        "land": True,
    }


def sample_short_random_route_config() -> dict[str, Any]:
    return {
        "name": "Short random relative route",
        "type": "relative_waypoints",
        "description": "Short test route. Each drone flies the same small shape relative to its own start position.",
        "altitude": 8,
        "takeoff": True,
        "land": True,
        "waypoints": [
            {"cmd": "WAYPOINT", "north_m": 12, "east_m": 5, "alt": 8},
            {"cmd": "WAYPOINT", "north_m": 18, "east_m": -8, "alt": 8},
            {"cmd": "WAYPOINT", "north_m": 6, "east_m": -14, "alt": 8},
        ],
    }


def ensure_route_files() -> None:
    ROUTE_DIR.mkdir(parents=True, exist_ok=True)

    default_path = ROUTE_DIR / DEFAULT_ROUTE_FILE
    sample_path = ROUTE_DIR / SAMPLE_ROUTE_FILE

    if not default_path.exists():
        default_path.write_text(
            json.dumps(default_circle_route_config(), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

    if not sample_path.exists():
        sample_path.write_text(
            json.dumps(sample_short_random_route_config(), indent=2, ensure_ascii=False),
            encoding="utf-8",
        )


def list_route_files() -> list[Path]:
    ensure_route_files()
    return sorted(path for path in ROUTE_DIR.iterdir() if path.suffix.lower() == ".json")


def load_route_config(path: Path) -> dict[str, Any]:
    route_config = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(route_config, dict):
        raise ValueError("Route file must contain a JSON object")
    if "type" not in route_config:
        raise ValueError("Route file must contain field: type")
    return route_config


def print_route_menu(route_files: list[Path]) -> None:
    print("\nAvailable route files:\n")
    for idx, route_path in enumerate(route_files, start=1):
        try:
            config = load_route_config(route_path)
            name = config.get("name", "-")
            route_type = config.get("type", "-")
            description = config.get("description", "-")
        except Exception as exc:
            name = "INVALID ROUTE FILE"
            route_type = "-"
            description = str(exc)

        print(f"{idx}. {route_path.name}")
        print(f"   name: {name}")
        print(f"   type: {route_type}")
        print(f"   info: {description}")
        print()


def ask_route_number(route_files: list[Path], prompt: str) -> tuple[Path, dict[str, Any]]:
    while True:
        raw = input(prompt).strip()
        if raw == "":
            print("Empty input. Try again.")
            continue

        try:
            selected_index = int(raw)
        except ValueError:
            print(f"Invalid route selection: {raw}")
            print("Try again.")
            continue

        if selected_index < 1 or selected_index > len(route_files):
            print(f"Invalid route number: {selected_index}")
            print("Try again.")
            continue

        selected_route_path = route_files[selected_index - 1]
        try:
            selected_route_config = load_route_config(selected_route_path)
        except Exception as exc:
            print(f"Invalid route file: {selected_route_path}")
            print(str(exc))
            print("Try again.")
            continue

        return selected_route_path, selected_route_config


def select_routes_for_drones(drone_count: int) -> dict[int, dict[str, Any]]:
    status_table.set_phase("Route selection")
    pause_table_for_input()

    route_files = list_route_files()
    if not route_files:
        print("\n[FATAL] No route files found")
        resume_table_after_input("FATAL: no routes")
        kill_all_sitl()
        sys.exit(1)

    print_route_menu(route_files)
    print("Route selection mode:")
    print("1. Use one route for ALL drones")
    print("2. Select route for EACH drone separately")
    print()

    while True:
        mode_raw = input("Select route selection mode [1/2]: ").strip()
        if mode_raw in {"1", ""}:
            selection_mode = "all"
            break
        if mode_raw == "2":
            selection_mode = "per_drone"
            break
        print("Invalid mode. Enter 1 or 2.")

    selected_routes: dict[int, dict[str, Any]] = {}

    if selection_mode == "all":
        selected_route_path, selected_route_config = ask_route_number(
            route_files,
            "Select route number for ALL drones: ",
        )
        for i in range(drone_count):
            selected_routes[i] = {"path": selected_route_path, "config": selected_route_config}
        print(f"\nSelected for ALL drones: {selected_route_path.name}")
    else:
        for i in range(drone_count):
            print()
            print(f"Selecting route for Drone {i} | SYSID {i + 1} | PORT {BASE_PORT + i}")
            selected_route_path, selected_route_config = ask_route_number(
                route_files,
                f"Select route number for Drone {i}: ",
            )
            selected_routes[i] = {"path": selected_route_path, "config": selected_route_config}
            print(f"Drone {i}: selected {selected_route_path.name}")

    print("\nRoute selection summary:")
    for i in range(drone_count):
        print(f"  Drone {i} | SYSID {i + 1}: {selected_routes[i]['path'].name}")
    print()

    input("Press Enter to continue with mission upload...")
    resume_table_after_input("Route selected")

    for i in range(drone_count):
        status_table.update(i, mission=f"ROUTE {selected_routes[i]['path'].name}", error="-")

    return selected_routes


# build and start positions
def prebuild_arducopter_sitl() -> None:
    if not PREBUILD_BEFORE_START:
        return

    status_table.set_phase("Prebuilding ArduCopter SITL")
    if not ARDUPILOT_DIR.is_dir():
        status_table.set_phase("FATAL")
        print(f"\n[FATAL] ARDUPILOT_DIR does not exist: {ARDUPILOT_DIR}")
        kill_all_sitl()
        sys.exit(1)

    commands = [
        ["./waf", "configure", "--board", "sitl"],
        ["./waf", "copter"],
    ]

    for command in commands:
        result = subprocess.run(command, cwd=ARDUPILOT_DIR)
        if result.returncode != 0:
            status_table.set_phase(f"FATAL: {' '.join(command)} failed")
            print(f"\n[FATAL] {' '.join(command)} failed")
            kill_all_sitl()
            sys.exit(1)

    status_table.set_phase("Prebuild OK")
    time.sleep(1)


def generate_start_positions(base_lat: float, base_lon: float, count: int) -> list[tuple[float, float]]:
    return [(base_lat, base_lon + i * START_SPACING_DEG) for i in range(count)]


# start and restart SITL
def start_sitl(i: int, lat: float, lon: float) -> None:
    port = BASE_PORT + i
    location_str = f"{lat},{lon},0,0"
    cmd = [
        "sim_vehicle.py",
        "-v",
        "ArduCopter",
        "--instance",
        str(i),
        "--sysid",
        str(i + 1),
        "--out",
        f"127.0.0.1:{port}",
        "--out",
        f"{GCS_IP}:{port}",
        "--custom-location",
        location_str,
        "--no-rebuild",
        "-w",
    ]

    status_table.update(i, sitl="STARTING", sysid=i + 1, port=port, error="-")

    log_file = None
    try:
        if DEBUG_SITL_LOG_FILES:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            log_path = LOG_DIR / f"sitl_{i}_sysid_{i + 1}_port_{port}.log"
            log_file = log_path.open("w", buffering=1, encoding="utf-8", errors="replace")
            stdout_target = log_file
            stderr_target = subprocess.STDOUT
        else:
            log_path = None
            stdout_target = subprocess.DEVNULL
            stderr_target = subprocess.DEVNULL

        process, master_fd = popen_sitl_isolated(
            cmd=cmd,
            stdout_target=stdout_target,
            stderr_target=stderr_target,
        )
    except Exception as exc:
        status_table.update(i, sitl="START FAIL", error=str(exc))
        raise
    finally:
        if log_file is not None:
            log_file.close()

    with sitl_processes_lock:
        sitl_processes[i] = process
        sitl_pty_masters[i] = master_fd

    if DEBUG_SITL_LOG_FILES:
        status_table.update(i, sitl="STARTED", error=f"log: {log_path}")
    else:
        status_table.update(i, sitl="STARTED", error="-")


def restart_sitl(i: int, lat: float, lon: float) -> None:
    status_table.update(i, sitl="RESTARTING", heartbeat="RETRY", connect="RETRY", error="Restarting SITL")

    with sitl_processes_lock:
        process = sitl_processes.pop(i, None)
        close_sitl_pty(i)

    if process is not None:
        try:
            kill_process(process)
        except Exception as exc:
            status_table.update(i, error=f"restart kill failed: {exc}")

    time.sleep(SITL_RESTART_DELAY_SEC)
    start_sitl(i, lat, lon)


def start_all_sitl_parallel(positions: list[tuple[float, float]], startup_gap: float = SITL_STARTUP_GAP_SEC) -> None:
    status_table.set_phase("Starting SITL")
    threads: list[threading.Thread] = []
    results: dict[int, bool] = {}
    results_lock = threading.Lock()

    for i, _ in enumerate(positions):
        status_table.init_drone(i)

    def start_worker(i: int, lat: float, lon: float) -> None:
        ok = False
        try:
            start_sitl(i, lat, lon)
            ok = True
        except Exception as exc:
            status_table.update(i, sitl="FATAL", error=str(exc))
        finally:
            with results_lock:
                results[i] = ok

    for i, (lat, lon) in enumerate(positions):
        thread = threading.Thread(target=start_worker, args=(i, lat, lon), daemon=True)
        thread.start()
        threads.append(thread)
        time.sleep(startup_gap)

    for thread in threads:
        thread.join()

    with sitl_processes_lock:
        dead_processes = [i for i, process in sitl_processes.items() if process.poll() is not None]

    failed = sorted({i for i, ok in results.items() if not ok} | set(dead_processes))
    if len(results) != len(positions):
        failed.extend(i for i in range(len(positions)) if i not in results)
        failed = sorted(set(failed))

    if failed:
        status_table.set_phase(f"FATAL: SITL start failed {failed}")
        kill_all_sitl()
        sys.exit(1)

    status_table.set_phase("All SITL start commands issued")


# MAVLink connection
def make_mavlink_connection(port: int) -> Any:
    return mavutil.mavlink_connection(f"udp:127.0.0.1:{port}", timeout=5)


def wait_heartbeat_after_start(
    i: int,
    lat: float,
    lon: float,
    retries: int = HEARTBEAT_RETRIES,
    heartbeat_timeout: int = HEARTBEAT_TIMEOUT_SEC,
    retry_delay: int = 5,
) -> bool:
    port = BASE_PORT + i

    for attempt in range(retries):
        status_table.update(i, heartbeat=f"CHECK {attempt + 1}/{retries}", error="-")
        conn = None
        try:
            conn = make_mavlink_connection(port)
            conn.wait_heartbeat(timeout=heartbeat_timeout)
            if conn.target_system == 0:
                raise RuntimeError("Invalid SYSID 0")
            status_table.update(i, heartbeat="OK", sysid=conn.target_system, error="-")
            return True
        except Exception as exc:
            status_table.update(i, heartbeat=f"FAIL {attempt + 1}/{retries}", error=str(exc))
            if attempt < retries - 1:
                restart_sitl(i, lat, lon)
                time.sleep(retry_delay)
        finally:
            if conn is not None:
                try:
                    conn.close()
                except Exception:
                    pass

    return False


def wait_all_heartbeats_parallel(positions: list[tuple[float, float]], retries: int = HEARTBEAT_RETRIES) -> None:
    status_table.set_phase("Waiting heartbeats")
    results: dict[int, bool] = {}
    threads: list[threading.Thread] = []
    lock = threading.Lock()

    def heartbeat_worker(i: int, lat: float, lon: float) -> None:
        ok = wait_heartbeat_after_start(i, lat, lon, retries=retries)
        with lock:
            results[i] = ok

    for i, (lat, lon) in enumerate(positions):
        thread = threading.Thread(target=heartbeat_worker, args=(i, lat, lon), daemon=True)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    failed = [i for i, ok in sorted(results.items()) if not ok]
    if failed:
        for i in failed:
            status_table.update(i, heartbeat="FATAL", error="Heartbeat failed")
        status_table.set_phase(f"FATAL: heartbeat failed {failed}")
        kill_all_sitl()
        sys.exit(1)

    status_table.set_phase("All heartbeats OK")


def connect_with_retry(i: int, lat: float, lon: float, retries: int = CONNECT_RETRIES) -> Any | None:
    port = BASE_PORT + i

    for attempt in range(retries):
        status_table.update(i, connect=f"TRY {attempt + 1}/{retries}", error="-")
        try:
            conn = make_mavlink_connection(port)
            conn.wait_heartbeat(timeout=10)
            if conn.target_system == 0:
                raise RuntimeError("Invalid SYSID 0")
            status_table.update(i, connect="OK", sitl="RUNNING", sysid=conn.target_system, error="-")
            return conn
        except Exception as exc:
            status_table.update(i, connect=f"FAIL {attempt + 1}/{retries}", error=str(exc))
            if attempt < retries - 1:
                restart_sitl(i, lat, lon)
                time.sleep(5)

    return None


def connect_all(count: int, positions: list[tuple[float, float]]) -> None:
    status_table.set_phase("Connecting MAVLink")

    for i in range(count):
        lat, lon = positions[i]
        conn = connect_with_retry(i, lat, lon)
        if conn is None:
            status_table.update(i, connect="FATAL", error="Cannot connect")
            status_table.set_phase("FATAL: cannot connect")
            kill_all_sitl()
            sys.exit(1)

        with connections_lock:
            connections[i] = conn


# mission generation
def mav_cmd_from_string(command_name: Any) -> int:
    command_map = {
        "TAKEOFF": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        "WAYPOINT": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        "LAND": mavutil.mavlink.MAV_CMD_NAV_LAND,
        "LOITER_TIME": mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
        "LOITER_UNLIM": mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
        "RETURN_TO_LAUNCH": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        "RTL": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
    }

    normalized = str(command_name).strip().upper()
    if normalized not in command_map:
        raise ValueError(f"Unsupported mission command: {command_name}")
    return command_map[normalized]


def meters_to_lat_lon(base_lat: float, base_lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(base_lat))
    if abs(meters_per_deg_lon) < 1e-9:
        raise ValueError("Invalid longitude scale near poles")
    return base_lat + north_m / meters_per_deg_lat, base_lon + east_m / meters_per_deg_lon


def add_takeoff_if_needed(mission: list[dict[str, Any]], lat: float, lon: float, altitude: float, enabled: bool) -> None:
    if enabled:
        mission.append({"cmd": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, "lat": lat, "lon": lon, "alt": altitude})


def add_land_if_needed(mission: list[dict[str, Any]], lat: float, lon: float, enabled: bool) -> None:
    if enabled:
        mission.append({"cmd": mavutil.mavlink.MAV_CMD_NAV_LAND, "lat": lat, "lon": lon, "alt": 0})


def get_waypoint_list(route_config: dict[str, Any], drone_index: int) -> list[dict[str, Any]]:
    per_drone = route_config.get("per_drone", {})
    drone_key = str(drone_index)

    if isinstance(per_drone, dict) and drone_key in per_drone:
        waypoint_list = per_drone[drone_key]
    else:
        waypoint_list = route_config.get("waypoints", [])

    if not isinstance(waypoint_list, list):
        raise ValueError("waypoints or per_drone entry must be a list")

    for waypoint in waypoint_list:
        if not isinstance(waypoint, dict):
            raise ValueError("Each waypoint must be a JSON object")

    return waypoint_list


def generate_circle_mission_from_config(
    route_config: dict[str, Any],
    lat: float,
    lon: float,
    i: int,
    total: int,
) -> list[dict[str, Any]]:
    altitude = float(route_config.get("altitude", 10))
    num_points = int(route_config.get("num_points", 20))
    spacing = float(route_config.get("spacing", START_SPACING_DEG))
    offset = float(route_config.get("offset", START_SPACING_DEG))
    add_takeoff = parse_bool(route_config.get("takeoff", True), default=True)
    add_land = parse_bool(route_config.get("land", True), default=True)

    if num_points <= 0:
        raise ValueError("circle route num_points must be greater than 0")

    mid = total // 2
    leftmost_lon = lon - i * spacing
    rightmost_lon = lon + (total - i - 1) * spacing

    if i < mid:
        center_lat = lat
        center_lon = leftmost_lon - offset
        direction = -1
    else:
        center_lat = lat
        center_lon = rightmost_lon + offset
        direction = 1

    lon_scale = math.cos(math.radians(center_lat))
    if abs(lon_scale) < 1e-9:
        raise ValueError("Invalid longitude scale near poles")

    dx = lat - center_lat
    dy = (lon - center_lon) * lon_scale
    radius = math.hypot(dx, dy)
    start_angle = math.atan2(dy, dx)

    mission: list[dict[str, Any]] = []
    add_takeoff_if_needed(mission, lat, lon, altitude, add_takeoff)

    for k in range(num_points):
        angle = start_angle + direction * (2 * math.pi * k / num_points)
        mission.append(
            {
                "cmd": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                "lat": center_lat + radius * math.cos(angle),
                "lon": center_lon + (radius * math.sin(angle)) / lon_scale,
                "alt": altitude,
            }
        )

    add_land_if_needed(mission, lat, lon, add_land)
    return mission


def generate_waypoints_mission_from_config(
    route_config: dict[str, Any],
    lat: float,
    lon: float,
    i: int,
) -> list[dict[str, Any]]:
    altitude = float(route_config.get("altitude", 10))
    add_takeoff = parse_bool(route_config.get("takeoff", True), default=True)
    add_land = parse_bool(route_config.get("land", True), default=True)

    mission: list[dict[str, Any]] = []
    add_takeoff_if_needed(mission, lat, lon, altitude, add_takeoff)

    for wp in get_waypoint_list(route_config, i):
        if "lat" not in wp or "lon" not in wp:
            raise ValueError("Each absolute waypoint must contain lat and lon")
        mission.append(
            {
                "cmd": mav_cmd_from_string(wp.get("cmd", "WAYPOINT")),
                "lat": float(wp["lat"]),
                "lon": float(wp["lon"]),
                "alt": float(wp.get("alt", altitude)),
            }
        )

    add_land_if_needed(mission, lat, lon, add_land)
    return mission


def generate_relative_waypoints_mission_from_config(
    route_config: dict[str, Any],
    lat: float,
    lon: float,
    i: int,
) -> list[dict[str, Any]]:
    altitude = float(route_config.get("altitude", 10))
    add_takeoff = parse_bool(route_config.get("takeoff", True), default=True)
    add_land = parse_bool(route_config.get("land", True), default=True)

    mission: list[dict[str, Any]] = []
    add_takeoff_if_needed(mission, lat, lon, altitude, add_takeoff)

    for wp in get_waypoint_list(route_config, i):
        wp_lat, wp_lon = meters_to_lat_lon(
            base_lat=lat,
            base_lon=lon,
            north_m=float(wp.get("north_m", 0)),
            east_m=float(wp.get("east_m", 0)),
        )
        mission.append(
            {
                "cmd": mav_cmd_from_string(wp.get("cmd", "WAYPOINT")),
                "lat": wp_lat,
                "lon": wp_lon,
                "alt": float(wp.get("alt", altitude)),
            }
        )

    add_land_if_needed(mission, lat, lon, add_land)
    return mission


def generate_mission_from_route(
    route_config: dict[str, Any],
    lat: float,
    lon: float,
    i: int,
    total: int,
) -> list[dict[str, Any]]:
    route_type = str(route_config.get("type", "circle")).strip().lower()

    if route_type == "circle":
        return generate_circle_mission_from_config(route_config, lat, lon, i, total)
    if route_type == "waypoints":
        return generate_waypoints_mission_from_config(route_config, lat, lon, i)
    if route_type == "relative_waypoints":
        return generate_relative_waypoints_mission_from_config(route_config, lat, lon, i)

    raise ValueError(f"Unsupported route type: {route_type}")


# mission upload and launch
def upload_mission(i: int, conn: Any, mission: list[dict[str, Any]]) -> bool:
    try:
        if not mission:
            raise ValueError("Generated mission is empty")

        status_table.update(i, mission="CLEARING", flight="-", error="-")
        conn.mav.mission_clear_all_send(conn.target_system, conn.target_component)
        time.sleep(1)

        status_table.update(i, mission=f"COUNT {len(mission)}")
        conn.mav.mission_count_send(conn.target_system, conn.target_component, len(mission))

        for seq, wp in enumerate(mission):
            status_table.update(i, mission=f"ITEM {seq + 1}/{len(mission)}")
            conn.mav.mission_item_int_send(
                conn.target_system,
                conn.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                int(wp["cmd"]),
                0,
                1,
                0,
                0,
                0,
                0,
                int(float(wp["lat"]) * 1e7),
                int(float(wp["lon"]) * 1e7),
                float(wp["alt"]),
            )
            time.sleep(0.1)

        time.sleep(0.5)
        conn.mav.mission_set_current_send(conn.target_system, conn.target_component, 0)
        status_table.update(i, mission="UPLOADED", error="-")
        return True
    except Exception as exc:
        status_table.update(i, mission="FAIL", error=str(exc))
        return False


def upload_selected_routes(
    selected_routes: dict[int, dict[str, Any]],
    positions: list[tuple[float, float]],
    drone_count: int,
) -> dict[int, int] | None:
    status_table.set_phase("Uploading missions")
    mission_lengths: dict[int, int] = {}
    failed: list[int] = []

    with connections_lock:
        connection_items = list(connections.items())

    for i, conn in connection_items:
        lat, lon = positions[i]
        route_config = selected_routes[i]["config"]
        try:
            mission = generate_mission_from_route(route_config, lat, lon, i, drone_count)
            if upload_mission(i, conn, mission):
                mission_lengths[i] = len(mission)
            else:
                failed.append(i)
        except Exception as exc:
            failed.append(i)
            status_table.update(i, mission="FAIL", error=str(exc))

    if failed:
        status_table.set_phase(f"Mission upload failed: {failed}")
        return None

    return mission_lengths


def set_mode(i: int, conn: Any, mode_name: str, custom_mode: int) -> bool:
    try:
        status_table.update(i, error="-")
        conn.mav.set_mode_send(
            conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
        )
        return True
    except Exception as exc:
        status_table.update(i, error=str(exc))
        return False


def arm_and_takeoff(i: int, conn: Any, altitude: float = 10) -> bool:
    try:
        status_table.update(i, flight="SET GUIDED", error="-")
        set_mode(i, conn, "GUIDED", COPTER_MODE_GUIDED)
        time.sleep(1)

        status_table.update(i, flight="ARMING")
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(2)

        status_table.update(i, flight=f"TAKEOFF {altitude}m")
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )
        status_table.update(i, flight="TAKEOFF CMD SENT")
        return True
    except Exception as exc:
        status_table.update(i, flight="FAIL", error=str(exc))
        return False


def arm_and_takeoff_all_on_key(altitude: float = 10) -> bool:
    status_table.set_phase("Ready for manual takeoff")
    wait_for_enter("\nPress Enter to ARM, TAKEOFF and START mission automatically...")
    status_table.set_phase("Arming and takeoff")

    results: dict[int, bool] = {}
    threads: list[threading.Thread] = []
    lock = threading.Lock()

    def worker(i: int, conn: Any) -> None:
        ok = arm_and_takeoff(i, conn, altitude)
        with lock:
            results[i] = ok

    with connections_lock:
        connection_items = list(connections.items())

    for i, conn in connection_items:
        thread = threading.Thread(target=worker, args=(i, conn), daemon=True)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    failed = [i for i, ok in sorted(results.items()) if not ok]
    if failed:
        status_table.set_phase(f"Arm/takeoff failed: {failed}")
        return False

    status_table.set_phase("Takeoff commands sent")
    return True


def start_auto_all() -> None:
    status_table.set_phase("Starting AUTO missions")
    with connections_lock:
        connection_items = list(connections.items())

    for i, conn in connection_items:
        status_table.update(i, flight="MISSION START")
        set_mode(i, conn, "AUTO", COPTER_MODE_AUTO)

    status_table.set_phase("Missions started")


# mission monitoring
def request_status_stream(conn: Any) -> None:
    try:
        conn.mav.request_data_stream_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2,
            1,
        )
    except Exception:
        pass


def clear_mavlink_buffer(conn: Any, max_messages: int = 500) -> None:
    for _ in range(max_messages):
        try:
            if conn.recv_match(blocking=False) is None:
                break
        except Exception:
            break


def wait_mission_finished(i: int, conn: Any, mission_length: int, timeout: int = MISSION_FINISH_TIMEOUT_SEC) -> bool:
    status_table.update(i, flight="MONITORING", error="-")
    request_status_stream(conn)

    start_time = time.time()
    last_seq = -1
    landed = False
    was_airborne = False
    disarmed = False
    mission_complete_text = False
    last_item = max(mission_length - 1, 0)
    landed_since: float | None = None
    airborne_since: float | None = None

    clear_mavlink_buffer(conn)

    while not shutdown_event.is_set() and time.time() - start_time < timeout:
        try:
            msg = conn.recv_match(
                type=["MISSION_CURRENT", "EXTENDED_SYS_STATE", "HEARTBEAT", "STATUSTEXT"],
                blocking=True,
                timeout=1,
            )
        except OSError as exc:
            if shutdown_event.is_set():
                status_table.update(i, flight="STOPPED", error="Interrupted")
                return False
            status_table.update(i, flight="MONITOR FAIL", error=f"MAVLink socket error: {exc}")
            return False
        except Exception as exc:
            if shutdown_event.is_set():
                status_table.update(i, flight="STOPPED", error="Interrupted")
                return False
            status_table.update(i, flight="MONITOR FAIL", error=f"MAVLink monitor error: {exc}")
            return False

        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "MISSION_CURRENT":
            last_seq = int(msg.seq)
            status_table.update(i, mission=f"SEQ {last_seq}/{last_item}")
        elif msg_type == "EXTENDED_SYS_STATE":
            landed_state = int(msg.landed_state)
            now = time.time()
            if landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                if was_airborne and not landed:
                    landed_since = now
                landed = True
                status_table.update(i, flight="LANDED")
            elif landed_state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR:
                landed = False
                if not was_airborne:
                    airborne_since = now
                was_airborne = True
                landed_since = None
                status_table.update(i, flight="IN AIR")
        elif msg_type == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            disarmed = not armed
        elif msg_type == "STATUSTEXT":
            text = msg.text
            if isinstance(text, bytes):
                text = text.decode(errors="ignore")
            text = text.strip("\x00").strip()
            if time.time() - start_time >= IGNORE_STATUS_TEXT_AFTER_AUTO_SEC:
                if "Mission Complete" in text or "Land complete" in text:
                    mission_complete_text = True
                    status_table.update(i, flight="MISSION COMPLETE")

        reached_last_item = last_seq >= last_item
        monitoring_elapsed = time.time() - start_time
        enough_time_after_airborne = (
            airborne_since is not None
            and time.time() - airborne_since >= MIN_AIRBORNE_BEFORE_FINISH_SEC
        )
        landed_stable = (
            landed
            and was_airborne
            and enough_time_after_airborne
            and landed_since is not None
            and monitoring_elapsed >= MIN_LANDED_FINISH_ELAPSED_SEC
            and time.time() - landed_since >= LANDED_STABLE_FINISH_SEC
        )

        if (
            landed_stable
            or (was_airborne and enough_time_after_airborne and landed and disarmed)
            or (was_airborne and enough_time_after_airborne and reached_last_item and landed)
            or (was_airborne and enough_time_after_airborne and mission_complete_text and landed)
        ):
            status_table.update(i, mission="FINISHED", flight="FINISHED", error="-")
            return True

    if shutdown_event.is_set():
        status_table.update(i, flight="STOPPED", error="Interrupted")
        return False

    status_table.update(i, flight="FINISH TIMEOUT", error="Mission finish timeout")
    return False


def wait_all_missions_finished(mission_lengths: dict[int, int]) -> bool:
    status_table.set_phase("Waiting mission completion")
    results: dict[int, bool] = {}
    threads: list[threading.Thread] = []
    lock = threading.Lock()

    def monitor_worker(i: int, conn: Any, mission_length: int) -> None:
        try:
            ok = wait_mission_finished(i, conn, mission_length)
        except Exception as exc:
            ok = False
            if not shutdown_event.is_set():
                status_table.update(i, flight="MONITOR FAIL", error=str(exc))
        with lock:
            results[i] = ok

    with connections_lock:
        connection_items = list(connections.items())

    for i, conn in connection_items:
        mission_length = mission_lengths.get(i, 0)
        thread = threading.Thread(target=monitor_worker, args=(i, conn, mission_length), daemon=True)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    failed = [i for i, ok in sorted(results.items()) if not ok]
    if failed:
        status_table.set_phase(f"Mission monitoring timeout: {failed}")
    else:
        status_table.set_phase("All missions finished")

    return len(failed) == 0


# main cycle
def wait_warmup(seconds: int) -> None:
    status_table.set_phase("Warmup before route selection")
    for remaining in range(seconds, 0, -1):
        with connections_lock:
            drone_ids = list(connections.keys())
        for i in drone_ids:
            status_table.update(i, error=f"warmup: {remaining}s")
        time.sleep(1)

    with connections_lock:
        drone_ids = list(connections.keys())
    for i in drone_ids:
        status_table.update(i, error="-")


def reset_current_mission_all() -> bool:
    status_table.set_phase("Resetting current mission")
    failed: list[int] = []

    with connections_lock:
        connection_items = list(connections.items())

    for i, conn in connection_items:
        try:
            clear_mavlink_buffer(conn)
            conn.mav.mission_set_current_send(
                conn.target_system,
                conn.target_component,
                0,
            )
            status_table.update(i, mission="RESET TO 0", flight="-", error="-")
        except Exception as exc:
            failed.append(i)
            status_table.update(i, mission="RESET FAIL", error=str(exc))

    if failed:
        status_table.set_phase(f"Mission reset failed: {failed}")
        return False

    return True


def run_selected_mission_cycle(
    cycle_number: int,
    drone_count: int,
    positions: list[tuple[float, float]],
    selected_routes: dict[int, dict[str, Any]],
    mission_lengths: dict[int, int] | None = None,
    upload_before_start: bool = True,
) -> tuple[bool, dict[int, int] | None]:
    if upload_before_start:
        status_table.set_phase(f"Mission cycle {cycle_number}: uploading missions")
        mission_lengths = upload_selected_routes(selected_routes, positions, drone_count)
        if mission_lengths is None:
            return False, None
    else:
        status_table.set_phase(f"Mission cycle {cycle_number}: repeat loaded mission")
        if mission_lengths is None:
            status_table.set_phase("FATAL: no cached mission lengths")
            return False, None
        if not reset_current_mission_all():
            return False, mission_lengths

    with connections_lock:
        for _, conn in connections.items():
            clear_mavlink_buffer(conn)

    if not arm_and_takeoff_all_on_key(altitude=10):
        return False, mission_lengths

    status_table.set_phase("Airborne stabilization")
    time.sleep(AIRBORNE_STABILIZATION_SEC)

    with connections_lock:
        for _, conn in connections.items():
            clear_mavlink_buffer(conn)

    start_auto_all()
    shutdown_event.clear()
    ok = wait_all_missions_finished(mission_lengths)
    return ok, mission_lengths


def select_routes_for_cycle(drone_count: int, cycle_number: int) -> dict[int, dict[str, Any]]:
    status_table.set_phase(f"Mission cycle {cycle_number}: route selection")
    return select_routes_for_drones(drone_count)


def main() -> None:
    drone_count = get_drone_count()

    for drone_id in range(drone_count):
        status_table.init_drone(drone_id)

    ensure_route_files()
    kill_all_sitl()
    shutdown_event.clear()
    prebuild_arducopter_sitl()

    positions = generate_start_positions(BASE_LAT, BASE_LON, drone_count)

    start_all_sitl_parallel(positions)
    wait_all_heartbeats_parallel(positions)
    connect_all(drone_count, positions)
    wait_warmup(WARMUP_SECONDS)

    with connections_lock:
        for i in connections.keys():
            status_table.update(i, error="-")

    cycle_number = 1
    selected_routes: dict[int, dict[str, Any]] | None = None
    mission_lengths: dict[int, int] | None = None
    upload_before_start = True

    while True:
        if selected_routes is None:
            selected_routes = select_routes_for_cycle(drone_count, cycle_number)
            mission_lengths = None
            upload_before_start = True

        ok, mission_lengths = run_selected_mission_cycle(
            cycle_number=cycle_number,
            drone_count=drone_count,
            positions=positions,
            selected_routes=selected_routes,
            mission_lengths=mission_lengths,
            upload_before_start=upload_before_start,
        )

        with connections_lock:
            drone_ids = list(connections.keys())
        for i in drone_ids:
            status_table.update(i, error="ready for next mission" if ok else "mission cycle failed")

        status_table.set_phase("Mission cycle finished" if ok else "Mission cycle failed")

        load_another_mission = ask_yes_no("\nMission finished. Select and run another mission?", default="y")
        if load_another_mission:
            selected_routes = None
            mission_lengths = None
            upload_before_start = True
        else:
            upload_before_start = False

        cycle_number += 1


if __name__ == "__main__":
    try:
        main()
    finally:
        status_table.show_cursor()
