import csv
import math
import os
import time
from dataclasses import dataclass, asdict
from statistics import mean
from typing import Optional, List

import serial
import zhinst.core


################ User Configuration ################

# Arduino connection settings
ARDUINO_PORT = "COM6" # Change to actual port
ARDUINO_BAUD = 115200
SERIAL_TIMEOUT_S = 1.0
SERIAL_STARTUP_WAIT_S = 2.5

# MFLI connection settings
DEVICE_ID = "dev30337" # Change to actual device ID
API_LEVEL = 6

# Total run settings
TOTAL_RUN_TIME_S = 60.0 * 60 * 2  # example: 60.0 * 60 * 2 = run for 2 hours total
SCAN_INTERVAL_S = 60.0 # start a new scan every 60 s

# Sweep settings
SETTLE_DELAY_S = 2.000 # May not be sufficient for major changes between channels
AVERAGE_WINDOW_S = 0.5
POLL_TIMEOUT_MS = 100

# Output file name (will auto-increment if file exists)
CSV_PATH = "main_impedance_sweep.csv"

# MFLI measurement settings
OSC_INDEX = 0
DEMOD_I = 0
DEMOD_V = 1
SIGOUT = 0

OSC_FREQ_HZ = 1000.0

DEMOD_I_TIMECONSTANT_S = 0.08114109382567461
DEMOD_V_TIMECONSTANT_S = 1 / (2 * math.pi * 99.51)

DEMOD_ORDER = 3
DEMOD_HARMONIC = 1
DEMOD_PHASESHIFT_DEG = 0.0
DEMOD_RATE_SA_S = 1674.0

SIGOUT_RANGE_V = 0.1
SIGOUT_OFFSET_V = 0.0
SIGOUT_VRMS_V = 0.070710678

CURRIN_RANGE_A = 1e-2
SIGIN_RANGE_V = 0.1


################ Step Sequence ################

PAIR_STEP_ORDER = [
    (1,  "E0",  "Reference Electrode"),
    (2,  "E1",  "Reference Electrode"),
    (3,  "E2",  "Reference Electrode"),
    (4,  "E3",  "Reference Electrode"),
    (5,  "E4",  "Reference Electrode"),
    (6,  "E5",  "Reference Electrode"),
    (7,  "E6",  "Reference Electrode"),
    (8,  "E7",  "Reference Electrode"),
    (9,  "E8",  "Reference Electrode"),
    (10, "E9",  "Reference Electrode"),
    (11, "E10", "Reference Electrode"),
    (12, "E11", "Reference Electrode"),
    (13, "E12", "Reference Electrode"),
    (14, "E13", "Reference Electrode"),
    (15, "E14", "Reference Electrode"),
    (16, "E15", "Reference Electrode"),
    (17, "E16", "Reference Electrode"),
    (18, "E17", "Reference Electrode"),
    (19, "E18", "Reference Electrode"),
    (20, "E19", "Reference Electrode"),
    (21, "E20", "Reference Electrode"),
    (22, "E21", "Reference Electrode"),
    (23, "E22", "Reference Electrode"),
    (24, "E23", "Reference Electrode"),
]

@dataclass
class SweepRow:
    pc_time_epoch_s: float
    repeat_index: int
    step_index: int
    left_name: str
    right_name: str
    arduino_message: str

    settle_delay_s: float
    average_window_s: float

    n_i_samples: int

    i_mean_x: float
    i_mean_y: float
    i_mean_mag: float

    ui_vrms_v: float
    ui_impedance_ohm: Optional[float]

 
####### CSV processing helper functions #######

# Increment file name if user-selected name exists
def get_next_filename(base_name: str) -> str:
    if not os.path.exists(base_name):
        return base_name

    stem, ext = os.path.splitext(base_name)
    i = 2
    while True:
        candidate = f"{stem}{i}{ext}"
        if not os.path.exists(candidate):
            return candidate
        i += 1

def make_header_fieldnames():
    dummy = SweepRow(
        pc_time_epoch_s=0.0,
        repeat_index=0,
        step_index=0,
        left_name="",
        right_name="",
        arduino_message="",
        settle_delay_s=0.0,
        average_window_s=0.0,
        n_i_samples=0,
        i_mean_x=0.0,
        i_mean_y=0.0,
        i_mean_mag=0.0,
        ui_vrms_v=0.0,
        ui_impedance_ohm=0.0,
    )
    return list(asdict(dummy).keys())

def init_csv(path: str) -> str:
    out_path = get_next_filename(path)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=make_header_fieldnames())
        writer.writeheader()
        f.flush()
        os.fsync(f.fileno())
    return out_path

def append_rows_to_csv(rows: List[SweepRow], path: str):
    if not rows:
        return

    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=make_header_fieldnames())
        for row in rows:
            writer.writerow(asdict(row))
        f.flush()
        os.fsync(f.fileno())

# used for formatting xi and yi during polling
def as_list(v):
    try:
        return list(v)
    except Exception:
        return [v]

def format_hms(seconds: float) -> str:
    seconds = max(0.0, seconds)
    total = int(round(seconds))
    h = total // 3600
    m = (total % 3600) // 60
    s = total % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


################ Arduino Communication ################

class ArduinoMuxController:
    def __init__(self, port: str, baud: int, timeout: float):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        time.sleep(SERIAL_STARTUP_WAIT_S)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._flush_boot_messages()

    def _flush_boot_messages(self):
        end_time = time.time() + 1.0
        while time.time() < end_time:
            line = self.ser.readline().decode(errors="replace").strip()
            if not line:
                break
            print(f"[Arduino boot] {line}")

    def send_reset(self) -> str:
        self.ser.write(b"r")
        self.ser.flush()
        return self.ser.readline().decode(errors="replace").strip()

    def send_next(self) -> str:
        self.ser.write(b"n")
        self.ser.flush()
        return self.ser.readline().decode(errors="replace").strip()

    def close(self):
        if self.ser.is_open:
            self.ser.close()


################ MFLI Communication ################

def discover_mfli(device_id: str) -> dict:
    d = zhinst.core.ziDiscovery()
    props = d.get(d.find(device_id))

    print("Discovered MFLI:")
    print(f"  device id:      {device_id}")
    print(f"  serveraddress:  {props['serveraddress']}")
    print(f"  serverport:     {props['serverport']}")
    print(f"  interfaces:     {props['interfaces']}")

    return props


class MFLIController:
    def __init__(self, device_id: str, api_level: int):
        self.device_id = device_id

        props = discover_mfli(device_id)

        self.server_host = props["serveraddress"]
        self.server_port = int(props["serverport"])

        interfaces = props.get("interfaces", [])
        if not interfaces:
            raise RuntimeError("No interfaces reported by LabOne discovery for this device.")

        self.interface = interfaces[0]

        self.daq = zhinst.core.ziDAQServer(
            self.server_host,
            self.server_port,
            api_level,
        )

        self.daq.connectDevice(device_id, self.interface)
        self.path_i = f"/{device_id}/demods/{DEMOD_I}/sample"

        print("Connected to MFLI:")
        print(f"  host:           {self.server_host}")
        print(f"  port:           {self.server_port}")
        print(f"  interface:      {self.interface}")

    def config_run(self):
        dev = self.device_id
        daq = self.daq

        daq.setDouble(f"/{dev}/currins/0/range", CURRIN_RANGE_A)
        daq.setInt(f"/{dev}/sigins/0/ac", 0)
        daq.setDouble(f"/{dev}/sigins/0/range", SIGIN_RANGE_V)

        daq.setDouble(f"/{dev}/oscs/{OSC_INDEX}/freq", OSC_FREQ_HZ)

        # Demod 1: current input
        daq.setInt(f"/{dev}/demods/{DEMOD_I}/adcselect", 1)
        daq.setInt(f"/{dev}/demods/{DEMOD_I}/oscselect", OSC_INDEX)
        daq.setInt(f"/{dev}/demods/{DEMOD_I}/harmonic", DEMOD_HARMONIC)
        daq.setDouble(f"/{dev}/demods/{DEMOD_I}/phaseshift", DEMOD_PHASESHIFT_DEG)
        daq.setInt(f"/{dev}/demods/{DEMOD_I}/order", DEMOD_ORDER)
        daq.setInt(f"/{dev}/demods/{DEMOD_I}/sinc", 0)
        daq.setDouble(f"/{dev}/demods/{DEMOD_I}/timeconstant", DEMOD_I_TIMECONSTANT_S)
        daq.setDouble(f"/{dev}/demods/{DEMOD_I}/rate", DEMOD_RATE_SA_S)
        daq.setInt(f"/{dev}/demods/{DEMOD_I}/enable", 1)

        # Signal Output 1
        daq.setDouble(f"/{dev}/sigouts/{SIGOUT}/range", SIGOUT_RANGE_V)
        daq.setDouble(f"/{dev}/sigouts/{SIGOUT}/offset", SIGOUT_OFFSET_V)
        daq.setInt(f"/{dev}/sigouts/{SIGOUT}/add", 0)
        daq.setInt(f"/{dev}/sigouts/{SIGOUT}/diff", 0)
        daq.setInt(f"/{dev}/sigouts/{SIGOUT}/on", 1)

        print("Demod readback:")
        print("  demod0 enable =", daq.getInt(f"/{dev}/demods/{DEMOD_I}/enable"))
        print("  demod1 enable =", daq.getInt(f"/{dev}/demods/{DEMOD_V}/enable"))
        print("  demod0 rate   =", daq.getDouble(f"/{dev}/demods/{DEMOD_I}/rate"))
        print("  demod1 rate   =", daq.getDouble(f"/{dev}/demods/{DEMOD_V}/rate"))
        print("  demod0 adcsel =", daq.getInt(f"/{dev}/demods/{DEMOD_I}/adcselect"))
        print("  demod1 adcsel =", daq.getInt(f"/{dev}/demods/{DEMOD_V}/adcselect"))

        daq.sync()

    def poll_means(self, duration_s: float):
        daq = self.daq
        path_i = self.path_i

        daq.subscribe(path_i)
        daq.poll(0.0, POLL_TIMEOUT_MS, 0, True)
        time.sleep(0.05)
        data = daq.poll(duration_s, POLL_TIMEOUT_MS, 0, True)
        daq.unsubscribe(path_i)

        lowered = {k.lower(): v for k, v in data.items()}

        if path_i.lower() not in lowered:
            print("poll() returned keys:")
            for k in data.keys():
                print("  ", k)
            raise RuntimeError("Did not receive data from demodulator 0.")

        di = lowered[path_i.lower()]

        xi = as_list(di["x"])
        yi = as_list(di["y"])

        if len(xi) == 0:
            raise RuntimeError("Received demod node but no samples.")

        i_mean_x = mean(xi)
        i_mean_y = mean(yi)

        return {
            "n_i": len(xi),
            "i_mean_x": i_mean_x,
            "i_mean_y": i_mean_y,
            "i_mean_mag": math.hypot(i_mean_x, i_mean_y),
        }

    def close(self):
        pass


################ Impedance Calculation ################
def ui_style_impedance(i_avg_amp: float, vrms: float) -> Optional[float]:
    if i_avg_amp == 0:
        return None
    return vrms / i_avg_amp


################ Scan Through Steps ################
def scan_pair_step_order(
    arduino: ArduinoMuxController,
    mfli: MFLIController,
    repeat_index: int,
) -> List[SweepRow]:
    print(f"\n========== FULL SCAN {repeat_index + 1} ==========")

    reset_msg = arduino.send_reset()
    print(f"Arduino reset: {reset_msg}")

    scan_rows: List[SweepRow] = []

    for idx, (step_index, left_name, right_name) in enumerate(PAIR_STEP_ORDER):
        if idx == 0:
            arduino_msg = reset_msg
        else:
            arduino_msg = arduino.send_next()

        time.sleep(SETTLE_DELAY_S)

        means = mfli.poll_means(AVERAGE_WINDOW_S)
        z_ui = ui_style_impedance(means["i_mean_mag"], SIGOUT_VRMS_V)

        row = SweepRow(
            pc_time_epoch_s=time.time(),
            repeat_index=repeat_index,
            step_index=step_index,
            left_name=left_name,
            right_name=right_name,
            arduino_message=arduino_msg,
            settle_delay_s=SETTLE_DELAY_S,
            average_window_s=AVERAGE_WINDOW_S,
            n_i_samples=means["n_i"],
            i_mean_x=means["i_mean_x"],
            i_mean_y=means["i_mean_y"],
            i_mean_mag=means["i_mean_mag"],
            ui_vrms_v=SIGOUT_VRMS_V,
            ui_impedance_ohm=z_ui,
        )

        scan_rows.append(row)

        print(
            f"step={step_index:02d}  "
            f"Iavg={row.i_mean_mag:.6g} A  "
            f"Z_ui={row.ui_impedance_ohm if row.ui_impedance_ohm is not None else float('nan'):.6g} ohm"
        )

    return scan_rows


################ Main Timed Loop ################

def run_impedance_sweep_timed(csv_path: str) -> List[SweepRow]:
    if SCAN_INTERVAL_S <= 0:
        raise ValueError("SCAN_INTERVAL_S must be > 0.")
    if TOTAL_RUN_TIME_S <= 0:
        raise ValueError("TOTAL_RUN_TIME_S must be > 0.")

    arduino = ArduinoMuxController(
        port=ARDUINO_PORT,
        baud=ARDUINO_BAUD,
        timeout=SERIAL_TIMEOUT_S,
    )
    mfli = MFLIController(
        device_id=DEVICE_ID,
        api_level=API_LEVEL,
    )

    all_rows: List[SweepRow] = []
    out_path = init_csv(csv_path)

    try:
        mfli.config_run()
        print("MFLI configured.")
        time.sleep(0.2)

        run_start_time = time.time()
        run_end_time = run_start_time + TOTAL_RUN_TIME_S
        next_scan_start = run_start_time
        scan_index = 0

        print(f"\nTimed run started at epoch {run_start_time:.3f}")
        print(f"Total run time: {format_hms(TOTAL_RUN_TIME_S)}")
        print(f"Scan interval:  {SCAN_INTERVAL_S:.1f} s")
        print("A new scan is scheduled from the previous scheduled start time,")
        print("so the scan duration counts inside the minute.")

        while next_scan_start <= run_end_time:
            now = time.time()

            # Wait until the scheduled start time
            wait_s = next_scan_start - now
            if wait_s > 0:
                print(
                    f"\nWaiting {wait_s:.2f} s for next scheduled scan start "
                    f"(elapsed {format_hms(now - run_start_time)} / {format_hms(TOTAL_RUN_TIME_S)})"
                )
                time.sleep(wait_s)

            actual_scan_start = time.time()
            elapsed_before = actual_scan_start - run_start_time

            if actual_scan_start > run_end_time:
                break

            print(
                f"\nStarting scan {scan_index + 1} at "
                f"elapsed {format_hms(elapsed_before)} "
                f"(scheduled {format_hms(next_scan_start - run_start_time)})"
            )

            scan_rows = scan_pair_step_order(
                arduino=arduino,
                mfli=mfli,
                repeat_index=scan_index,
            )

            append_rows_to_csv(scan_rows, out_path)
            all_rows.extend(scan_rows)

            actual_scan_end = time.time()
            scan_duration = actual_scan_end - actual_scan_start
            elapsed_after = actual_scan_end - run_start_time

            print(
                f"Saved scan {scan_index + 1} to {out_path} | "
                f"scan duration = {scan_duration:.2f} s | "
                f"elapsed total = {format_hms(elapsed_after)}"
            )

            # Advance by the fixed schedule interval from the prior scheduled start,
            # NOT from the end of the scan.
            next_scan_start += SCAN_INTERVAL_S
            scan_index += 1

            lag_s = time.time() - next_scan_start
            if lag_s > 0:
                print(
                    f"Warning: scan took longer than the scheduled interval by {lag_s:.2f} s. "
                    "Next scan will start immediately to catch up."
                )

        print(f"\nFinished timed run. Wrote {len(all_rows)} total rows to {out_path}")
        return all_rows

    finally:
        try:
            arduino.close()
        finally:
            mfli.close()


if __name__ == "__main__":
    rows = run_impedance_sweep_timed(CSV_PATH)