#!/usr/bin/env python3
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("Error: pyserial is required. Install with 'pip install pyserial'.")
    sys.exit(1)

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

EXPECTED_OUTPUT_PATTERNS = [
    b"CO2:",
    b"Concentration:",
    b"Sensor's Device ID",
    b"Current ABC time",
    b"Calibration command sent"
]

ERROR_OUTPUT_PATTERNS = [
    b"Error:",
    b"Failed",
    b"Communication error",
    b"Attempted again",
]

TEST_MATRIX = {
    "examples/sunrise_single": {
        "protocols": ["i2c", "modbus"],
        "destructive": False,
        "timeout": 30,
        "mode": "single",
        "variants": [
            {"name": "default", "extra_flags": [
                "-DCHANGE_MEASUREMENT_CONFIGURATION=1",
                "-DCHANGE_EXTENDED_CONFIGURATION=1",
            ]},
            {"name": "simple", "extra_flags": []},
        ],
    },
    "examples/s12_single": {
        "protocols": ["i2c", "modbus"],
        "destructive": False,
        "timeout": 30,
        "mode": "single",
        "variants": [
            {"name": "default", "extra_flags": [
                "-DCHANGE_MEASUREMENT_CONFIGURATION=1",
                "-DCHANGE_EXTENDED_CONFIGURATION=1",
            ]},
            {"name": "simple", "extra_flags": []},
        ],
    },
    "examples/sunrise_continuous": {
        "protocols": ["i2c", "modbus"],
        "destructive": False,
        "timeout": 30,
        "mode": "continuous",
        "variants": [
            {"name": "default", "extra_flags": [
                "-DCHANGE_MEASUREMENT_CONFIGURATION=1",
                "-DCHANGE_EXTENDED_CONFIGURATION=1",
            ]},
            {"name": "simple", "extra_flags": []},
        ],
    },
    "examples/s12_continuous": {
        "protocols": ["i2c", "modbus"],
        "destructive": False,
        "timeout": 30,
        "mode": "continuous",
        "variants": [
            {"name": "default", "extra_flags": [
                "-DCHANGE_MEASUREMENT_CONFIGURATION=1",
                "-DCHANGE_EXTENDED_CONFIGURATION=1",
            ]},
            {"name": "simple", "extra_flags": []},
        ],
    },
    "examples/sunrise_scale_factor": {
        "protocols": ["i2c", "modbus"],
        "destructive": False,
        "timeout": 30,
        "mode": "continuous",
        "variants": [
            {"name": "default", "extra_flags": [
                "-DCHANGE_SCALE_FACTOR=1",
            ]},
            {"name": "simple", "extra_flags": []},
        ],
    },
    "examples/sunrise_i2c_change_address": {
        "protocols": ["i2c"],
        "destructive": True,
        "timeout": 20,
        "mode": None,
        "variants": [{"name": "default", "extra_flags": []}],
    },
    "examples/sunrise_modbus_change_address": {
        "protocols": ["modbus"],
        "destructive": True,
        "timeout": 20,
        "mode": None,
        "variants": [{"name": "default", "extra_flags": []}],
    },
}


def run_command(command, capture_output=True, text=True):
    result = subprocess.run(command, capture_output=capture_output, text=text)
    if result.returncode != 0:
        raise RuntimeError(
            f"Command failed: {' '.join(command)}\nexit code: {result.returncode}\nstdout: {result.stdout}\nstderr: {result.stderr}"
        )
    return result.stdout.strip()


def detect_boards():
    output = run_command(["arduino-cli", "board", "list", "--format", "json"])
    try:
        data = json.loads(output)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Failed to parse arduino-cli JSON output: {exc}\n{output}")

    ports = data.get("detected_ports", []) if isinstance(data, dict) else data
    results = []
    for port in ports:
        matching = port.get("matching_boards")
        if matching is None:
            continue

        fqbn = matching[0].get("fullyQualifiedName") or matching[0].get("fqbn")
        address = port.get("port").get("address")
        print(f"Detected board: FQBN={fqbn}, port={address}")
        if fqbn and port:
            results.append({"fqbn": fqbn, "port": address, "board": port})
    return results


def compile_example(example_path: Path, fqbn: str, protocol: str = None, extra_flags: list = None):
    flags = []
    if protocol == "i2c":
        flags.append("-DSUNRISE_PROTOCOL_I2C")
    elif protocol == "modbus":
        flags.append("-DSUNRISE_PROTOCOL_MODBUS")
    if extra_flags:
        flags.extend(extra_flags)

    cmd = ["arduino-cli", "compile", "--libraries", str(REPO_ROOT), "--fqbn", fqbn, str(example_path)]
    if flags:
        cmd.extend(["--build-property", f"build.extra_flags={' '.join(flags)}"])

    protocol_str = protocol or "default"
    flags_str = f" [{' '.join(flags)}]" if flags else ""
    print(f"Compiling '{example_path.name}' ({protocol_str}){flags_str} for {fqbn}...")
    run_command(cmd)
    print("Compile succeeded.")


def upload_example(example_path: Path, fqbn: str, port: str):
    print(f"Uploading '{example_path.name}' to {port} ({fqbn})...")
    run_command(["arduino-cli", "upload", "-p", port, "--fqbn", fqbn, str(example_path)])
    print("Upload succeeded.")


def verify_serial(port: str, timeout: int):
    print(f"Opening serial port {port} and waiting for output...")
    result = {
        'success': False,
        'errors': [],
        'output': []
    }

    with serial.Serial(port, 115200, timeout=1) as ser:
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            try:
                line = ser.readline()
            except serial.SerialException as exc:
                raise RuntimeError(f"Serial error on {port}: {exc}")
            if not line:
                continue

            output_str = line.decode(errors="replace").rstrip()
            print(output_str)
            result['output'].append(output_str)

            if any(pattern in line for pattern in EXPECTED_OUTPUT_PATTERNS):
                result['success'] = True

            for error_pattern in ERROR_OUTPUT_PATTERNS:
                if error_pattern in line:
                    result['errors'].append(output_str)

    return result


def build_test_plan(args):
    """Build an ordered list of (example, protocol, variant) tuples to run."""
    plan = []

    if args.all:
        protocols = ["i2c", "modbus"] if args.protocol == "both" else [args.protocol]

        for example_name, config in TEST_MATRIX.items():
            if config["destructive"] and not args.include_destructive:
                continue

            variants = config["variants"] if args.variant == "all" else [config["variants"][0]]

            for protocol in protocols:
                if protocol not in config["protocols"]:
                    continue
                for variant in variants:
                    plan.append({
                        "example": example_name,
                        "protocol": protocol,
                        "variant": variant,
                        "timeout": config["timeout"],
                    })
    else:
        example_name = args.example
        config = TEST_MATRIX.get(example_name)
        protocols = ["i2c", "modbus"] if args.protocol == "both" else [args.protocol]

        if config:
            variants = config["variants"] if args.variant == "all" else [config["variants"][0]]
            timeout = config["timeout"]
        else:
            variants = [{"name": "default", "extra_flags": []}]
            timeout = args.timeout

        for protocol in protocols:
            if config and protocol not in config["protocols"]:
                continue
            for variant in variants:
                plan.append({
                    "example": example_name,
                    "protocol": protocol,
                    "variant": variant,
                    "timeout": timeout,
                })

    return plan


def print_test_plan(plan):
    print("\n=== Test Plan ===")
    for i, entry in enumerate(plan, 1):
        variant_name = entry["variant"]["name"]
        flags = entry["variant"]["extra_flags"]
        flags_str = f"  flags: {' '.join(flags)}" if flags else ""
        print(f"  {i}. {entry['example']} ({entry['protocol']}, {variant_name}){flags_str}")
    print(f"\nTotal: {len(plan)} test(s)")


def run_single_test(entry, fqbn, port, no_verify, strict, allow_errors):
    """Run a single test case. Returns a result dict."""
    example_name = entry["example"]
    protocol = entry["protocol"]
    variant = entry["variant"]
    timeout = entry["timeout"]

    example_path = REPO_ROOT / example_name
    if not example_path.exists():
        return {"status": "SKIP", "reason": f"Path not found: {example_path}"}

    label = f"{example_name} ({protocol}, {variant['name']})"
    print(f"\n{'='*60}")
    print(f"TEST: {label}")
    print(f"{'='*60}")

    try:
        compile_example(example_path, fqbn, protocol, variant["extra_flags"])
    except RuntimeError as exc:
        return {"status": "FAIL", "reason": f"Compile error: {exc}"}

    try:
        upload_example(example_path, fqbn, port)
    except RuntimeError as exc:
        return {"status": "FAIL", "reason": f"Upload error: {exc}"}

    if no_verify:
        return {"status": "PASS", "reason": "Upload OK (verification skipped)"}

    print("Waiting 5 seconds for the board to restart...")
    time.sleep(5)

    try:
        verify_result = verify_serial(port, timeout)
    except RuntimeError as exc:
        return {"status": "FAIL", "reason": f"Serial error: {exc}"}

    if not verify_result['success']:
        return {"status": "FAIL", "reason": f"No expected output within {timeout}s"}

    if verify_result['errors']:
        error_count = len(verify_result['errors'])
        error_lines = "; ".join(verify_result['errors'][:3])
        if strict or not allow_errors:
            return {"status": "FAIL", "reason": f"{error_count} error(s): {error_lines}"}
        else:
            return {"status": "WARN", "reason": f"{error_count} error(s): {error_lines}"}

    return {"status": "PASS", "reason": "OK"}


def main():
    parser = argparse.ArgumentParser(
        description="Compile, upload, and verify Arduino examples for connected boards.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  %(prog)s --list                                    List connected boards
  %(prog)s --example examples/sunrise_single         Test single example (default protocol)
  %(prog)s --example examples/sunrise_single --protocol i2c
  %(prog)s --all --protocol both --variant all       Full test matrix
  %(prog)s --all --protocol modbus                   All examples, Modbus only, default variant
        """,
    )
    parser.add_argument("--example", default="examples/sunrise_single",
                        help="Path to the example sketch directory (default: examples/sunrise_single)")
    parser.add_argument("--fqbn", help="Arduino board FQBN to compile/upload for")
    parser.add_argument("--port", help="Serial port to upload to and verify")
    parser.add_argument("--timeout", type=int, default=20,
                        help="Seconds to wait for serial output (default for unlisted examples)")
    parser.add_argument("--list", action="store_true", help="List connected Arduino boards and exit")
    parser.add_argument("--no-verify", action="store_true", help="Skip serial output verification after upload")
    parser.add_argument("--strict", action="store_true",
                        help="Fail if any error patterns are detected in sketch output")
    parser.add_argument("--allow-errors", action="store_true",
                        help="Report detected errors but do not fail")
    parser.add_argument("--protocol", choices=["i2c", "modbus", "both"], default="modbus",
                        help="Protocol(s) to test (default: modbus)")
    parser.add_argument("--all", action="store_true",
                        help="Run all non-destructive examples for the selected protocol(s)")
    parser.add_argument("--include-destructive", action="store_true",
                        help="Include destructive examples (e.g. address change) when using --all")
    parser.add_argument("--variant", choices=["default", "all"], default="default",
                        help="Build variant(s) to test: 'default' or 'all' (default: default)")
    args = parser.parse_args()

    boards = detect_boards()
    if args.list:
        if not boards:
            print("No Arduino boards detected.")
            return
        for entry in boards:
            print(f"{entry['port']} -> {entry['fqbn']}")

        if args.all or args.protocol != "modbus" or args.variant != "default":
            plan = build_test_plan(args)
            print_test_plan(plan)
        return

    if not boards:
        raise RuntimeError("No connected Arduino boards detected.")

    selected = boards[0]
    if args.fqbn:
        selected = next((entry for entry in boards if entry["fqbn"] == args.fqbn), selected)
    if args.port:
        selected = next((entry for entry in boards if entry["port"] == args.port), selected)

    fqbn = selected["fqbn"]
    port = selected["port"]

    plan = build_test_plan(args)
    if not plan:
        raise RuntimeError("No tests match the given filters.")

    print_test_plan(plan)

    results = []
    for entry in plan:
        result = run_single_test(entry, fqbn, port, args.no_verify, args.strict, args.allow_errors)
        results.append({"entry": entry, **result})

    # Print summary
    print(f"\n{'='*60}")
    print("=== Test Summary ===")
    print(f"{'='*60}")

    max_label_len = max(
        len(f"{r['entry']['example']} ({r['entry']['protocol']}, {r['entry']['variant']['name']})")
        for r in results
    )

    pass_count = 0
    fail_count = 0
    for r in results:
        entry = r["entry"]
        label = f"{entry['example']} ({entry['protocol']}, {entry['variant']['name']})"
        status = r["status"]
        reason = r.get("reason", "")

        if status == "PASS":
            status_str = "PASS"
            pass_count += 1
        elif status == "WARN":
            status_str = f"WARN - {reason}"
            pass_count += 1
        elif status == "SKIP":
            status_str = f"SKIP - {reason}"
        else:
            status_str = f"FAIL - {reason}"
            fail_count += 1

        print(f"  {label:<{max_label_len}}  {status_str}")

    total = len(results)
    print(f"\nTotal: {pass_count}/{total} passed", end="")
    if fail_count:
        print(f", {fail_count} failed")
    else:
        print()

    if fail_count:
        sys.exit(1)
    print("\nAll tests passed.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"Error: {exc}")
        sys.exit(1)
