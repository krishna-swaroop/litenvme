#!/usr/bin/env python3
"""Run the complete pytest suite with bounded collection and process-group cleanup."""

from __future__ import annotations

import argparse
import datetime as _datetime
import json
import math
import os
from pathlib import Path, PurePosixPath
import selectors
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPORTS = ROOT / "build" / "ci"
COLLECTION_TIMEOUT_SECONDS = 60.0
DEFAULT_TIMEOUT_SECONDS = 5400.0
DEFAULT_KILL_GRACE_SECONDS = 5.0
OWNED_OUTPUTS = (
    "collection.log",
    "pytest.log",
    "pytest.xml",
    "runner-error.xml",
    "result.json",
)


class RunnerError(RuntimeError):
    """A collection, JUnit, or runner precondition failed."""

    def __init__(self, message: str, counts: dict | None = None):
        super().__init__(message)
        self.counts = counts


def _utc_now() -> str:
    return _datetime.datetime.now(_datetime.timezone.utc).isoformat()


def _group_exists(pgid: int) -> bool:
    if os.name != "posix":
        return False
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _signal_group(pgid: int, sig: int) -> bool:
    if os.name != "posix":
        return False
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return False
    return True


def run_process(
    argv: list[str] | tuple[str, ...],
    log: str | Path,
    timeout: float,
    *,
    cwd: str | Path | None = None,
    env: dict[str, str] | None = None,
    kill_grace_seconds: float = DEFAULT_KILL_GRACE_SECONDS,
    echo: bool = True,
) -> dict:
    """Stream a child process to a combined log and enforce a process-group timeout.

    On POSIX the child starts a new session. On timeout, TERM is sent to its process
    group, followed by KILL after ``kill_grace_seconds`` if anything remains. Output
    already emitted remains in ``log`` on success, failure, and timeout.
    """
    if not isinstance(argv, (list, tuple)) or not argv or not all(isinstance(a, str) for a in argv):
        raise TypeError("argv must be a non-empty list or tuple of strings")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be a positive finite number")
    if not math.isfinite(kill_grace_seconds) or kill_grace_seconds < 0:
        raise ValueError("kill_grace_seconds must be a non-negative finite number")

    log_path = Path(log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command_env = os.environ.copy()
    command_env["PYTHONUNBUFFERED"] = "1"
    if env:
        command_env.update(env)

    result = {
        "argv": list(argv),
        "cwd": str(Path(cwd).resolve()) if cwd is not None else str(Path.cwd()),
        "log": str(log_path.resolve()),
        "timeout_seconds": timeout,
        "kill_grace_seconds": kill_grace_seconds,
        "started_at": _utc_now(),
        "returncode": None,
        "timed_out": False,
        "killed": False,
        "orphaned": False,
        "launch_error": None,
    }
    started = time.monotonic()

    with log_path.open("wb") as log_file:
        try:
            popen_options = {
                "cwd": str(cwd) if cwd is not None else None,
                "env": command_env,
                "stdout": subprocess.PIPE,
                "stderr": subprocess.STDOUT,
                "bufsize": 0,
            }
            if os.name == "posix":
                popen_options["start_new_session"] = True
            else:
                popen_options["creationflags"] = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
            process = subprocess.Popen(list(argv), **popen_options)
        except OSError as exc:
            result["launch_error"] = f"{type(exc).__name__}: {exc}"
            payload = (result["launch_error"] + "\n").encode("utf-8", "replace")
            log_file.write(payload)
            log_file.flush()
            if echo:
                _write_console(payload)
            result["duration_seconds"] = round(time.monotonic() - started, 3)
            result["finished_at"] = _utc_now()
            return result

        result["process_id"] = process.pid
        pgid = process.pid if os.name == "posix" else None
        selector = selectors.DefaultSelector()
        assert process.stdout is not None
        selector.register(process.stdout, selectors.EVENT_READ)
        deadline = started + timeout

        def pump(wait_seconds: float) -> None:
            for key, _ in selector.select(max(0.0, wait_seconds)):
                try:
                    chunk = os.read(key.fileobj.fileno(), 65536)
                except BlockingIOError:
                    continue
                if not chunk:
                    selector.unregister(key.fileobj)
                    key.fileobj.close()
                    continue
                log_file.write(chunk)
                log_file.flush()
                if echo:
                    _write_console(chunk)

        try:
            while selector.get_map() or process.poll() is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    result["timed_out"] = True
                    if pgid is not None:
                        _signal_group(pgid, signal.SIGTERM)
                        grace_deadline = time.monotonic() + kill_grace_seconds
                        while time.monotonic() < grace_deadline:
                            pump(min(0.05, grace_deadline - time.monotonic()))
                            process.poll()
                            if not _group_exists(pgid):
                                break
                        if _group_exists(pgid):
                            result["killed"] = _signal_group(pgid, signal.SIGKILL)
                    else:
                        process.terminate()
                        try:
                            process.wait(timeout=kill_grace_seconds)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            result["killed"] = True
                    try:
                        process.wait(timeout=max(1.0, kill_grace_seconds + 0.5))
                    except subprocess.TimeoutExpired:
                        if pgid is not None:
                            result["killed"] = _signal_group(pgid, signal.SIGKILL) or result["killed"]
                        else:
                            process.kill()
                            result["killed"] = True
                        process.wait()

                    # Reap remaining pipe data after the process group is gone. Keep this
                    # drain bounded in case a child escaped into another session.
                    drain_deadline = time.monotonic() + 1.0
                    while selector.get_map() and time.monotonic() < drain_deadline:
                        pump(0.05)
                    for key in list(selector.get_map().values()):
                        selector.unregister(key.fileobj)
                        key.fileobj.close()
                    break

                pump(min(0.1, remaining))

            result["returncode"] = process.wait()

            # A command should not leave detached workers in its session after the leader
            # exits. Clean them up without changing the leader's status.
            if pgid is not None and _group_exists(pgid):
                result["orphaned"] = True
                _signal_group(pgid, signal.SIGTERM)
                grace_deadline = time.monotonic() + kill_grace_seconds
                while time.monotonic() < grace_deadline and _group_exists(pgid):
                    time.sleep(min(0.05, grace_deadline - time.monotonic()))
                if _group_exists(pgid):
                    result["killed"] = _signal_group(pgid, signal.SIGKILL) or result["killed"]
        finally:
            selector.close()
            if process.stdout is not None and not process.stdout.closed:
                process.stdout.close()

    result["duration_seconds"] = round(time.monotonic() - started, 3)
    result["finished_at"] = _utc_now()
    return result


def _write_console(payload: bytes) -> None:
    """Write child bytes immediately to stdout, including under text redirection."""
    output = getattr(sys.stdout, "buffer", None)
    if output is not None:
        output.write(payload)
        output.flush()
    else:
        sys.stdout.write(payload.decode("utf-8", "replace"))
        sys.stdout.flush()


def parse_collected_nodeids(log_path: str | Path) -> list[str]:
    """Extract pytest's collected ``test/...::...`` node IDs from quiet output."""
    nodeids = []
    for raw_line in Path(log_path).read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if line.startswith("test/") and "::" in line:
            nodeids.append(line)
    if not nodeids:
        raise RunnerError("pytest collection produced no test/...::... node IDs")
    duplicates = sorted({nodeid for nodeid in nodeids if nodeids.count(nodeid) > 1})
    if duplicates:
        raise RunnerError(f"pytest collection produced duplicate node IDs: {duplicates}")
    return nodeids


def _expected_identity(nodeid: str) -> tuple[str, str]:
    components = nodeid.split("::")
    if len(components) < 2 or not components[-1]:
        raise RunnerError(f"invalid pytest node ID: {nodeid!r}")
    source = PurePosixPath(components[0])
    if source.is_absolute() or ".." in source.parts or not source.parts or source.parts[0] != "test":
        raise RunnerError(f"pytest node ID is outside test/: {nodeid!r}")
    module = source.with_suffix("")
    class_components = [*module.parts, *components[1:-1]]
    classname = ".".join(class_components)
    return classname, components[-1]


def validate_junit(junit_path: str | Path, expected_nodeids: list[str] | tuple[str, ...]) -> dict:
    """Require a fresh JUnit report matching every collected test exactly once."""
    path = Path(junit_path)
    if not path.is_file():
        raise RunnerError(f"JUnit report is missing: {path}")
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise RunnerError(f"JUnit report is invalid: {exc}") from exc

    def local_name(element: ET.Element) -> str:
        return element.tag.rsplit("}", 1)[-1]

    suites = [element for element in root.iter() if local_name(element) in ("testsuite", "testsuites")]
    testcases = [element for element in root.iter() if local_name(element) == "testcase"]
    if not testcases:
        raise RunnerError("JUnit report contains zero testcases", {
            "collected": len(expected_nodeids), "executed": 0, "passed": 0,
            "failures": 0, "errors": 0, "skipped": 0,
        })

    expected = {}
    for nodeid in expected_nodeids:
        identity = _expected_identity(nodeid)
        if identity in expected:
            raise RunnerError(f"collected test identities are ambiguous or duplicated: {identity}")
        expected[identity] = nodeid
    if not expected:
        raise RunnerError("no collected test identities were supplied for JUnit validation")

    seen = {}
    failure_count = 0
    error_count = 0
    skipped_count = 0
    for testcase in testcases:
        identity = (testcase.attrib.get("classname", ""), testcase.attrib.get("name", ""))
        for child in testcase.iter():
            kind = local_name(child)
            if kind == "failure":
                failure_count += 1
            elif kind == "error":
                error_count += 1
            elif kind == "skipped":
                skipped_count += 1
        if identity not in seen:
            seen[identity] = testcase

    suite_counts = {"failures": 0, "errors": 0, "skipped": 0}
    for suite in suites:
        for key in suite_counts:
            raw_value = suite.attrib.get(key)
            if raw_value is None:
                continue
            try:
                count = int(raw_value)
            except ValueError as exc:
                raise RunnerError(f"JUnit {local_name(suite)} has invalid {key} count {raw_value!r}") from exc
            if count < 0:
                raise RunnerError(f"JUnit {local_name(suite)} has negative {key} count")
            # Suite counts may be nested/aggregated. The maximum preserves a root-level
            # nonzero count without double-counting parent and child suites.
            suite_counts[key] = max(suite_counts[key], count)

    failures = max(failure_count, suite_counts["failures"])
    errors = max(error_count, suite_counts["errors"])
    skipped = max(skipped_count, suite_counts["skipped"])
    report_counts = {
        "collected": len(expected),
        "executed": len(testcases),
        "passed": max(0, len(testcases) - failures - errors - skipped),
        "failures": failures,
        "errors": errors,
        "skipped": skipped,
    }

    for testcase in testcases:
        identity = (testcase.attrib.get("classname", ""), testcase.attrib.get("name", ""))
        if identity in seen and seen[identity] is not testcase:
            raise RunnerError(f"JUnit report contains duplicate testcase identity: {identity}", report_counts)
        if identity not in expected:
            raise RunnerError(f"JUnit report contains unexpected testcase identity: {identity}", report_counts)

    missing = sorted(set(expected) - set(seen))
    if missing:
        missing_nodeids = [expected[item] for item in missing]
        raise RunnerError(f"JUnit report is missing collected tests: {missing_nodeids}", report_counts)
    if failures or errors or skipped:
        raise RunnerError(
            f"JUnit is not all-green: failures={failures}, errors={errors}, skipped={skipped}", report_counts
        )
    if len(testcases) != len(expected):
        # The exact set check above normally catches this; retain a direct cardinality guard.
        raise RunnerError(f"JUnit has {len(testcases)} tests, expected {len(expected)}", report_counts)

    report_counts["passed"] = len(testcases)
    report_counts["identities"] = [list(identity) for identity in seen]
    return report_counts


def _write_runner_error(reports: Path, message: str) -> None:
    root = ET.Element("testsuites", {"tests": "1", "failures": "0", "errors": "1", "skipped": "0"})
    suite = ET.SubElement(root, "testsuite", {
        "name": "qualification_runner", "tests": "1", "failures": "0", "errors": "1", "skipped": "0"
    })
    testcase = ET.SubElement(suite, "testcase", {"classname": "qualification_runner", "name": "harness"})
    error = ET.SubElement(testcase, "error", {"message": message})
    error.text = message
    ET.ElementTree(root).write(reports / "runner-error.xml", encoding="utf-8", xml_declaration=True)


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_ci_tests(reports: str | Path = DEFAULT_REPORTS, timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS) -> int:
    """Collect and run every test under ``test/``, saving a fail-closed report."""
    reports_path = Path(reports)
    if not reports_path.is_absolute():
        reports_path = ROOT / reports_path
    reports_path = reports_path.resolve()
    state = {
        "status": "failed",
        "started_at": _utc_now(),
        "reports_dir": str(reports_path),
        "timeout_seconds": timeout_seconds,
        "commands": [],
        "counts": {"collected": 0, "executed": 0, "passed": 0, "failures": 0, "errors": 0, "skipped": 0},
        "errors": [],
    }
    pytest_env = os.environ.copy()
    pytest_env.pop("PYTEST_ADDOPTS", None)
    pytest_env.pop("PYTEST_PLUGINS", None)
    # Disable third-party entry-point plugins so ambient user/site configuration
    # cannot alter the collected or executed full-suite test set.
    pytest_env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    try:
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise RunnerError("pytest timeout must be a positive finite number")
        reports_path.mkdir(parents=True, exist_ok=True)
        for filename in OWNED_OUTPUTS:
            output = reports_path / filename
            if output.is_file() or output.is_symlink():
                output.unlink()
            elif output.is_dir():
                raise RunnerError(f"runner-owned output path is a directory: {output}")

        collect_argv = [sys.executable, "-m", "pytest", "--collect-only", "-q", "test"]
        collect = run_process(
            collect_argv,
            reports_path / "collection.log",
            COLLECTION_TIMEOUT_SECONDS,
            cwd=ROOT,
            env=pytest_env,
        )
        collect["name"] = "pytest-collect"
        state["commands"].append(collect)
        if collect["timed_out"]:
            raise RunnerError(f"pytest collection exceeded {COLLECTION_TIMEOUT_SECONDS:g} seconds")
        if collect["orphaned"]:
            raise RunnerError("pytest collection left a child process running after its leader exited")
        if collect["launch_error"]:
            raise RunnerError(f"could not start pytest collection: {collect['launch_error']}")
        if collect["returncode"] != 0:
            raise RunnerError(f"pytest collection exited with status {collect['returncode']}")
        nodeids = parse_collected_nodeids(reports_path / "collection.log")
        state["counts"]["collected"] = len(nodeids)

        junit_path = reports_path / "pytest.xml"
        pytest_argv = [
            sys.executable, "-m", "pytest", "-v", "--tb=short", "--capture=tee-sys",
            "--durations=15", "-o", "faulthandler_timeout=600", f"--junitxml={junit_path}", "test",
        ]
        run = run_process(
            pytest_argv, reports_path / "pytest.log", timeout_seconds, cwd=ROOT, env=pytest_env
        )
        run["name"] = "pytest-suite"
        state["commands"].append(run)
        if run["timed_out"]:
            state["errors"].append(f"pytest suite exceeded {timeout_seconds:g} seconds")
        if run["orphaned"]:
            state["errors"].append("pytest suite left a child process running after its leader exited")
        if run["launch_error"]:
            state["errors"].append(f"could not start pytest suite: {run['launch_error']}")
        if run["returncode"] != 0:
            state["errors"].append(f"pytest suite exited with status {run['returncode']}")

        try:
            junit = validate_junit(junit_path, nodeids)
            state["counts"].update({key: junit[key] for key in state["counts"] if key in junit})
        except RunnerError as exc:
            if exc.counts:
                state["counts"].update({key: exc.counts[key] for key in state["counts"] if key in exc.counts})
            state["errors"].append(str(exc))
            _write_runner_error(reports_path, str(exc))

    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
        if message not in state["errors"]:
            state["errors"].append(message)
        if reports_path.exists() and not (reports_path / "runner-error.xml").exists():
            try:
                _write_runner_error(reports_path, message)
            except OSError as write_exc:
                state["errors"].append(f"could not write runner-error.xml: {write_exc}")
    finally:
        state["status"] = "passed" if not state["errors"] else "failed"
        state["finished_at"] = _utc_now()
        try:
            reports_path.mkdir(parents=True, exist_ok=True)
            _write_json(reports_path / "result.json", state)
        except OSError as exc:
            message = f"could not write result.json: {exc}"
            if message not in state["errors"]:
                state["errors"].append(message)
            state["status"] = "failed"
            print(f"qualification runner {message}", file=sys.stderr)

    if state["errors"]:
        for error in state["errors"]:
            print(f"qualification failed: {error}", file=sys.stderr)
        return 1
    print(f"qualification passed: {state['counts']['passed']} tests; reports: {reports_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reports", type=Path, default=DEFAULT_REPORTS, help="runner report directory")
    parser.add_argument(
        "--timeout-seconds", type=float, default=DEFAULT_TIMEOUT_SECONDS,
        help=f"whole-suite wall timeout (default: {DEFAULT_TIMEOUT_SECONDS:g} seconds)",
    )
    args = parser.parse_args(argv)
    return run_ci_tests(args.reports, args.timeout_seconds)


if __name__ == "__main__":
    raise SystemExit(main())
