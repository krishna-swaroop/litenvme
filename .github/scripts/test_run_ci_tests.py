"""Unit tests for the CI runner's process watchdog and JUnit validation."""

from __future__ import annotations

from pathlib import Path
import contextlib
import io
import os
import sys
import tempfile
import time
import unittest
from unittest import mock

import run_ci_tests as runner
from run_ci_tests import RunnerError, run_process, validate_junit


NODEIDS = [
    "test/test_sample.py::TestExample::test_parameterized[small]",
    "test/test_sample.py::test_module_level",
]


def _write_junit(path: Path, body: str, *, tests: int = 2, failures: int = 0,
                 errors: int = 0, skipped: int = 0) -> None:
    path.write_text(
        f'<testsuites tests="{tests}" failures="{failures}" errors="{errors}" skipped="{skipped}">'
        f'<testsuite name="pytest" tests="{tests}" failures="{failures}" errors="{errors}" skipped="{skipped}">'
        f"{body}</testsuite></testsuites>",
        encoding="utf-8",
    )


def _clean_cases() -> str:
    return (
        '<testcase classname="test.test_sample.TestExample" name="test_parameterized[small]" />'
        '<testcase classname="test.test_sample" name="test_module_level" />'
    )


class RunProcessTests(unittest.TestCase):
    def test_captures_stdout_and_stderr_in_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            log = Path(temporary) / "child.log"
            result = run_process(
                [sys.executable, "-c", "import sys; print('stdout-line'); print('stderr-line', file=sys.stderr)"],
                log,
                3.0,
                echo=False,
            )
            self.assertEqual(result["returncode"], 0)
            self.assertFalse(result["timed_out"])
            content = log.read_text(encoding="utf-8")
            self.assertIn("stdout-line", content)
            self.assertIn("stderr-line", content)

    def test_timeout_kills_term_ignoring_parent_and_descendant(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            marker = root / "heartbeat"
            pid_file = root / "descendant.pid"
            child_code = (
                "import pathlib,signal,sys,time; "
                "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                "p=pathlib.Path(sys.argv[1]); "
                "exec('while True:\\n p.write_text(str(time.monotonic_ns()))\\n time.sleep(0.01)')"
            )
            parent_code = (
                "import pathlib,signal,subprocess,sys,time; "
                "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                "child=subprocess.Popen([sys.executable,'-c',sys.argv[1],sys.argv[2]]); "
                "marker=pathlib.Path(sys.argv[2]); "
                "exec('while not marker.exists():\\n time.sleep(0.005)'); "
                "pathlib.Path(sys.argv[3]).write_text(str(child.pid)); "
                "print('descendant ready', flush=True); "
                "exec('while True:\\n time.sleep(1)')"
            )
            result = run_process(
                [sys.executable, "-c", parent_code, child_code, str(marker), str(pid_file)],
                root / "timeout.log",
                0.2,
                kill_grace_seconds=0.12,
                echo=False,
            )
            self.assertTrue(result["timed_out"])
            self.assertTrue(result["killed"])
            self.assertIsNotNone(result["returncode"])
            self.assertNotEqual(result["returncode"], 0)
            self.assertTrue(pid_file.exists(), "descendant should have started before timeout")
            first = marker.read_text(encoding="utf-8")
            time.sleep(0.06)
            second = marker.read_text(encoding="utf-8")
            self.assertEqual(first, second, "descendant kept running after timeout cleanup")

    def test_normal_exit_with_descendant_is_reported_and_cleaned_up(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            marker = root / "heartbeat"
            child_code = (
                "import pathlib,signal,sys,time; "
                "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
                "p=pathlib.Path(sys.argv[1]); "
                "exec('while True:\\n p.write_text(str(time.monotonic_ns()))\\n time.sleep(0.01)')"
            )
            parent_code = (
                "import pathlib,subprocess,sys,time; "
                "child=subprocess.Popen([sys.executable,'-c',sys.argv[1],sys.argv[2]], "
                "stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); "
                "marker=pathlib.Path(sys.argv[2]); "
                "exec('while not marker.exists():\\n time.sleep(0.005)'); "
                "print('leader exits successfully', flush=True)"
            )
            result = run_process(
                [sys.executable, "-c", parent_code, child_code, str(marker)],
                root / "orphan.log",
                2.0,
                kill_grace_seconds=0.1,
                echo=False,
            )
            self.assertEqual(result["returncode"], 0)
            self.assertFalse(result["timed_out"])
            self.assertTrue(result["orphaned"])
            self.assertTrue(result["killed"])
            first = marker.read_text(encoding="utf-8")
            time.sleep(0.05)
            self.assertEqual(first, marker.read_text(encoding="utf-8"))


class JUnitValidationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.junit = Path(self.temporary.name) / "pytest.xml"

    def tearDown(self):
        self.temporary.cleanup()

    def test_accepts_exact_clean_collected_identities(self):
        _write_junit(self.junit, _clean_cases())
        result = validate_junit(self.junit, NODEIDS)
        self.assertEqual(result["collected"], 2)
        self.assertEqual(result["executed"], 2)
        self.assertEqual(result["passed"], 2)
        self.assertEqual(result["failures"], 0)

    def test_rejects_missing_junit(self):
        with self.assertRaisesRegex(RunnerError, "missing"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_failed_case(self):
        body = _clean_cases().replace(
            'name="test_module_level" />',
            'name="test_module_level"><failure message="assertion failed" /></testcase>',
        )
        _write_junit(self.junit, body, failures=1)
        with self.assertRaisesRegex(RunnerError, "failures=1"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_error_case(self):
        body = _clean_cases().replace(
            'name="test_module_level" />',
            'name="test_module_level"><error message="runner error" /></testcase>',
        )
        _write_junit(self.junit, body, errors=1)
        with self.assertRaisesRegex(RunnerError, "errors=1"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_skipped_case(self):
        body = _clean_cases().replace(
            'name="test_module_level" />',
            'name="test_module_level"><skipped message="skip" /></testcase>',
        )
        _write_junit(self.junit, body, skipped=1)
        with self.assertRaisesRegex(RunnerError, "skipped=1"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_empty_junit(self):
        _write_junit(self.junit, "", tests=0)
        with self.assertRaisesRegex(RunnerError, "zero testcases"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_missing_collected_case(self):
        _write_junit(self.junit, _clean_cases().split('<testcase classname="test.test_sample"')[0], tests=1)
        with self.assertRaisesRegex(RunnerError, "missing collected tests"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_duplicate_case_identity(self):
        body = _clean_cases().split('<testcase classname="test.test_sample"')[0]
        duplicate = '<testcase classname="test.test_sample.TestExample" name="test_parameterized[small]" />'
        _write_junit(self.junit, body + duplicate, tests=2)
        with self.assertRaisesRegex(RunnerError, "duplicate testcase"):
            validate_junit(self.junit, [NODEIDS[0]])

    def test_rejects_root_failure_count_even_without_failure_node(self):
        _write_junit(self.junit, _clean_cases(), failures=1)
        with self.assertRaisesRegex(RunnerError, "failures=1"):
            validate_junit(self.junit, NODEIDS)

    def test_rejects_unexpected_identity(self):
        body = _clean_cases().replace("test_module_level", "test_different")
        _write_junit(self.junit, body)
        with self.assertRaisesRegex(RunnerError, "unexpected testcase"):
            validate_junit(self.junit, NODEIDS)


class RunCiTestsHarnessTests(unittest.TestCase):
    def test_result_report_write_failure_fails_run_and_ambient_pytest_options_are_removed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "test").mkdir()
            reports = root / "reports"
            observed_envs = []
            final_state = []
            real_write_json = runner._write_json

            def fake_run_process(argv, log, timeout, *, cwd=None, env=None, **kwargs):
                observed_envs.append(env)
                log_path = Path(log)
                log_path.parent.mkdir(parents=True, exist_ok=True)
                if "--collect-only" in argv:
                    log_path.write_text("test/test_sample.py::test_ok\n", encoding="utf-8")
                else:
                    junit_path = Path(next(arg.split("=", 1)[1] for arg in argv if arg.startswith("--junitxml=")))
                    _write_junit(
                        junit_path,
                        '<testcase classname="test.test_sample" name="test_ok" />',
                        tests=1,
                    )
                    log_path.write_text("1 passed\n", encoding="utf-8")
                return {
                    "argv": list(argv), "cwd": str(cwd), "log": str(log_path),
                    "timeout_seconds": timeout, "returncode": 0, "timed_out": False,
                    "killed": False, "orphaned": False, "launch_error": None,
                }

            def fail_result_json(path, state):
                if Path(path).name == "result.json":
                    final_state.append(state)
                    raise OSError("simulated report storage failure")
                return real_write_json(path, state)

            ambient = {
                "PYTEST_ADDOPTS": "-k definitely-not-the-suite",
                "PYTEST_PLUGINS": "ambient_plugin",
                "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "0",
            }
            with mock.patch.object(runner, "ROOT", root), \
                    mock.patch.object(runner, "run_process", side_effect=fake_run_process), \
                    mock.patch.object(runner, "_write_json", side_effect=fail_result_json), \
                    mock.patch.dict(os.environ, ambient, clear=False), \
                    contextlib.redirect_stderr(io.StringIO()):
                result = runner.run_ci_tests(reports, 2.0)

            self.assertEqual(result, 1)
            self.assertEqual(len(final_state), 1)
            self.assertEqual(final_state[0]["status"], "failed")
            self.assertTrue(any("could not write result.json" in item for item in final_state[0]["errors"]))
            self.assertEqual(len(observed_envs), 2)
            for env in observed_envs:
                self.assertNotIn("PYTEST_ADDOPTS", env)
                self.assertNotIn("PYTEST_PLUGINS", env)
                self.assertEqual(env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"], "1")

    def test_orphaned_or_unreported_suite_process_fails_with_clean_junit(self):
        scenarios = (
            ({"returncode": 0, "orphaned": True}, "left a child process running"),
            ({"returncode": None, "orphaned": False}, "exited with status None"),
        )
        for process_overrides, expected_error in scenarios:
            with self.subTest(process_overrides=process_overrides), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                (root / "test").mkdir()
                reports = root / "reports"

                def fake_run_process(argv, log, timeout, *, cwd=None, env=None, **kwargs):
                    log_path = Path(log)
                    log_path.parent.mkdir(parents=True, exist_ok=True)
                    suite = "--collect-only" not in argv
                    if not suite:
                        log_path.write_text("test/test_sample.py::test_ok\n", encoding="utf-8")
                    else:
                        junit_path = Path(next(
                            arg.split("=", 1)[1] for arg in argv if arg.startswith("--junitxml=")
                        ))
                        _write_junit(
                            junit_path,
                            '<testcase classname="test.test_sample" name="test_ok" />',
                            tests=1,
                        )
                        log_path.write_text("1 passed\n", encoding="utf-8")
                    result = {
                        "argv": list(argv), "cwd": str(cwd), "log": str(log_path),
                        "timeout_seconds": timeout, "returncode": 0, "timed_out": False,
                        "killed": False, "orphaned": False, "launch_error": None,
                    }
                    if suite:
                        result.update(process_overrides)
                    return result

                with mock.patch.object(runner, "ROOT", root), \
                        mock.patch.object(runner, "run_process", side_effect=fake_run_process), \
                        contextlib.redirect_stderr(io.StringIO()):
                    result = runner.run_ci_tests(reports, 2.0)

                self.assertEqual(result, 1)
                report = runner.json.loads((reports / "result.json").read_text(encoding="utf-8"))
                self.assertEqual(report["status"], "failed")
                self.assertTrue(any(expected_error in error for error in report["errors"]))


if __name__ == "__main__":
    unittest.main()
