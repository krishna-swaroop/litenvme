# Fork CI qualification

This fork's complete suite uses the immutable generator and transport dependencies
in `requirements-ci.lock`. They match the AXI-NVMe development gate at commit
`599ad73564f73c416b24b9f762ef68d0fe612991`, whose run `39585000331` passed all
106 LiteNVMe tests. LiteNVMe itself is always installed from the current checkout,
with dependency resolution disabled; the lock deliberately contains no LiteNVMe
wheel that could replace the code under review.

The previous workflow ran Python 3.9 and downloaded the moving LiteX setup script,
which installed upstream LiteX/LitePCIe 2026.8. It was different from the pinned
environment already qualifying this fork. Runs `39584563439` and `39584518318`
remained in `pytest -q test` for approximately five hours and were cancelled for
diagnosis. The released logs show a 72-test progress line out of 106 collected
tests. Buffered quiet output does not identify the exact hanging testcase, so
the logs alone do not establish a particular RTL failure or deadlock location.

The replacement retains the package/import/configuration/generator smoke checks
and the whole `test/` suite. It does not deselect slow tests. It verifies installed
dependency commit IDs and package versions, checks the current LiteNVMe import
path, and records those identities. The test runner records each test name and
uses pytest's fault-handler stack dump to aid diagnosis of long-running cases.
Its 90-minute process-group watchdog terminates a hung suite and child processes;
the surrounding job leaves time to upload evidence. A timeout is a failure.

Before reporting success, the runner compares JUnit testcase identities against
the collected suite and rejects empty, missing, duplicated, failed, errored or
skipped results. Watchdog and result-parser checks run separately, without RTL.
`build/ci/` retains progress logs, collected tests, JUnit, status, dependency
provenance, compiler identity and generator output, including on failure.

Feature branches run on pull requests rather than duplicate push/PR events.
Push checks still cover `main` and the `feature/stream-ring` integration branch.
Superseded runs for the same PR/ref are cancelled by workflow concurrency.

Reproduce in Python 3.12 with a fresh virtual environment:

```sh
python3 -m venv /tmp/litenvme-ci
/tmp/litenvme-ci/bin/python -m pip install -r .github/requirements-ci.lock
/tmp/litenvme-ci/bin/python -m pip install --no-deps --no-build-isolation -e .
/tmp/litenvme-ci/bin/python .github/scripts/check_ci_provenance.py
/tmp/litenvme-ci/bin/python -m unittest discover -s .github/scripts -p test_run_ci_tests.py -v
/tmp/litenvme-ci/bin/python .github/scripts/run_ci_tests.py
```

The complete Migen/C-driver simulation is intentionally left to cloud CI when
the local Mac is shared with other agents. Passing simulation is not hardware
timing, throughput or real-SSD qualification.
