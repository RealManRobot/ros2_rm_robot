#!/usr/bin/env python3

# Copyright 2026 realman-robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run Gazebo in its own process group and normalize requested shutdowns."""

import os
import signal
import subprocess
import sys


def normalized_return_code(returncode, requested_signal):
    """Treat a child terminated by our requested shutdown as a clean exit."""
    if requested_signal is None:
        return returncode

    signal_number = int(requested_signal)
    expected_codes = {0, -signal_number, 128 + signal_number}
    return 0 if returncode in expected_codes else returncode


def main(arguments=None):
    """Forward termination signals to Gazebo and wait for all cleanup."""
    command = list(sys.argv[1:] if arguments is None else arguments)
    if not command:
        print("gz_sim_clean_exit.py: a Gazebo command is required", file=sys.stderr)
        return 2

    child = subprocess.Popen(command, start_new_session=True)
    requested_signal = None

    def _forward_signal(signal_number, _frame):
        nonlocal requested_signal
        requested_signal = signal_number
        try:
            os.killpg(child.pid, signal_number)
        except ProcessLookupError:
            pass

    previous_handlers = {}
    for signal_number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous_handlers[signal_number] = signal.signal(
            signal_number,
            _forward_signal,
        )

    try:
        returncode = child.wait()
    finally:
        for signal_number, previous_handler in previous_handlers.items():
            signal.signal(signal_number, previous_handler)

    return normalized_return_code(returncode, requested_signal)


if __name__ == "__main__":
    sys.exit(main())
