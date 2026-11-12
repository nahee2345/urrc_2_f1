"""Regression for Ubuntu ROS 2 Jazzy setup with set -u in team11 test script."""
from pathlib import Path
import subprocess
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "test_team11_unified.sh"


class RosSetupScriptTests(unittest.TestCase):
    def test_bash_syntax(self):
        completed = subprocess.run(
            ["bash", "-n", str(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_disable_nounset_around_each_ros_source(self):
        lines = SCRIPT.read_text(encoding="utf-8").splitlines()
        source_lines = [i for i, line in enumerate(lines)
                        if line.strip().startswith("source ")]
        self.assertEqual(len(source_lines), 2)
        for i in source_lines:
            with self.subTest(line=i+1):
                self.assertEqual(lines[i-2].strip(), "set +u")
                self.assertEqual(lines[i+1].strip(), "set -u")

    def test_ros_setup_compatibility_with_unset_variables(self):
        # Reproduce ament's expansion of AMENT_TRACE_SETUP_FILES while the
        # variable is unset. The script's set +u / source / set -u sequence
        # must work, and strict nounset must be restored afterwards.
        result = subprocess.run(["bash", "-c", '''
            set -euo pipefail
            unset AMENT_TRACE_SETUP_FILES
            set +u
            source /dev/stdin <<'EOROS'
            : "${AMENT_TRACE_SETUP_FILES}"
EOROS
            set -u
            [[ $- == *u* ]]
        '''], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
