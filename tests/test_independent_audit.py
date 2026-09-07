"""Tests for the reusable pure helper in examples/independent_audit.py
(Milestone 9). The audit script itself is exercised end-to-end by
running it directly (see README "Install and test" / CI-equivalent
workflow); this test file covers only the small reusable `check()`
bookkeeping helper, since the audit's substantive engineering checks
are themselves already independent verifications (duplicating them
again here would not add genuine value -- see module docstring).
"""

import importlib.util
import sys
from pathlib import Path

import pytest


def _load_audit_module():
    """Import examples/independent_audit.py as a module without
    executing its __main__ block (pytest imports the module body only,
    since `if __name__ == "__main__":` guards the run)."""
    path = Path(__file__).resolve().parent.parent / "examples" / "independent_audit.py"
    spec = importlib.util.spec_from_file_location("independent_audit_module", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["independent_audit_module"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def audit_module():
    return _load_audit_module()


def test_check_helper_records_pass_for_matching_values(audit_module):
    audit_module.CHECKS.clear()
    result = audit_module.check("unit test pass case", 10.0, 10.0000001, tol=1e-6)
    assert result is True
    assert audit_module.CHECKS[-1][6] is True


def test_check_helper_records_fail_for_mismatched_values(audit_module):
    audit_module.CHECKS.clear()
    result = audit_module.check("unit test fail case", 10.0, 12.0, tol=1e-6)
    assert result is False
    assert audit_module.CHECKS[-1][6] is False


def test_check_helper_relative_tolerance_mode(audit_module):
    audit_module.CHECKS.clear()
    # 1% relative difference, tolerance 2% -> should pass.
    result = audit_module.check("relative tolerance case", 100.0, 101.0, tol=0.02, kind="rel")
    assert result is True
    # Same values, tighter tolerance -> should fail.
    audit_module.CHECKS.clear()
    result = audit_module.check("relative tolerance tight case", 100.0, 101.0, tol=0.001, kind="rel")
    assert result is False


def test_full_audit_run_reports_zero_failures(audit_module, capsys):
    """The audit module's own main() should complete with zero
    failures against the current, unmodified Milestone 1-8 baseline --
    running it here as a lightweight regression guard for this
    milestone's synthesis script (not a re-verification of M1-8
    physics, which remains covered by their own dedicated test files)."""
    audit_module.CHECKS.clear()
    audit_module.main()
    failures = [c for c in audit_module.CHECKS if not c[6]]
    assert len(failures) == 0
    assert len(audit_module.CHECKS) >= 50
