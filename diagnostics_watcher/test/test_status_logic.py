from diagnostic_msgs.msg import DiagnosticStatus

from diagnostics_watcher.status_logic import (
    classify_freshness,
    classify_battery,
)


class TestClassifyFreshness:
    def test_never_received_within_grace_is_stale(self):
        level, _ = classify_freshness(age_sec=None, stale_sec=2.0, grace_active=True)
        assert level == DiagnosticStatus.STALE

    def test_never_received_after_grace_is_error(self):
        level, _ = classify_freshness(age_sec=None, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.ERROR

    def test_fresh_is_ok(self):
        level, msg = classify_freshness(age_sec=0.5, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.OK
        assert "0.50" in msg

    def test_stale_within_grace_is_stale(self):
        level, _ = classify_freshness(age_sec=5.0, stale_sec=2.0, grace_active=True)
        assert level == DiagnosticStatus.STALE

    def test_stale_after_grace_is_error(self):
        level, msg = classify_freshness(age_sec=5.0, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.ERROR
        assert "2.00" in msg

    def test_at_stale_boundary_is_ok(self):
        level, _ = classify_freshness(age_sec=2.0, stale_sec=2.0, grace_active=False)
        assert level == DiagnosticStatus.OK


class TestClassifyBattery:
    def test_no_message_within_grace_is_stale(self):
        level, _ = classify_battery(
            percentage=None, age_sec=None,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=True,
        )
        assert level == DiagnosticStatus.STALE

    def test_no_message_after_grace_is_error(self):
        level, _ = classify_battery(
            percentage=None, age_sec=None,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR

    def test_stale_message_after_grace_is_error(self):
        level, _ = classify_battery(
            percentage=0.8, age_sec=10.0,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR

    def test_stale_message_within_grace_is_stale(self):
        level, _ = classify_battery(
            percentage=0.8, age_sec=10.0,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=True,
        )
        assert level == DiagnosticStatus.STALE

    def test_at_stale_boundary_is_ok(self):
        level, _ = classify_battery(
            percentage=0.8, age_sec=5.0,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.OK

    def test_critical_soc_is_error(self):
        level, msg = classify_battery(
            percentage=0.03, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR
        assert "critical" in msg.lower()

    def test_at_critical_threshold_is_error(self):
        level, _ = classify_battery(
            percentage=0.05, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.ERROR

    def test_low_soc_is_warn(self):
        level, msg = classify_battery(
            percentage=0.15, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.WARN
        assert "low" in msg.lower()

    def test_at_warn_threshold_is_warn(self):
        level, _ = classify_battery(
            percentage=0.20, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.WARN

    def test_healthy_is_ok(self):
        level, msg = classify_battery(
            percentage=0.85, age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.OK
        assert "85" in msg

    def test_nan_percentage_after_grace_is_warn(self):
        level, msg = classify_battery(
            percentage=float('nan'), age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=False,
        )
        assert level == DiagnosticStatus.WARN
        assert "nan" in msg.lower()

    def test_nan_percentage_within_grace_is_stale(self):
        level, _ = classify_battery(
            percentage=float('nan'), age_sec=0.5,
            stale_sec=5.0, warn_soc=0.2, critical_soc=0.05, grace_active=True,
        )
        assert level == DiagnosticStatus.STALE
