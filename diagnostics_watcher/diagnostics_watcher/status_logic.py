"""Pure status-classification helpers. No ROS state — easy to unit-test."""

from typing import Optional, Tuple

from diagnostic_msgs.msg import DiagnosticStatus


def classify_freshness(
    age_sec: Optional[float],
    stale_sec: float,
    grace_active: bool,
) -> Tuple[int, str]:
    """Classify how fresh a topic is.

    age_sec: seconds since the last message, or None if never received.
    stale_sec: age threshold beyond which the topic is considered stale.
    grace_active: True during the startup grace window — downgrades
        missing/stale conditions from ERROR to STALE to avoid bringup noise.
    """
    if age_sec is None:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, "no message received yet"
    if age_sec > stale_sec:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, f"last message {age_sec:.2f}s ago (threshold {stale_sec:.2f}s)"
    return DiagnosticStatus.OK, f"last message {age_sec:.2f}s ago"


def classify_battery(
    percentage: Optional[float],
    age_sec: Optional[float],
    stale_sec: float,
    warn_soc: float,
    critical_soc: float,
    grace_active: bool,
) -> Tuple[int, str]:
    """Classify battery health from latest BatteryState.percentage (0.0..1.0)."""
    if percentage is None or age_sec is None:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, "no battery message received"
    if age_sec > stale_sec:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, f"battery message stale ({age_sec:.2f}s, threshold {stale_sec:.2f}s)"
    if percentage <= critical_soc:
        return DiagnosticStatus.ERROR, f"battery critical: {percentage * 100:.1f}%"
    if percentage <= warn_soc:
        return DiagnosticStatus.WARN, f"battery low: {percentage * 100:.1f}%"
    return DiagnosticStatus.OK, f"battery {percentage * 100:.1f}%"
