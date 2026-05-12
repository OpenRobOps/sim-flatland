"""Pure status-classification helpers. No ROS state — easy to unit-test."""

import math
from typing import Optional, Tuple

from diagnostic_msgs.msg import DiagnosticStatus


def classify_freshness(
    age_sec: Optional[float],
    stale_sec: float,
    grace_active: bool,
) -> Tuple[bytes, str]:
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
) -> Tuple[bytes, str]:
    """Classify battery health from the latest BatteryState.

    percentage: latest BatteryState.percentage in [0.0, 1.0], or None if no
        message has been received.
    age_sec: seconds since the last battery message, or None if no message
        has been received.
    stale_sec: age threshold beyond which the battery message is stale.
    warn_soc / critical_soc: thresholds (in [0.0, 1.0]) for WARN and ERROR.
    grace_active: True during the startup grace window — downgrades
        missing/stale conditions from ERROR to STALE to avoid bringup noise.
    """
    if percentage is None or age_sec is None:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, "no battery message received"
    if math.isnan(percentage):
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.WARN
        return level, "battery percentage unknown (NaN)"
    if age_sec > stale_sec:
        level = DiagnosticStatus.STALE if grace_active else DiagnosticStatus.ERROR
        return level, f"battery message stale ({age_sec:.2f}s, threshold {stale_sec:.2f}s)"
    if percentage <= critical_soc:
        return DiagnosticStatus.ERROR, f"battery critical: {percentage * 100:.1f}%"
    if percentage <= warn_soc:
        return DiagnosticStatus.WARN, f"battery low: {percentage * 100:.1f}%"
    return DiagnosticStatus.OK, f"battery {percentage * 100:.1f}%"
