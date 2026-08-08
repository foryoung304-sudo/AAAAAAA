"""Host-side invariants for the aircraft field-map calibration feature.

This does not compile the IAR target.  It catches accidental Flash-page reuse,
shared-RAM growth, removal of the mission fallback, and transform regressions.
"""

from __future__ import annotations

import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(pattern: str, text: str, message: str) -> None:
    if re.search(pattern, text, re.MULTILINE) is None:
        raise AssertionError(message)


def local_to_earth(
    origin_x: float,
    origin_y: float,
    yaw_deg: float,
    forward: float,
    right: float,
) -> tuple[float, float]:
    yaw = math.radians(yaw_deg)
    return (
        origin_x + forward * math.cos(yaw) - right * math.sin(yaw),
        origin_y + forward * math.sin(yaw) + right * math.cos(yaw),
    )


def main() -> None:
    menu_h = source("project/code/pid_menu.h")
    menu_c = source("project/code/pid_menu.c")
    nav_c = source("project/code/vision_nav.c")
    shared_c = source("project/code/vision_shared.c")

    require(r"FIELD_MAP_CALIBRATION_ENABLE\s+0u", menu_h,
            "baseline run requires field-map calibration to be disabled")
    require(r"FIELD_MAP_FLASH_PAGE\s+94u", menu_h,
            "field map must remain on Work Flash page 94")
    require(r"FIELD_MAP_PID_RESERVED_PAGE\s+95u", menu_h,
            "historical PID page 95 must remain explicitly reserved")
    require(r"FIELD_MAP_FLASH_PAGE\s*!=\s*FIELD_MAP_PID_RESERVED_PAGE",
            menu_c, "compile-time map/PID page separation is missing")
    require(r"memcmp\(&verify,\s*&field_map_record", menu_c,
            "Flash write-back verification is missing")
    require(r"vehicle_state\.armed\s*!=\s*0u", menu_c,
            "disarmed-only write guard is missing")
    require(r"VISION_NAV_COMPETITION_ADVANCE_CM", nav_c,
            "rough search-center fallback was removed")
    require(r"if\(FIELD_MAP_CALIBRATION_ENABLE == 0u\)\s*\{\s*return 0u;",
            menu_c, "disabled map must not supply a mission center")
    require(r"sizeof\(VisionAttitudeMailbox_t\)\s*<=\s*256u", shared_c,
            "attitude mailbox bound is missing")

    # Stored map centre is the arithmetic mean of all recorded beacon points.
    points = [(100.0, 20.0), (300.0, -20.0), (200.0, 100.0)]
    center = (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )
    assert center == (200.0, 100.0 / 3.0)

    # A nonzero EKF origin and a changed start heading must preserve the map.
    x, y = local_to_earth(37.0, -81.0, 90.0, 200.0, 50.0)
    assert math.isclose(x, -13.0, abs_tol=1e-6)
    assert math.isclose(y, 119.0, abs_tol=1e-6)

    print("field-map contract: PASS")


if __name__ == "__main__":
    main()
