"""nav_groups.py - which pages are a main sidebar tab and which are sub-tabs of it (no Qt, unit-testable).

4.10.0: the sidebar shows 6 main tabs. Every other page is a SUB-TAB of one of them:

    Home      : Overview | Conveyors | Alarms & error log | Diagnostics     (all the monitoring lives here)
    Navigation: Navigation | Stations
    Mapping   : Mapping | Map library                                        (Engineer)
    Routes    : Routes | Schedule | Reports
    Fleet     : Fleet
    Settings  : Settings | Robot & Safety | Forklift                         (the last two: Engineer)

The page widgets and their IDX_* numbers do NOT change, so every existing "go to page X" call keeps working.
"""
from __future__ import annotations

IDX_HOME = 0
IDX_MAPPING = 1
IDX_NAV = 2
IDX_ROUTES = 3
IDX_CONVEYOR = 4
IDX_MAPLIB = 5
IDX_SETTINGS = 6
IDX_FLEET = 7
IDX_ALARMS = 8
IDX_DIAG = 9
IDX_REPORTS = 10
IDX_STATIONS = 11
IDX_SCHEDULE = 12
IDX_SAFETY = 13
IDX_FORKLIFT = 14

# (main page index, [pages of the group, first = the main page itself])  in sidebar order
GROUPS = [
    (IDX_HOME,     [IDX_HOME, IDX_CONVEYOR, IDX_ALARMS, IDX_DIAG]),
    (IDX_NAV,      [IDX_NAV, IDX_STATIONS]),
    (IDX_MAPPING,  [IDX_MAPPING, IDX_MAPLIB]),
    (IDX_ROUTES,   [IDX_ROUTES, IDX_SCHEDULE, IDX_REPORTS]),
    (IDX_FLEET,    [IDX_FLEET]),
    (IDX_SETTINGS, [IDX_SETTINGS, IDX_SAFETY, IDX_FORKLIFT]),
]

ENGINEER_ONLY = {IDX_MAPPING, IDX_MAPLIB, IDX_DIAG, IDX_SCHEDULE, IDX_SAFETY, IDX_FORKLIFT}

# i18n key of the sub-tab title of each page
TITLE_KEY = {
    IDX_HOME: "sub_overview", IDX_CONVEYOR: "sub_conveyors", IDX_ALARMS: "page_alarms", IDX_DIAG: "page_diag",
    IDX_NAV: "sub_navigation", IDX_STATIONS: "page_stations",
    IDX_MAPPING: "sub_mapping", IDX_MAPLIB: "maplib_title",
    IDX_ROUTES: "sub_routes", IDX_SCHEDULE: "page_schedule", IDX_REPORTS: "page_reports",
    IDX_FLEET: "page_fleet",
    IDX_SETTINGS: "sub_general", IDX_SAFETY: "page_safety", IDX_FORKLIFT: "page_forklift",
}
# i18n key of the main (sidebar) tab
MAIN_KEY = {IDX_HOME: "page_home", IDX_NAV: "page_navigation", IDX_MAPPING: "page_mapping",
            IDX_ROUTES: "page_routes", IDX_FLEET: "page_fleet", IDX_SETTINGS: "page_settings"}
MAIN_ICON = {IDX_HOME: "⌂", IDX_NAV: "⊳", IDX_MAPPING: "◎", IDX_ROUTES: "≡", IDX_FLEET: "◉", IDX_SETTINGS: "⚙"}


def _allowed(idx: int, role: str) -> bool:
    return role == "engineer" or idx not in ENGINEER_ONLY


def main_pages(role: str) -> list[int]:
    """Main sidebar tabs the role can see (a group whose main page is Engineer-only is hidden)."""
    return [m for m, _ in GROUPS if _allowed(m, role)]


def group_main(idx: int) -> int:
    """The main tab a page belongs to (unknown pages belong to Home)."""
    for m, pages in GROUPS:
        if idx in pages:
            return m
    return IDX_HOME


def subpages(main_idx: int, role: str) -> list[int]:
    """Pages of the group of `main_idx` visible to the role, main page first."""
    for m, pages in GROUPS:
        if m == main_idx:
            return [p for p in pages if _allowed(p, role)]
    return [main_idx]


def target_for_main_click(main_idx: int, current: int | None, last_sub: dict[int, int], role: str) -> int:
    """Page to open when a main tab is clicked.

    From another group: the sub-tab used last time in that group (or the main page).
    From inside the same group: the main page (a second click returns to the group's overview).
    """
    if current is not None and group_main(current) == main_idx:
        return main_idx
    pages = subpages(main_idx, role)
    last = last_sub.get(main_idx)
    return last if last in pages else main_idx
