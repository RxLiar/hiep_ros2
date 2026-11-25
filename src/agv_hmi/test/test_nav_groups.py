import os, re, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from agv_hmi.core import nav_groups as G

ALL = set(range(15))


def test_every_page_is_in_exactly_one_group():
    seen = [p for _, pages in G.GROUPS for p in pages]
    assert sorted(seen) == sorted(ALL) and len(seen) == len(set(seen))
    for m, pages in G.GROUPS:
        assert pages[0] == m, "the first page of a group is its main page"


def test_main_tabs_per_role():
    assert G.main_pages("engineer") == [G.IDX_HOME, G.IDX_NAV, G.IDX_MAPPING, G.IDX_ROUTES, G.IDX_FLEET, G.IDX_SETTINGS]
    assert G.main_pages("operator") == [G.IDX_HOME, G.IDX_NAV, G.IDX_ROUTES, G.IDX_FLEET, G.IDX_SETTINGS]


def test_operator_never_sees_engineer_pages():
    for m in G.main_pages("operator"):
        assert not (set(G.subpages(m, "operator")) & G.ENGINEER_ONLY)
    assert G.subpages(G.IDX_HOME, "operator") == [G.IDX_HOME, G.IDX_CONVEYOR, G.IDX_ALARMS]
    assert G.subpages(G.IDX_SETTINGS, "operator") == [G.IDX_SETTINGS]
    assert G.subpages(G.IDX_HOME, "engineer")[-1] == G.IDX_DIAG


def test_group_main():
    assert G.group_main(G.IDX_DIAG) == G.IDX_HOME and G.group_main(G.IDX_ALARMS) == G.IDX_HOME
    assert G.group_main(G.IDX_MAPLIB) == G.IDX_MAPPING and G.group_main(G.IDX_REPORTS) == G.IDX_ROUTES
    assert G.group_main(G.IDX_SAFETY) == G.IDX_SETTINGS and G.group_main(G.IDX_STATIONS) == G.IDX_NAV
    assert G.group_main(99) == G.IDX_HOME


def test_main_click_remembers_last_sub_page():
    last = {G.IDX_HOME: G.IDX_DIAG}
    assert G.target_for_main_click(G.IDX_HOME, G.IDX_NAV, last, "engineer") == G.IDX_DIAG
    assert G.target_for_main_click(G.IDX_HOME, G.IDX_DIAG, last, "engineer") == G.IDX_HOME, "second click = overview"
    assert G.target_for_main_click(G.IDX_HOME, G.IDX_NAV, last, "operator") == G.IDX_HOME, "Diagnostics is engineer-only"
    assert G.target_for_main_click(G.IDX_ROUTES, G.IDX_HOME, {}, "operator") == G.IDX_ROUTES


def test_every_page_has_a_title_and_main_tabs_have_icon_and_key():
    assert set(G.TITLE_KEY) == ALL
    for m in G.main_pages("engineer"):
        assert m in G.MAIN_KEY and m in G.MAIN_ICON


def test_constants_match_the_sidebar():
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "agv_hmi", "ui", "main_window.py")).read()
    for name, val in re.findall(r"^\s+(IDX_[A-Z]+)\s*=\s*(\d+)\s*$", src, re.M):
        assert getattr(G, name) == int(val), name


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"):
            f(); print("ok", n)
