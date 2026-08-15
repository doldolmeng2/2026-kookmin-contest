from main.shortcut_exit import RoadSurface, ShortcutExitGuard


def test_black_road_cannot_finish_before_shortcut_white_is_seen():
    guard = ShortcutExitGuard()
    assert guard.update(RoadSurface.STANDARD_BLACK, 1.0) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 1.3) is False


def test_white_then_continuous_black_finishes_shortcut():
    guard = ShortcutExitGuard()
    assert guard.update(RoadSurface.SHORTCUT_WHITE, 1.0) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.0) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.1) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.2) is True


def test_unknown_or_white_resets_black_debounce():
    guard = ShortcutExitGuard()
    guard.update(RoadSurface.SHORTCUT_WHITE, 1.0)
    guard.update(RoadSurface.STANDARD_BLACK, 2.0)
    assert guard.update(RoadSurface.UNKNOWN, 2.1) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.2) is False
