from main.shortcut_exit import RoadSurface, ShortcutExitGuard


def test_missing_label_cannot_finish_before_shortcut_label_is_seen():
    guard = ShortcutExitGuard()
    assert guard.update(RoadSurface.STANDARD_BLACK, 1.0) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 1.3) is False


def test_shortcut_label_then_continuous_missing_finishes_shortcut():
    guard = ShortcutExitGuard()
    assert guard.update(RoadSurface.SHORTCUT_WHITE, 1.0) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.0) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.1) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.2) is True


def test_unknown_counts_as_missing_after_shortcut_label_was_seen():
    guard = ShortcutExitGuard()
    guard.update(RoadSurface.SHORTCUT_WHITE, 1.0)
    assert guard.update(RoadSurface.UNKNOWN, 2.0) is False
    assert guard.update(RoadSurface.UNKNOWN, 2.1) is False
    assert guard.update(RoadSurface.STANDARD_BLACK, 2.2) is True


def test_shortcut_label_resets_missing_debounce():
    guard = ShortcutExitGuard()
    guard.update(RoadSurface.SHORTCUT_WHITE, 1.0)
    guard.update(RoadSurface.STANDARD_BLACK, 2.0)
    guard.update(RoadSurface.SHORTCUT_WHITE, 2.1)

    assert guard.update(RoadSurface.STANDARD_BLACK, 2.2) is False
