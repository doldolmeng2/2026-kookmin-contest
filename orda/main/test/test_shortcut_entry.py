from main.shortcut_entry import ShortcutEntryGuard


def test_left_signal_must_be_visible_for_one_second_before_arming():
    guard = ShortcutEntryGuard()

    assert guard.update(True, 1.0).armed is False
    assert guard.update(True, 1.9).armed is False

    decision = guard.update(True, 2.0)

    assert decision.armed is True
    assert decision.enter is False


def test_armed_left_signal_enters_after_consecutive_missing_frames():
    guard = ShortcutEntryGuard()
    guard.update(True, 1.0)
    guard.update(True, 2.0)

    assert guard.update(False, 2.1).enter is False
    assert guard.update(False, 2.2).enter is False

    decision = guard.update(False, 2.3)

    assert decision.enter is True
    assert decision.missing_frames == 3


def test_short_left_flash_never_enters_shortcut():
    guard = ShortcutEntryGuard()

    guard.update(True, 1.0)
    guard.update(True, 1.5)

    assert guard.update(False, 1.6).enter is False
    assert guard.update(False, 1.7).enter is False
    assert guard.update(False, 1.8).enter is False


def test_armed_left_signal_times_out_without_followup_frames():
    guard = ShortcutEntryGuard()
    guard.update(True, 1.0)
    guard.update(True, 2.0)

    decision = guard.update(False, 4.2)

    assert decision.enter is False
    assert decision.reason == "armed timeout"
