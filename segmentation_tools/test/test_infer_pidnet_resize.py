import cv2
import numpy as np

from segmentation_tools import infer_pidnet


def test_exact_model_size_skips_resize(monkeypatch):
    frame = np.arange(360 * 640 * 3, dtype=np.uint8).reshape(360, 640, 3)

    def unexpected_resize(*args, **kwargs):
        raise AssertionError('cv2.resize must not run for an exact-size input')

    monkeypatch.setattr(infer_pidnet.cv2, 'resize', unexpected_resize)
    assert infer_pidnet.resize_for_model(frame, 640, 360) is frame


def test_other_size_preserves_inter_linear_resize(monkeypatch):
    frame = np.arange(480 * 640 * 3, dtype=np.uint8).reshape(480, 640, 3)
    expected = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_LINEAR)
    calls = []
    real_resize = cv2.resize

    def recorded_resize(image, size, *, interpolation):
        calls.append((image, size, interpolation))
        return real_resize(image, size, interpolation=interpolation)

    monkeypatch.setattr(infer_pidnet.cv2, 'resize', recorded_resize)
    actual = infer_pidnet.resize_for_model(frame, 640, 360)

    assert len(calls) == 1
    assert calls[0][0] is frame
    assert calls[0][1:] == ((640, 360), cv2.INTER_LINEAR)
    assert np.array_equal(actual, expected)
