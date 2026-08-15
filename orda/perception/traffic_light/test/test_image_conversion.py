from types import SimpleNamespace

import numpy as np

from traffic_light.image_conversion import imgmsg_to_bgr


def _message(*, encoding, width, height, step, data):
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=step,
        data=data,
    )


def test_yuy2_camera_frame_becomes_three_channel_bgr():
    # Two neutral YUYV pixels: Y0 U Y1 V.  The exact gray value is not the
    # contract; the detector only requires a valid HxWx3 BGR frame.
    msg = _message(
        encoding="yuv422_yuy2",
        width=2,
        height=1,
        step=4,
        data=bytes((128, 128, 128, 128)),
    )
    image = imgmsg_to_bgr(msg)
    assert image.shape == (1, 2, 3)
    assert image.dtype == np.uint8
    assert image.flags.c_contiguous


def test_step_padding_is_not_treated_as_pixels():
    msg = _message(
        encoding="bgr8",
        width=1,
        height=2,
        step=4,
        data=bytes((1, 2, 3, 99, 4, 5, 6, 88)),
    )
    image = imgmsg_to_bgr(msg)
    assert image.tolist() == [[[1, 2, 3]], [[4, 5, 6]]]


def test_unknown_encoding_fails_closed():
    msg = _message(
        encoding="mystery",
        width=1,
        height=1,
        step=1,
        data=b"\x00",
    )
    try:
        imgmsg_to_bgr(msg)
    except ValueError as exc:
        assert "unsupported image encoding" in str(exc)
    else:
        raise AssertionError("unknown encoding must not be guessed")
