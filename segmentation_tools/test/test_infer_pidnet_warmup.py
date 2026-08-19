from types import SimpleNamespace

import pytest

from segmentation_tools import infer_pidnet


class Runner:
    width = 640
    height = 360

    def __init__(self, device_type='cpu', error=None):
        self.device = SimpleNamespace(type=device_type)
        self.error = error
        self.calls = 0

    def predict(self, frame):
        self.calls += 1
        assert frame.shape == (self.height, self.width, 3)
        if self.error:
            raise self.error


def test_warmup_enabled_predicts_exactly_once():
    runner = Runner()
    assert infer_pidnet.warmup_runner(runner, enabled=True) >= 0.0
    assert runner.calls == 1


def test_warmup_disabled_does_not_predict():
    runner = Runner()
    assert infer_pidnet.warmup_runner(runner, enabled=False) is None
    assert runner.calls == 0


def test_cuda_warmup_synchronizes(monkeypatch):
    calls = []
    monkeypatch.setattr(infer_pidnet.torch.cuda, 'synchronize', lambda device: calls.append(device))
    runner = Runner('cuda')
    infer_pidnet.warmup_runner(runner)
    assert runner.calls == 1
    assert calls == [runner.device]


def test_warmup_failure_propagates():
    with pytest.raises(RuntimeError, match='failed'):
        try:
            infer_pidnet.warmup_runner(Runner(error=RuntimeError('boom')))
        except RuntimeError as error:
            raise RuntimeError(f'PIDNet startup warm-up failed: {error}') from error
