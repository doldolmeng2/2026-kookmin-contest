"""Photometric-only augmentation for the fixed Xycar camera."""
import cv2
import numpy as np


def augment_lighting(image, rng=None):
    """Return a lighting variant without moving any pixel geometrically."""
    rng = np.random if rng is None else rng
    if rng.random() < 0.10:
        return image.copy()
    output = image.astype(np.float32) / 255.0
    tone = rng.random()
    if tone < 0.45:
        gain = rng.uniform(0.65, 0.95) if rng.random() < 0.5 else rng.uniform(1.05, 1.35)
        output = output * gain + rng.uniform(-12.0, 12.0) / 255.0
    elif tone < 0.80:
        gamma = rng.uniform(0.65, 0.95) if rng.random() < 0.5 else rng.uniform(1.05, 1.50)
        output = np.power(np.clip(output, 0.0, 1.0), gamma)
    else:
        contrast = rng.uniform(0.75, 1.30)
        midpoint = output.mean(axis=(0, 1), keepdims=True)
        output = (output - midpoint) * contrast + midpoint + rng.uniform(-20.0, 20.0) / 255.0
    if rng.random() < 0.30:
        temperature = rng.uniform(-0.10, 0.10)
        output *= np.array([1.0-temperature, rng.uniform(0.97, 1.03), 1.0+temperature], np.float32)
    if rng.random() < 0.20:
        saturation = rng.uniform(0.80, 1.20)
        gray = cv2.cvtColor(np.clip(output, 0.0, 1.0), cv2.COLOR_BGR2GRAY)[..., None]
        output = gray + saturation * (output-gray)
    spatial = rng.random()
    if spatial < 0.30:
        output *= _soft_ellipse(image.shape[:2], rng, rng.uniform(0.55, 0.90), True)
    elif spatial < 0.45:
        output += _soft_ellipse(image.shape[:2], rng, rng.uniform(30.0, 100.0)/255.0, False)
    if rng.random() < 0.20:
        output += rng.normal(0.0, rng.uniform(2.0, 8.0)/255.0, output.shape).astype(np.float32)
    return np.clip(output*255.0, 0, 255).astype(np.uint8)


def _soft_ellipse(shape, rng, strength, shadow):
    height, width = shape
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    center_x, center_y = rng.uniform(0, width), rng.uniform(0.45*height, height)
    radius_x, radius_y = rng.uniform(0.18, 0.48)*width, rng.uniform(0.10, 0.30)*height
    angle = rng.uniform(-np.pi, np.pi)
    xr = (xx-center_x)*np.cos(angle)+(yy-center_y)*np.sin(angle)
    yr = -(xx-center_x)*np.sin(angle)+(yy-center_y)*np.cos(angle)
    field = np.exp(-0.5*((xr/radius_x)**2+(yr/radius_y)**2))[..., None]
    return 1.0-(1.0-strength)*field if shadow else strength*field