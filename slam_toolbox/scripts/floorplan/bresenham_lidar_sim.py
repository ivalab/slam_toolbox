
import numpy as np
import math
from typing import Tuple

def bresenham_line(x0: int, y0: int, x1: int, y1: int):
    """Classic integer Bresenham; yields (x, y) pixels from start to end inclusive."""
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy

def _world_to_px(x_m: float, y_m: float, origin: Tuple[float, float], m_per_px: float, H: int):
    """World (+x right, +y up) -> pixel indices (ix, iy) for numpy image with origin at top-left.
    We assume the map's lower-left corner sits at 'origin' in world coordinates.
    """
    ox, oy = origin
    mx = (x_m - ox) / m_per_px
    my = (y_m - oy) / m_per_px
    ix = int(round(mx - 0.5))
    # flip y for image coordinates: world y up -> image y down
    iy = int(round((H - 1) - (my - 0.5)))
    return ix, iy

def _px_to_world(ix: int, iy: int, origin: Tuple[float, float], m_per_px: float, H: int):
    """Pixel index -> world center coordinate (meters)."""
    ox, oy = origin
    mx = ix + 0.5
    my = (H - 1 - iy) + 0.5
    x = ox + mx * m_per_px
    y = oy + my * m_per_px
    return x, y

def simulate_scan_bresenham(
    free: np.ndarray,
    m_per_px: float,
    pose_xytheta: Tuple[float, float, float],
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    origin: Tuple[float, float] = (0.0, 0.0),
    noise_std: float = 0.0,
    dropout_prob: float = 0.0,
) -> np.ndarray:
    """Simulate a 2D LiDAR scan using Bresenham line tracing.
    
    Args:
        free: HxW boolean array, True = free, False = occupied.
        m_per_px: meters per pixel.
        pose_xytheta: (x_m, y_m, yaw_rad) robot pose in world frame.
        angle_min, angle_max: scan limits (radians).
        angle_increment: angular step (radians).
        range_min, range_max: sensor min/max range (meters).
        origin: (ox, oy) world coords of the map's lower-left pixel corner.
        noise_std: Gaussian noise std to add to each beam (meters).
        dropout_prob: probability that a beam is replaced with max range.
    
    Returns:
        ranges: 1D float32 array of shape [N], N = floor((angle_max-angle_min)/inc)+1
    """
    if free.dtype != np.bool_:
        free = free.astype(bool)
    H, W = free.shape
    x_m, y_m, yaw = pose_xytheta

    # origin pixel for the robot
    ix0, iy0 = _world_to_px(x_m, y_m, origin, m_per_px, H)
    if not (0 <= ix0 < W and 0 <= iy0 < H) or not free[iy0, ix0]:
        # start in obstacle or OOB -> return min ranges
        N = int(math.floor((angle_max - angle_min) / angle_increment) + 1)
        return np.full(N, range_min, dtype=np.float32)

    # precompute angles
    N = int(math.floor((angle_max - angle_min) / angle_increment) + 1)
    ranges = np.empty(N, dtype=np.float32)
    for k in range(N):
        a = angle_min + k * angle_increment
        th = yaw + a

        # compute beam endpoint at max range in world (then pixels)
        xe = x_m + range_max * math.cos(th)
        ye = y_m + range_max * math.sin(th)
        ix1, iy1 = _world_to_px(xe, ye, origin, m_per_px, H)

        r = range_max
        # walk from robot pixel to endpoint pixel
        for (ix, iy) in bresenham_line(ix0, iy0, ix1, iy1):
            if ix < 0 or ix >= W or iy < 0 or iy >= H:
                break
            if not free[iy, ix]:
                # obstacle hit at this cell center; compute hit distance
                cx, cy = _px_to_world(ix, iy, origin, m_per_px, H)
                dx = cx - x_m
                dy = cy - y_m
                r = max(range_min, min(range_max, math.hypot(dx, dy)))
                break

        # noise and dropout
        if noise_std > 0.0:
            r = float(np.clip(r + np.random.normal(0.0, noise_std), range_min, range_max))
        if dropout_prob > 0.0 and np.random.rand() < dropout_prob:
            r = range_max

        ranges[k] = r

    return ranges.astype(np.float32)

if __name__ == "__main__":
    # quick self-test on a synthetic map: empty room with a block
    H, W = 200, 200
    free = np.ones((H, W), dtype=bool)
    free[0,:] = False; free[-1,:] = False; free[:,0] = False; free[:,-1] = False  # walls
    free[80:120, 90:110] = False  # obstacle block

    m_per_px = 0.05
    origin = (0.0, 0.0)
    pose = (5.0, 5.0, 0.0)  # meters, yaw=0
    angle_min = -math.pi
    angle_max = math.pi
    angle_inc = math.radians(0.5)

    ranges = simulate_scan_bresenham(free, m_per_px, pose, angle_min, angle_max, angle_inc,
                                     range_min=0.05, range_max=10.0, origin=origin, noise_std=0.0)

    print("Generated beams:", ranges.shape[0])
    print("Min/Max range:", float(ranges.min()), float(ranges.max()))
