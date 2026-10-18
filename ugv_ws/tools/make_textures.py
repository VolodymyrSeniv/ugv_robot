"""Generate feature-rich textures (blurred noise + random shapes) for VSLAM in simulation.

ground.png, wall.png (shared by every wall: the 'aliased' world) and one unique texture per
wall / obstacle (the 'unique' world, which removes visual aliasing).
"""
import sys
import cv2
import numpy as np

out = sys.argv[1]


def make(size, shapes, seed):
    rng = np.random.default_rng(seed)
    img = np.zeros((size, size, 3), np.uint8)
    for s in (4, 16, 64):                       # multi-scale noise -> corners at many scales
        n = rng.integers(0, 255, (size // s, size // s, 3), dtype=np.uint8)
        img = cv2.addWeighted(img, 0.6, cv2.resize(n, (size, size), interpolation=cv2.INTER_CUBIC), 0.5, 0)
    for _ in range(shapes):
        c = tuple(int(v) for v in rng.integers(0, 255, 3))
        p = tuple(int(v) for v in rng.integers(0, size, 2))
        if rng.random() < 0.5:
            cv2.circle(img, p, int(rng.integers(8, 60)), c, -1)
        else:
            q = (p[0] + int(rng.integers(10, 120)), p[1] + int(rng.integers(10, 120)))
            cv2.rectangle(img, p, q, c, -1)
    return img


cv2.imwrite(f"{out}/ground.png", make(2048, 1500, 7))
cv2.imwrite(f"{out}/wall.png", make(1024, 500, 8))
for i, name in enumerate(["wall_n", "wall_e", "wall_s", "wall_w", "box1", "box2", "box3", "box4"]):
    cv2.imwrite(f"{out}/{name}.png", make(1024, 500, 100 + i))
print("ok")
