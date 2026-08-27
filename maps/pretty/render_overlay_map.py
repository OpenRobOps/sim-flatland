#!/usr/bin/env python3
"""
Overlay-ready colour rendering of a ROS occupancy-grid map.

Output framing is pixel-identical to the source PGM:
  * no titles, axes, legend or margins -- the image is only the map,
  * the 1x output has exactly the source dimensions (400 x 400),
  * the 4x output is an exact integer upscale of the same framing, so either
    can be overlaid on the original map without any offset or rescaling.

Geometry is preserved exactly: cells are upscaled by an integer factor (pure
block replication), and the anti-aliasing filter is symmetric, so its
alpha = 0.5 iso-line lies on the original cell boundary -- straight wall runs
keep their exact position and only corners are eased.
"""

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties

MAP_YAML = "/mnt/user-data/uploads/sample_map.yaml"
MODEL_YAML = "/mnt/user-data/uploads/turtlebot_model.yaml"
PGM = "/mnt/user-data/uploads/sample_map.pgm"
OUT_4X = "/mnt/user-data/outputs/sample_map_pretty_1600x1600.png"
OUT_1X = "/mnt/user-data/outputs/sample_map_pretty_400x400.png"

meta = yaml.safe_load(open(MAP_YAML))
model = yaml.safe_load(open(MODEL_YAML))

RES = float(meta["resolution"])
OX, OY, _ = meta["origin"]

grid = np.array(Image.open(PGM))
H, W = grid.shape
X1, Y1 = OX + W * RES, OY + H * RES
EXTENT = [OX, X1, OY, Y1]

occ = grid == 0
free = grid >= 254
passage = grid == 255

wlab, wn = ndimage.label(occ, structure=np.ones((3, 3)))
sizes = ndimage.sum(occ, wlab, range(1, wn + 1))
walls = wlab == (int(np.argmax(sizes)) + 1)
objects = occ & ~walls

# ---------------------------------------------------------------- palette ---
C_WALL, C_OBJ, C_OBJ_EDGE = "#2E3944", "#93A3B1", "#5C6B7A"
C_PAPER = "#F4F1EA"
C_CHG, C_CHG_FILL, C_DOOR = "#1F8A4C", "#3FB273", "#B4622A"
C_PASSAGE = "#C9DEED"

# room keys are the A-D letters; 'hall' is the central corridor
ROOM_COLORS = {"A": "#D9E8CF", "B": "#E5DEEF", "hall": "#DAE7F1",
               "C": "#F6E6CB", "D": "#F9DBCB"}


def hex2rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4)], float) / 255.0


# ------------------------------------------------------- room segmentation ---
# The source encodes its through-routes as 255, so labelling only the 254
# cells separates the floor into the rooms plus the three hall segments.
rlab, rn = ndimage.label(grid == 254)
regions = {}
for i in range(1, rn + 1):
    ys, xs = np.nonzero(rlab == i)
    cy, cx = OY + (H - 1 - ys.mean()) * RES, OX + xs.mean() * RES
    if 8.0 <= cy <= 12.0:
        key = "hall"
    elif cy > 12.0:                      # north band
        key = "A" if cx < 10.0 else "B"
    else:                                # south band
        key = "C" if cx < 10.0 else "D"
    regions.setdefault(key, []).append(i)

masks = {k: np.isin(rlab, ids) for k, ids in regions.items()}

# ------------------------------------------------------------ colour raster ---
S = 4
rgb = np.empty((H, W, 3), float)
rgb[:] = hex2rgb(C_PAPER)
for k, m in masks.items():
    rgb[m] = hex2rgb(ROOM_COLORS[k])
rgb[passage] = hex2rgb(C_PASSAGE)

_, (iy, ix) = ndimage.distance_transform_edt(occ, return_indices=True)
rgb[occ] = rgb[iy[occ], ix[occ]]

step = int(round(1.0 / RES))
fine = np.zeros((H, W), bool)
bold = np.zeros((H, W), bool)
for c in range(0, W, step):
    fine[:, c] = True
    if (c // step) % 5 == 0:
        bold[:, c] = True
for r in range(0, H, step):
    fine[r, :] = True
    if ((H - r) // step) % 5 == 0:
        bold[r, :] = True
m = fine & free
rgb[m] = rgb[m] * 0.90 + 0.10
m = bold & free
rgb[m] = rgb[m] * 0.84 + hex2rgb("#8FA0AE") * 0.16

big = np.kron(rgb, np.ones((S, S, 1)))


def soft_mask(mask, sigma=1.35, lo=0.34, hi=0.66):
    b = ndimage.gaussian_filter(np.kron(mask.astype(float), np.ones((S, S))),
                                sigma)
    a = np.clip((b - lo) / (hi - lo), 0, 1)
    return a * a * (3 - 2 * a)


a_wall, a_obj = soft_mask(walls), soft_mask(objects)

shadow = ndimage.gaussian_filter(
    np.kron((walls | objects).astype(float), np.ones((S, S))), 3.2)
shadow = np.roll(np.roll(shadow, 3, 0), 3, 1)
big *= (1.0 - np.clip(shadow, 0, 1) * 0.16)[..., None]

edge = np.clip(a_obj - ndimage.gaussian_filter(a_obj, 1.9), 0, 1) * 0.9
big = big * (1 - a_obj[..., None]) + hex2rgb(C_OBJ) * a_obj[..., None]
big = big * (1 - edge[..., None]) + hex2rgb(C_OBJ_EDGE) * edge[..., None]
big = big * (1 - a_wall[..., None]) + hex2rgb(C_WALL) * a_wall[..., None]
big = np.clip(big, 0, 1)

# ------------------------------------------------------------ figure metrics ---
plt.rcParams["font.family"] = "DejaVu Sans"

DPI = 200
PIX = W * S                              # 1600
FIG_IN = PIX / DPI                       # 8.0 in, axes fills the whole figure
M_PER_PT = ((X1 - OX) / FIG_IN) / 72.0   # map metres per typographic point

ROOM_FS, CHG_FS, DOOR_FS = 15.0, 11.0, 8.0


def text_size(text, fontsize, weight="normal", linespacing=1.5):
    fp = FontProperties(family="DejaVu Sans", size=fontsize, weight=weight)
    lines = text.split("\n")
    w_pt = max(TextPath((0, 0), s or " ", prop=fp).get_extents().width
               for s in lines)
    h_pt = (len(lines) - 1) * linespacing * fontsize + fontsize
    return w_pt * M_PER_PT, h_pt * M_PER_PT


# ------------------------------------------------- collision-aware placement ---
reserved = ndimage.binary_dilation(occ, iterations=3)


def rect_cells(cx, cy, w, h):
    c0 = max(0, int(np.floor((cx - w / 2 - OX) / RES)))
    c1 = min(W, int(np.ceil((cx + w / 2 - OX) / RES)))
    r0 = max(0, int(np.floor((H - 1) - (cy + h / 2 - OY) / RES)))
    r1 = min(H, int(np.ceil((H - 1) - (cy - h / 2 - OY) / RES)) + 1)
    return r0, r1, c0, c1


def reserve(cx, cy, w, h):
    r0, r1, c0, c1 = rect_cells(cx, cy, w, h)
    reserved[r0:r1, c0:c1] = True


def rect_ok(cx, cy, w, h, region):
    r0, r1, c0, c1 = rect_cells(cx, cy, w, h)
    if r1 <= r0 or c1 <= c0:
        return False
    return (not reserved[r0:r1, c0:c1].any()) and bool(region[r0:r1, c0:c1].all())


def place(region, w, h, near=None, max_dist=None):
    """Most-open spot in `region` fitting a w x h label, optionally pulled
    towards `near` and hard-capped at `max_dist` metres from it."""
    avail = region & ~reserved
    d = ndimage.distance_transform_edt(region & ~reserved) * RES
    base = d
    if near is not None:
        yy, xx = np.mgrid[0:H, 0:W]
        px, py = (near[0] - OX) / RES, (H - 1) - (near[1] - OY) / RES
        dist = np.hypot(xx - px, yy - py) * RES
        if max_dist is not None:
            avail = avail & (dist <= max_dist)
        base = d - 0.55 * dist

    score = np.full((H, W), -np.inf)
    score[avail] = base[avail]
    flat = score.ravel()
    cand = np.argsort(flat)[::-1]
    cand = cand[np.isfinite(flat[cand])]

    for shrink in (1.0, 0.92, 0.84):
        for idx in cand:
            r, c = divmod(int(idx), W)
            x, y = OX + c * RES, OY + (H - 1 - r) * RES
            if rect_ok(x, y, w * shrink, h * shrink, region):
                return x, y
    r, c = np.unravel_index(np.argmax(d), d.shape)
    return OX + c * RES, OY + (H - 1 - r) * RES


def home_region(x, y):
    r, c = int((H - 1) - (y - OY) / RES), int((x - OX) / RES)
    return next(k for k, mk in masks.items() if mk[r, c])


# --- charging zones (fixed points; room labels yield to them) ---------------
battery = next(p for p in model["plugins"] if p["type"] == "Battery")
chg = []
for z in battery["charging_zones"]:
    x, y, r = float(z["x"]), float(z["y"]), float(z["radius"])
    txt = z["name"].split(" (")[0]                    # "Charger A" / "Charger D"
    w, h = text_size(txt, CHG_FS, weight="bold")
    w, h = w + 0.42, h + 0.34
    reserve(x, y, 2 * r + 0.40, 2 * r + 0.40)
    lx, ly = place(masks[home_region(x, y)], w, h, near=(x, y), max_dist=3.0)
    reserve(lx, ly, w + 0.2, h + 0.2)
    chg.append((x, y, r, txt, lx, ly))

# --- exterior door: genuine opening in the south outer wall ------------------
door_cols = np.nonzero(grid[H - 1] >= 254)[0]
door = dpos = None
DOOR_TXT = "EXTERIOR DOOR"
if len(door_cols):
    dx0 = OX + door_cols.min() * RES
    dx1 = OX + (door_cols.max() + 1) * RES
    door = (0.5 * (dx0 + dx1), dx1 - dx0)
    dw, dh = text_size(DOOR_TXT, DOOR_FS, weight="bold")
    dw, dh = dw + 0.36, dh + 0.30
    dpos = place(masks[home_region(door[0], OY + 0.3)], dw, dh,
                 near=(door[0], OY + 0.85), max_dist=3.2)
    reserve(dpos[0], dpos[1], dw + 0.2, dh + 0.2)

# --- room labels ------------------------------------------------------------
ROOM_TITLES = {"A": "OFFICE A", "B": "OFFICE B", "hall": "HALL",
               "C": "OFFICE C", "D": "OFFICE D"}

room_labels = {}
for k, title in ROOM_TITLES.items():
    w, h = text_size(title, ROOM_FS, weight="bold")
    w, h = w + 0.45, h + 0.40
    region = masks[k]
    if k == "hall":
        region = rlab == max(regions["hall"], key=lambda i: (rlab == i).sum())
    x, y = place(region, w, h)
    reserve(x, y, w, h)
    room_labels[k] = (x, y, title)

# ------------------------------------------------------------------ drawing ---
fig = plt.figure(figsize=(FIG_IN, FIG_IN), dpi=DPI)
ax = fig.add_axes([0, 0, 1, 1])          # axes fills the figure: no margins
ax.set_axis_off()
ax.imshow(big, extent=EXTENT, origin="upper", interpolation="antialiased",
          zorder=1)
ax.set_xlim(OX, X1)
ax.set_ylim(OY, Y1)
ax.set_aspect("equal")

for k, (x, y, title) in room_labels.items():
    ax.text(x, y, title, ha="center", va="center", fontsize=ROOM_FS,
            fontweight="bold", color="#3C4956", zorder=6)


def bolt(ax, x, y, s, color, zorder):
    pts = np.array([(-0.30, 1.0), (0.36, 0.10), (0.03, 0.10), (0.30, -1.0),
                    (-0.34, -0.06), (-0.01, -0.06)])
    ax.add_patch(Polygon(pts * s + (x, y), closed=True, facecolor=color,
                         edgecolor="none", zorder=zorder))


for x, y, r, txt, lx, ly in chg:
    ax.add_patch(Circle((x, y), r, facecolor=C_CHG_FILL, alpha=0.30,
                        edgecolor="none", zorder=5))
    ax.add_patch(Circle((x, y), r, facecolor="none", edgecolor=C_CHG, lw=2.2,
                        ls=(0, (4, 2.6)), zorder=6))
    ax.annotate("", xy=(x, y), xytext=(lx, ly), zorder=6,
                arrowprops=dict(arrowstyle="-", color=C_CHG, lw=1.4,
                                shrinkA=3, shrinkB=13))
    ax.add_patch(Circle((x, y), 0.30, facecolor=C_CHG, edgecolor="white",
                        lw=1.8, zorder=7))
    bolt(ax, x, y, 0.175, "white", 8)
    t = ax.text(lx, ly, txt, ha="center", va="center", fontsize=CHG_FS,
                color="#14532D", zorder=8, fontweight="bold")
    t.set_bbox(dict(boxstyle="round,pad=0.34", facecolor="#FFFFFF",
                    edgecolor=C_CHG, linewidth=1.3, alpha=0.95))

if door is not None:
    ax.annotate("", xy=(door[0], OY + 0.05), xytext=dpos, zorder=6,
                arrowprops=dict(arrowstyle="-", color=C_DOOR, lw=1.3,
                                shrinkA=3, shrinkB=2))
    t = ax.text(dpos[0], dpos[1], DOOR_TXT, ha="center", va="center",
                fontsize=DOOR_FS, color=C_DOOR, zorder=8, fontweight="bold")
    t.set_bbox(dict(boxstyle="round,pad=0.32", facecolor="#FFFFFF",
                    edgecolor=C_DOOR, linewidth=1.1, alpha=0.95))

fig.savefig(OUT_4X, dpi=DPI, pad_inches=0)

# exact integer downsample: 4x4 box average -> pixel-aligned 400 x 400
im4 = Image.open(OUT_4X).convert("RGB")
assert im4.size == (PIX, PIX), im4.size
im4.resize((W, H), Image.BOX).save(OUT_1X)

print("wrote", OUT_4X, im4.size)
print("wrote", OUT_1X, Image.open(OUT_1X).size)
print("rooms:", {k: (round(v[0], 2), round(v[1], 2), v[2])
                 for k, v in room_labels.items()})
print("chargers:", [(c[3], round(c[4], 2), round(c[5], 2)) for c in chg])
print("door:", door, "label at", dpos)
