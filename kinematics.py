import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from mpl_toolkits.mplot3d import Axes3D

# ============================================================
# XRAY X4 APPROX GEOMETRY
# Units: mm
# ============================================================

# Coordinate System:
# X = forward
# Y = left/right
# Z = height

# ------------------------------------------------------------
# FRONT LEFT CORNER
# ------------------------------------------------------------

HP = {

    # LOWER ARM INNER
    "LFI": np.array([105, 55, 22]),
    "LRI": np.array([75, 55, 22]),

    # UPPER ARM INNER
    "UFI": np.array([105, 48, 48]),
    "URI": np.array([75, 48, 48]),

    # STEERING LINK INNER
    "TRI": np.array([95, 42, 30]),

    # OUTER JOINTS
    "LO": np.array([92, 96, 20]),
    "UO": np.array([92, 94, 52]),
    "TO": np.array([94, 88, 31]),
}

# ============================================================
# STATIC LINK LENGTHS
# ============================================================

def dist(a, b):
    return np.linalg.norm(a - b)

lower_len_front = dist(HP["LFI"], HP["LO"])
lower_len_rear  = dist(HP["LRI"], HP["LO"])

upper_len_front = dist(HP["UFI"], HP["UO"])
upper_len_rear  = dist(HP["URI"], HP["UO"])

toe_len = dist(HP["TRI"], HP["TO"])

upright_upper_to_lower = dist(HP["UO"], HP["LO"])
upright_toe_to_lower   = dist(HP["TO"], HP["LO"])

# ============================================================
# SOLVER
# ============================================================

def solve_corner(wheel_travel):

    target_lo = HP["LO"] + np.array([0, 0, wheel_travel])

    def equations(vars):

        x, y, z = vars

        p = np.array([x, y, z])

        return [

            dist(HP["UFI"], p) - upper_len_front,
            dist(HP["URI"], p) - upper_len_rear,
            dist(target_lo, p) - upright_upper_to_lower,

        ]

    guess = HP["UO"]

    solved = fsolve(equations, guess)

    upper_outer = np.array(solved)

    # Solve toe-link point

    def toe_eq(vars):

        x, y, z = vars

        p = np.array([x, y, z])

        return [

            dist(HP["TRI"], p) - toe_len,
            dist(target_lo, p) - upright_toe_to_lower,
            dist(upper_outer, p) - 21.0,

        ]

    toe_guess = HP["TO"]

    toe_solved = fsolve(toe_eq, toe_guess)

    toe_outer = np.array(toe_solved)

    return target_lo, upper_outer, toe_outer

# ============================================================
# KINEMATICS
# ============================================================

travels = np.linspace(-5, 5, 41)

camber_curve = []
toe_curve = []

solutions = []

for t in travels:

    lo, uo, to = solve_corner(t)

    solutions.append((lo, uo, to))

    # --------------------------------------------------------
    # Camber
    # --------------------------------------------------------

    upright = uo - lo

    camber = np.degrees(
        np.arctan2(upright[1], upright[2])
    )

    camber_curve.append(camber)

    # --------------------------------------------------------
    # Toe
    # --------------------------------------------------------

    steering_vec = to - lo

    toe = np.degrees(
        np.arctan2(steering_vec[0], steering_vec[1])
    )

    toe_curve.append(toe)

# ============================================================
# PLOT
# ============================================================

fig = plt.figure(figsize=(15, 10))

# ============================================================
# 3D GEOMETRY
# ============================================================

ax = fig.add_subplot(221, projection='3d')

lo, uo, to = solve_corner(0)

# lower arms
for p in ["LFI", "LRI"]:
    a = HP[p]
    b = lo
    ax.plot(
        [a[0], b[0]],
        [a[1], b[1]],
        [a[2], b[2]],
        'r', lw=3
    )

# upper arms
for p in ["UFI", "URI"]:
    a = HP[p]
    b = uo
    ax.plot(
        [a[0], b[0]],
        [a[1], b[1]],
        [a[2], b[2]],
        'b', lw=3
    )

# tie rod
ax.plot(
    [HP["TRI"][0], to[0]],
    [HP["TRI"][1], to[1]],
    [HP["TRI"][2], to[2]],
    'g', lw=3
)

# upright
ax.plot(
    [lo[0], uo[0]],
    [lo[1], uo[1]],
    [lo[2], uo[2]],
    'k', lw=4
)

# points
for k, p in HP.items():
    ax.scatter(p[0], p[1], p[2], s=50)

ax.set_title("XRAY X4 Front Suspension")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.view_init(25, -40)

ax.set_box_aspect([1.5, 1.5, 1])

# ============================================================
# CAMBER CURVE
# ============================================================

ax2 = fig.add_subplot(222)

ax2.plot(travels, camber_curve, lw=3)

ax2.set_title("Camber Curve")
ax2.set_xlabel("Wheel Travel [mm]")
ax2.set_ylabel("Camber [deg]")

ax2.grid(True)

# ============================================================
# TOE CURVE
# ============================================================

ax3 = fig.add_subplot(224)

ax3.plot(travels, toe_curve, lw=3)

ax3.set_title("Toe Curve / Bumpsteer")
ax3.set_xlabel("Wheel Travel [mm]")
ax3.set_ylabel("Toe [deg]")

ax3.grid(True)

plt.tight_layout()
plt.show()