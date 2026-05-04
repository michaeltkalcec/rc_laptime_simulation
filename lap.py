import numpy
import matplotlib.pyplot as plt
from motor import simulate_motor, simulate_torque
from scipy.interpolate import splprep, splev
from matplotlib.collections import LineCollection

def run_sim(dg):
    fy_max = 13.5

    file = open("track_points.txt", "r")
    track = []
    for line in file:
        x, y = line.strip().split(",")
        track.append((float(x), float(y)))

    track = numpy.array(track)
    x, y = track[:, 0], track[:, 1]
    y = y/-12.915443745632423
    x = x/12.915443745632423

    x = numpy.append(x, x[0])
    y = numpy.append(y, y[0])

    tck, u = splprep([x, y], s=8.0, per=True)

    # fein aufgelöste Strecke
    u_fine = numpy.linspace(0, 1, 1000)

    x_fine, y_fine = splev(u_fine, tck)
    dx, dy = splev(u_fine, tck, der=1)
    ddx, ddy = splev(u_fine, tck, der=2)

    # ---- 3. Krümmung berechnen ----
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5

    # ---- 4. Plot ----
    # plt.figure(figsize=(10, 5))

    # plt.subplot(1, 2, 1)
    # plt.plot(x, y, 'o', label="Original")
    # plt.plot(x_fine, y_fine, '-', label="Spline")
    # plt.axis('equal')
    # plt.title("Strecke")
    # plt.legend()

    # plt.subplot(1, 2, 2)
    # plt.plot(curvature)
    # plt.title("Krümmung")

    # plt.tight_layout()
    # plt.show()

    kappa_safe = numpy.copy(curvature)
    kappa_safe[numpy.abs(kappa_safe) < 1e-6] = 1e-6

    v_corner = numpy.sqrt(fy_max / numpy.abs(kappa_safe))

    rho = 1.2            # Luftdichte
    CdA = 0.01           # RC typisch klein
    # Cr = 0.015
    mass = 1.32          # kg

    # v_max = v[numpy.argmin(numpy.abs(P_required - P_max))]
    v_sim, x, y = simulate_motor(50, V_bat=8.4, dg=dg)
    v_max = v_sim[-1]

    # ---- final speed ----
    # v_final = numpy.minimum(v_corner, v_max)
    v_final = v_corner

    # plt.plot(v_corner, label="v_corner")
    # plt.plot(v_final, label="v_final")
    # # plt.plot(v_max * numpy.ones_like(v_final), label="v_max")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    ay_max = 10.0      # m/s²
    ax_brake_max = -7.0  # Bremsen (negativ!)

    v_forward = numpy.copy(v_final)

    ds = numpy.sqrt(numpy.diff(x_fine)**2 + numpy.diff(y_fine)**2)
    ds = numpy.append(ds, ds[-1]) 

    for i in range(len(v_forward) - 1):
        torque = simulate_torque(v_forward[i], I_max=50, V_bat=8.4, dg=dg)
        F_drag = 0.5 * rho * CdA * v_forward[i]**2
        ax_acc_max = (torque * dg / 0.031 - F_drag) / mass
        v_possible = numpy.sqrt(v_forward[i]**2 + 2 * ax_acc_max * ds[i])
        v_forward[i+1] = numpy.minimum(v_forward[i+1], v_possible)


    # plt.plot(v_final)
    # plt.plot(v_forward, '.')
    # plt.grid(True)
    # plt.show()

    v_final =  numpy.copy(v_forward)

    # counter = 0
    # for i in range(len(v_final) - 2, -1, -1):
    #     inside = v_final[i+1]**2 + 2 * ax_brake_max * ds[i]
    #     v_final[i] = numpy.minimum(v_final[i], v_possible)
    #     counter += 1
    #     if counter > 500:
    #         plt.plot(v_final)
    #         plt.plot(v_forward, '.')
    #         plt.grid(True)
    #         plt.show()
    #         break

    ay = v_final[i]**2 * abs(curvature[i])

    ax_available =  ax_acc_max * numpy.sqrt(numpy.maximum(0, 1 - (ay / ay_max)**2))

    # plt.plot(v_final)
    # plt.grid(True)
    # plt.show()
    dt = ds / v_final
    lap_time = numpy.sum(dt)
    print(f"Lap time: {lap_time:.2f} seconds")


    ds = numpy.sqrt(numpy.diff(x_fine)**2 + numpy.diff(y_fine)**2)
    ds = numpy.append(ds, ds[-1])

    dt = ds / numpy.maximum(v_final, 1e-3)
    t = numpy.cumsum(dt)
    t = numpy.insert(t, 0, 0.0)

    lap_time = t[-1]


    points = numpy.array([x_fine, y_fine]).T.reshape(-1, 1, 2)
    segments = numpy.concatenate([points[:-1], points[1:]], axis=1)

    norm = plt.Normalize(v_final.min(), v_final.max())

    lc = LineCollection(segments, cmap='viridis', norm=norm)
    lc.set_array(v_final)
    lc.set_linewidth(2)

    ds = numpy.sqrt(numpy.diff(x_fine)**2 + numpy.diff(y_fine)**2)
    ds = numpy.append(ds, ds[-1])

    dt = ds / numpy.maximum(v_final, 1e-6)
    t = numpy.cumsum(dt)
    t = numpy.insert(t, 0, 0.0)

    s = numpy.cumsum(ds)
    s = numpy.insert(s, 0, 0.0)

    # --- Figure Layout ---
    # fig = plt.figure(figsize=(14, 6))
    # gs = fig.add_gridspec(2, 2, height_ratios=[2, 1])

    # ax_track = fig.add_subplot(gs[:, 0])
    # ax_v = fig.add_subplot(gs[0, 1])
    # ax_t = fig.add_subplot(gs[1, 1])

    # # --- LINKS: Strecke ---
    # ax_track.add_collection(lc)
    # ax_track.set_xlim(x_fine.min(), x_fine.max())
    # ax_track.set_ylim(y_fine.min(), y_fine.max())
    # ax_track.set_aspect('equal')
    # ax_track.set_title("Track")
    # ax_track.set_xlabel("X")
    # ax_track.set_ylabel("Y")

    # cbar = plt.colorbar(lc, ax=ax_track)
    # cbar.set_label("Speed [m/s]")

    # # --- RECHTS OBEN: Speed Verlauf ---
    # ax_v.plot(s[1:], v_final, label="Speed")
    # ax_v.set_title("Velocity profile")
    # # ax_v.set_xlabel("Distance index")
    # ax_v.set_ylabel("v [m/s]")
    # ax_v.grid(True)

    # # --- RECHTS UNTEN: Zeit ---
    # ax_t.plot(s, t, label="Time")
    # ax_t.set_title(f"Lap time: {lap_time:.3f} s")
    # # ax_t.set_xlabel("Distance index")
    # ax_t.set_ylabel("t [s]")
    # ax_t.grid(True)

    # plt.tight_layout()
    # plt.show()
    return lap_time, v_max

if __name__ == "__main__":
    lt, v_max = run_sim(4.5)
    # a_mass = []
    # a_lt = []
    # a_v_max = []
    # for mass in numpy.linspace(2, 6, 50):
    #     print(f"Übersetung: {mass}")
    #     lt, v_max = run_sim(mass)
    #     a_mass.append(mass)
    #     a_lt.append(lt)
    #     a_v_max.append(v_max)
    # plt.subplot(211)
    # plt.plot(a_mass, a_lt, 'o-')
    # plt.xlabel("Untersetzung")
    # plt.ylabel("Lap Time (s)")
    # plt.grid(True)
    # plt.subplot(212)
    # plt.plot(a_mass, a_v_max, 'o-')
    # plt.xlabel("Untersetzung")
    # plt.ylabel("Top Speed (m/s)")
    # plt.grid(True)
    # plt.show()  