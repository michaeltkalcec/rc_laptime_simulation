import matplotlib.pyplot as plt
import numpy

# Hobbywing 21.5
Kv = 2600                           # U/min pro Volt
Kv_rad = Kv * 2 * numpy.pi / 60     # rad/s pro Volt
Kt = 1/Kv_rad                       # ~0.00368 Nm/A
R  = 0.0551                         #Ohm
I_max = 50                          #A

# Ruddog 540p 21.5
# Kv = 2200                           # U/min pro Volt
# Kv_rad = Kv * 2 * numpy.pi / 60     # rad/s pro Volt
# Kt = 1/Kv_rad                       # ~0.00368 Nm/A
# R  = 0.0514                         #Ohm
# I_max = 50                          #A

rho = 1.2
CdA = 0.01

V_bat = 8.4
mass = 1.32

def simulate_motor(I_max, V_bat=8.4, dg=4.5):
    i_mot = [0]
    rpm_mot = [0]
    v = [0]
    V_mot = [0]

    for i in range(1000):
        h_i_mot = (V_bat - rpm_mot[i-1]/Kv) / R
        V_mot.append(V_bat - rpm_mot[i-1]/Kv)
        i_mot.append(numpy.minimum(h_i_mot, I_max))
        torque = i_mot[i] * Kt
        F_drag = 0.5 * rho * CdA * v[i-1]**2
        a = (torque*dg / 0.031 - F_drag) / mass
        v.append(v[i-1] + a * 0.005)
        rpm_mot.append(v[i] * dg / (2 * numpy.pi * 0.031) * 60)

    print(v[-1])
    return v, i_mot, V_mot

def simulate_torque(v, I_max=50, V_bat=8.4, dg=4.5):
    rpm = v * dg / (2 * numpy.pi * 0.031) * 60
    h_i_mot = (V_bat - rpm/Kv) / R
    i_mot = numpy.minimum(h_i_mot, I_max)
    torque = i_mot * Kt
    return torque

if __name__ == "__main__":
    # torque = []
    # for i in range(20):
    #     torque.append(simulate_torque(i))
    # plt.plot(torque)
    # plt.xlabel("Geschwindigkeit (m/s)")
    # plt.ylabel("Drehmoment (Nm)")
    # plt.grid(True)
    # plt.show()
    # exit()

    v, i_mot, V_mot = simulate_motor(50)
    v2, i_mot2, V_mot2 = simulate_motor(50, dg=7)

    plt.subplot(311)
    plt.plot(numpy.array(v)*3.6, label="I_max=50A")
    plt.plot(numpy.array(v2)*3.6, label="I_max=60A")
    #plt.xlabel("Zeit (0.01s Schritte)")
    plt.ylabel("Geschwindigkeit (km/h)")
    plt.grid(True)
    plt.subplot(312)
    plt.plot(i_mot, label="I_max=50A")
    plt.plot(i_mot2, label="I_max=60A")
    plt.ylabel("Strom (A)")
    plt.grid(True)
    plt.subplot(313)
    plt.plot(numpy.array(V_mot)*numpy.array(i_mot), label="I_max=50A")
    plt.plot(numpy.array(V_mot2)*numpy.array(i_mot2), label="I_max=60A")
    plt.ylabel("Leistung (W)")
    plt.grid(True)
    plt.show()
