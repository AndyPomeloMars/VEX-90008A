import matplotlib.pyplot as plt
import numpy as np

for p in range(1, 6):
    data = []

    for i in range(0, 101):
        j = i / 100
        k = pow(j, p)
        l = k * 100
        data.append(l)

    ypoints = np.array(data)

    plt.plot(ypoints, linestyle = 'dotted')

    plt.title(f"STICK CURVE TEST - {p}")
    plt.xlabel("Stick Turn Power(%)")
    plt.ylabel("Actual Turn Power(%)")

    plt.grid()

    plt.savefig(f"stick_curve{p}.png", dpi=300)

    plt.close()

    # plt.show()