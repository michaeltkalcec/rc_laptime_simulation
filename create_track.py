import matplotlib.pyplot as plt

img = plt.imread("mcss.png")
plt.imshow(img)

points = plt.ginput(n=-1, timeout=0)  # klick dich entlang der Strecke
plt.close()

with open("track_points.txt", "w") as f:
    for point in points:
        f.write(f"{point[0]},{point[1]}\n")

# (np.float64(97.39166097060843), np.float64(497.7474367737525)), (np.float64(166.7012987012988), np.float64(312.9217361585782))]
#184.82/14.31