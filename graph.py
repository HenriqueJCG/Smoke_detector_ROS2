import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("visibility_log_kdtree.csv")

plt.plot(df["time"], df["prob"])

plt.xlabel("Time [s]")
plt.ylabel("Visibility")
plt.title("Visibility / Degradation Over Time")

plt.grid(True)
plt.show()