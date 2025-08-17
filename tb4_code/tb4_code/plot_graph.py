
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os


pattern = os.path.expanduser("~/Documents/sharedDrive/PLOT_GRAPH/Real/comparison/*.csv")
fichiers_csv = glob.glob(pattern)

print(f"[DEBUG] Fichiers trouvés: {len(fichiers_csv)}")
for f in fichiers_csv[:5]:
    print(" -", f)
if not fichiers_csv:
    raise FileNotFoundError(f"Aucun CSV trouvé avec le motif : {pattern}")

fig, ax = plt.subplots(figsize=(10, 6))

for csv_path in fichiers_csv:
    df = pd.read_csv(csv_path, sep=None, engine="python")


    df.columns = [c.strip() for c in df.columns]

    if {'x','y'}.issubset(df.columns):
        xcol, ycol = 'x', 'y'
    else:
        num_cols = df.select_dtypes(include='number').columns.tolist()
        if len(num_cols) >= 2:
            xcol, ycol = num_cols[0], num_cols[1]
        else:
            print(f"[WARN] Colonnes numériques introuvables dans {os.path.basename(csv_path)} -> ignoré")
            continue

    name = os.path.splitext(os.path.basename(csv_path))[0]
    ax.plot(df[xcol], df[ycol], label=name)

if ax.get_legend_handles_labels()[1]:
    ax.legend()

ax.set_title("Comparison between different experiment done")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Errors [m]")
ax.grid(True)
plt.show()