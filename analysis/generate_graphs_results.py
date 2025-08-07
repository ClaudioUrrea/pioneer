#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import os

# === Configuración general ===
INPUT_FILE = "resultados_experimento.xlsx"
OUTPUT_DIR = "summary_results"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# === Configurar fuente similar a Palatino Linotype ===
matplotlib.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Palatino Linotype", "Palatino", "Georgia", "Times New Roman", "DejaVu Serif"],
    "font.size": 13
})

# === Renombrar clases de obstáculo por prioridad (en inglés) ===
PRIORITY_MAP = {
    "mochila": "Low",
    "silla": "Medium",
    "persona": "High"
}
PRIORITY_ORDER = ["Low", "Medium", "High"]

# === Métricas y etiquetas legibles ===
metric_labels = {
    "TotalEffort": "Total Effort (a.u.)",
    "TotalTime_s": "Total Time (s)",
    "MinDistance_m": "Minimum Distance (m)"
}
colors = ['#ff6b6b', '#4d96ff']  # rojo para OFF, azul para ON

# === Cargar y preparar datos ===
df = pd.read_excel(INPUT_FILE)
df["ObstaclePriority"] = df["ObstacleClass"].map(PRIORITY_MAP)
df.dropna(subset=["ObstaclePriority"], inplace=True)
df["ObstaclePriority"] = pd.Categorical(df["ObstaclePriority"], categories=PRIORITY_ORDER, ordered=True)

# === Inicializar tabla resumen ===
table_rows = []

# === Gráficos por métrica ===
for metric, label in metric_labels.items():
    grouped = df.groupby(['ObstaclePriority', 'Condition'])[metric].agg(['mean', 'std']).unstack()
    means = grouped['mean'].loc[PRIORITY_ORDER]
    stds = grouped['std'].loc[PRIORITY_ORDER]

    # === Crear gráfico ===
    plt.figure(figsize=(8, 6))
    means.plot(kind='bar', yerr=stds, capsize=5, rot=0, figsize=(8, 6), color=colors, legend=False)

    plt.title(f"{label} by Obstacle Priority and Condition", fontsize=14)
    plt.ylabel(label)
    plt.xlabel("Obstacle Priority")
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)
    plt.legend(["OFF", "ON"], title="Condition", fontsize=11, title_fontsize=12)
    plt.grid(axis='y', linestyle='--', alpha=0.6)
    plt.tight_layout()

    filename = f"{metric}_summary_barplot.png"
    save_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(save_path, dpi=600)
    plt.close()
    print(f"📊 Saved high-res plot: {filename}")

# === Tabla resumen ===
for priority in PRIORITY_ORDER:
    for condition in ["OFF", "ON"]:
        group = df[(df["ObstaclePriority"] == priority) & (df["Condition"] == condition)]
        if group.empty:
            continue
        row = {
            "Obstacle Priority": priority,
            "Condition": condition,
            "Total Effort (mean)": round(group["TotalEffort"].mean(), 2),
            "Total Effort (std)": round(group["TotalEffort"].std(), 2),
            "Total Time (s) (mean)": round(group["TotalTime_s"].mean(), 2),
            "Total Time (s) (std)": round(group["TotalTime_s"].std(), 2),
            "Min Distance (m) (mean)": round(group["MinDistance_m"].mean(), 2),
            "Min Distance (m) (std)": round(group["MinDistance_m"].std(), 2)
        }
        table_rows.append(row)

summary_df = pd.DataFrame(table_rows)
summary_df["Obstacle Priority"] = pd.Categorical(summary_df["Obstacle Priority"], categories=PRIORITY_ORDER, ordered=True)
summary_df.sort_values(by=["Obstacle Priority", "Condition"], inplace=True)

output_table = os.path.join(OUTPUT_DIR, "metric_summary_table.xlsx")
summary_df.to_excel(output_table, index=False)
print(f"📄 Summary table exported to: {output_table}")

