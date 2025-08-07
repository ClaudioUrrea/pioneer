#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import os

# === Configuración ===
INPUT_FILE = "resultados_experimento.xlsx"
OUTPUT_DIR = "posthoc_results"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# === Cargar datos ===
df = pd.read_excel(INPUT_FILE)

# Mapear prioridades
priority_map = {"mochila": "Low", "silla": "Medium", "persona": "High"}
df["ObstaclePriority"] = df["ObstacleClass"].map(priority_map)
df.dropna(subset=["ObstaclePriority"], inplace=True)

# === Métricas a analizar ===
metrics_to_analyze = {
    "TotalEffort": "Total Effort",
    "MinDistance_m": "Minimum Distance (m)"
}

# === Función para imprimir medias ordenadas ===
def print_group_means(df, group_col, metric):
    print(f"\nMedias por grupo para {metric}:")
    means = df.groupby(group_col)[metric].mean()
    means_sorted = means.sort_index()
    for group, mean_val in means_sorted.items():
        print(f"  {group}: {mean_val:.4f}")
    return means_sorted.index.tolist()  # devuelve orden de grupos

# === Ejecutar pruebas Tukey y graficar ===
for metric, label in metrics_to_analyze.items():
    print(f"\n=== Análisis para: {label} ===")

    # Imprimir medias por grupo para chequear
    ordered_groups = print_group_means(df, "ObstaclePriority", metric)

    # Ejecutar Tukey HSD
    tukey = pairwise_tukeyhsd(
        endog=df[metric],
        groups=df["ObstaclePriority"],
        alpha=0.05
    )

    print("\nResultados Tukey HSD:")
    print(tukey.summary())

    # Guardar resumen a TXT
    with open(os.path.join(OUTPUT_DIR, f"{metric}_tukey_summary.txt"), "w") as f:
        f.write(str(tukey.summary()))

    # Graficar resultados
    plt.figure(figsize=(8, 5))
    tukey.plot_simultaneous()
    plt.title(f"Tukey HSD: {label}")
    plt.xlabel(label)
    plt.grid(True, axis='x', linestyle='--', alpha=0.5)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, f"{metric}_tukey_plot.png"), dpi=300)
    plt.close()
    print(f"✅ Gráfico guardado: {metric}_tukey_plot.png")

