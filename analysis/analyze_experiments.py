#!/usr/bin/env python3
import rosbag
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse
import os
import math
import numpy as np
from geometry_msgs.msg import Twist

# --- CONFIGURACIÓN EXPERIMENTO ---
CMD_TOPIC = "/husky_velocity_controller/cmd_vel"
ODOM_TOPIC = "/odometry/filtered"
PERSONA_X = 5.320
PERSONA_Y = -0.393

RADIOS_OBSTACULOS = {
    "persona": 0.4,
    "carro": 1.0,
    "botella": 0.15,
    "caja": 0.6,
    "desconocido": 0.4
}

def calcular_esfuerzo(bag):
    print("🔍 Calculando esfuerzo total...")
    total_effort = 0.0
    last_time = None

    for topic, msg, t in bag.read_messages(topics=[CMD_TOPIC]):
        v = msg.linear.x
        w = msg.angular.z
        t_sec = t.to_sec()

        if last_time is not None:
            dt = t_sec - last_time
            if dt > 0.5:
                print(f"⏸ Pausa detectada dt={dt:.2f}s, saltando ese intervalo.")
                continue
            effort = (v ** 2 + w ** 2) * dt
            total_effort += effort

        last_time = t_sec

    print(f"✅ Esfuerzo total: {total_effort:.4f}")
    return total_effort

def calcular_distancia_minima(bag, px, py):
    print("🔍 Calculando distancia mínima al obstáculo...")
    min_dist = float('inf')
    trayectoria = []

    for topic, msg, t in bag.read_messages(topics=[ODOM_TOPIC]):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
        if dist < min_dist:
            min_dist = dist
        trayectoria.append((t.to_sec(), x, y))

    print(f"✅ Distancia mínima: {min_dist:.4f} m")
    return min_dist, trayectoria

def analizar_bag(bag_path):
    nombre = os.path.basename(bag_path)
    print(f"📦 Analizando archivo: {nombre}")

    condition = "ON" if "ON" in nombre else "OFF"
    run = int(nombre.split("_run_")[-1].split(".")[0])

    partes = nombre.split('_')
    obstacle_class = partes[1].lower() if len(partes) > 2 else "desconocido"
    print(f"   → Condition: {condition}, Run: {run}, Obstacle: {obstacle_class}")

    bag = rosbag.Bag(bag_path)

    tiempo_inicio = None
    tiempo_final = None

    for topic, _, t in bag.read_messages(topics=[CMD_TOPIC]):
        tiempo_inicio = t.to_sec()
        break

    for topic, _, t in reversed(list(bag.read_messages(topics=[CMD_TOPIC]))):
        tiempo_final = t.to_sec()
        break

    total_time = tiempo_final - tiempo_inicio if tiempo_inicio and tiempo_final else 0.0
    print(f"   → Total time: {total_time:.2f} s")

    total_effort = calcular_esfuerzo(bag)
    min_distance, trajectory = calcular_distancia_minima(bag, PERSONA_X, PERSONA_Y)

    bag.close()

    return {
        "Run": run,
        "Condition": condition,
        "ObstacleClass": obstacle_class,
        "TotalTime_s": total_time,
        "TotalEffort": total_effort,
        "MinDistance_m": min_distance,
        "File": nombre,
        "Trajectory": trajectory,
    }

def graficar_comparaciones1(df, output_dir):
    print("📊 Generando gráficos de comparación...")
    os.makedirs(output_dir, exist_ok=True)

    def crear_barplot(metrica, ylabel, filename):
        df_sorted = df.sort_values(by=["Run", "Condition"])
        runs = sorted(df["Run"].unique())
        width = 0.35

        on_vals = [df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "ON")][metrica].values[0] if not df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "ON")].empty else np.nan for r in runs]
        off_vals = [df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "OFF")][metrica].values[0] if not df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "OFF")].empty else np.nan for r in runs]

        x = np.arange(len(runs))
        plt.figure(figsize=(8, 5))
        plt.bar(x - width/2, off_vals, width, label='OFF', color='red')
        plt.bar(x + width/2, on_vals, width, label='ON', color='blue')
        plt.xticks(x, [f"Run {r}" for r in runs])
        plt.ylabel(ylabel)
        plt.title(f"{ylabel} Comparison by Condition")
        plt.legend()
        plt.grid(True, axis='y', linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, filename))
        plt.close()
        print(f"   → Gráfico guardado: {filename}")

    crear_barplot("TotalEffort", "Total Effort", "effort_comparison.png")
    crear_barplot("TotalTime_s", "Total Time (s)", "time_comparison.png")
    crear_barplot("MinDistance_m", "Minimum Distance (m)", "distance_comparison.png")


def graficar_comparaciones(df, output_dir):
    print("📊 Generando gráficos de comparación por clase de obstáculo...")
    os.makedirs(output_dir, exist_ok=True)

    def crear_barplot(df_local, metrica, ylabel, filename, clase):
        df_sorted = df_local.sort_values(by=["Run", "Condition"])
        runs = sorted(df_local["Run"].unique())
        width = 0.35

        on_vals = [df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "ON")][metrica].values[0] if not df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "ON")].empty else np.nan for r in runs]
        off_vals = [df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "OFF")][metrica].values[0] if not df_sorted[(df_sorted["Run"] == r) & (df_sorted["Condition"] == "OFF")].empty else np.nan for r in runs]

        x = np.arange(len(runs))
        plt.figure(figsize=(8, 5))
        plt.bar(x - width/2, off_vals, width, label='OFF', color='red')
        plt.bar(x + width/2, on_vals, width, label='ON', color='blue')
        plt.xticks(x, [f"Run {r}" for r in runs])
        plt.ylabel(ylabel)
        plt.title(f"{ylabel} Comparison - {clase}")
        plt.legend()
        plt.grid(True, axis='y', linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, clase, filename))
        plt.close()
        print(f"   → Gráfico guardado: {clase}/{filename}")

    for clase in df["ObstacleClass"].unique():
        df_clase = df[df["ObstacleClass"] == clase]
        clase_dir = os.path.join(output_dir, clase)
        os.makedirs(clase_dir, exist_ok=True)

        crear_barplot(df_clase, "TotalEffort", "Total Effort", "effort_comparison.png", clase)
        crear_barplot(df_clase, "TotalTime_s", "Total Time (s)", "time_comparison.png", clase)
        crear_barplot(df_clase, "MinDistance_m", "Minimum Distance (m)", "distance_comparison.png", clase)

def graficar_trayectorias1(df, output_dir, px, py):
    print("📈 Generando gráfico de trayectorias...")

    runs = sorted(df["Run"].unique())
    for run in runs:
        df_run = df[df["Run"] == run]
        if df_run.empty:
            continue

        plt.figure(figsize=(8, 6))
        ax = plt.gca()

        for _, row in df_run.iterrows():
            tray = row["Trajectory"]
            if not tray:
                continue
            xs = [p[1] for p in tray]
            ys = [p[2] for p in tray]
            color = "blue" if row["Condition"] == "ON" else "red"
            label = f'{row["Condition"]}_run_{row["Run"]}'
            ax.plot(xs, ys, label=label, color=color, alpha=0.8)

        clase = df_run.iloc[0]["ObstacleClass"]
        radio = RADIOS_OBSTACULOS.get(clase, RADIOS_OBSTACULOS["desconocido"])

        ax.scatter([px], [py], marker="X", s=100, color="black", label="Obstacle Center")
        ax.add_patch(patches.Circle((px, py), radio, color='black', alpha=0.25, label=f"Occupied Area (r={radio} m)"))

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(f"Trajectories Run {run} (ON vs OFF)")
        ax.axis("equal")
        ax.grid(True)
        ax.legend()
        plt.tight_layout()

        filename = f"trajectory_run_{run}.png"
        plt.savefig(os.path.join(output_dir, filename))
        plt.close()
        print(f"   → Imagen guardada: {filename}")

def graficar_trayectorias(df, output_dir, px, py):
    print("📈 Generando gráficos de trayectorias por clase de obstáculo...")

    for clase in df["ObstacleClass"].unique():
        df_clase = df[df["ObstacleClass"] == clase]
        clase_dir = os.path.join(output_dir, clase)
        os.makedirs(clase_dir, exist_ok=True)

        runs = sorted(df_clase["Run"].unique())
        for run in runs:
            df_run = df_clase[df_clase["Run"] == run]
            if df_run.empty:
                continue

            plt.figure(figsize=(8, 6))
            ax = plt.gca()

            for _, row in df_run.iterrows():
                tray = row["Trajectory"]
                if not tray:
                    continue
                xs = [p[1] for p in tray]
                ys = [p[2] for p in tray]
                color = "blue" if row["Condition"] == "ON" else "red"
                label = f'{row["Condition"]}_run_{row["Run"]}'
                ax.plot(xs, ys, label=label, color=color, alpha=0.8)

            radio = RADIOS_OBSTACULOS.get(clase, RADIOS_OBSTACULOS["desconocido"])
            ax.scatter([px], [py], marker="X", s=100, color="black", label="Obstacle Center")
            ax.add_patch(patches.Circle((px, py), radio, color='black', alpha=0.25, label=f"Occupied Area (r={radio} m)"))

            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title(f"Trajectories - {clase} - Run {run}")
            ax.axis("equal")
            ax.grid(True)
            ax.legend()
            plt.tight_layout()

            filename = f"trajectory_run_{run}.png"
            plt.savefig(os.path.join(clase_dir, filename))
            plt.close()
            print(f"   → Imagen guardada: {clase}/{filename}")
       



def exportar_excel(df, output_dir):
    print("💾 Exportando a Excel...")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "resultados_experimento.xlsx")
    df.to_excel(output_path, index=False)
    print(f"✅ Archivo guardado en: {output_path}")

def main():
    parser = argparse.ArgumentParser(description="ROSBag experiment analysis")
    parser.add_argument("carpeta_bags", help="Path to folder with .bag files")
    parser.add_argument("--salida", default="resultados", help="Output folder")
    args = parser.parse_args()

    registros = []
    for fname in sorted(os.listdir(args.carpeta_bags)):
        if fname.endswith(".bag"):
            print(f"📦 Analizando {fname}...")
            path = os.path.join(args.carpeta_bags, fname)
            registros.append(analizar_bag(path))

    df = pd.DataFrame(registros)
    exportar_excel(df, args.salida)
    graficar_comparaciones(df, args.salida)
    graficar_trayectorias(df, args.salida, PERSONA_X, PERSONA_Y)

if __name__ == "__main__":
    main()

