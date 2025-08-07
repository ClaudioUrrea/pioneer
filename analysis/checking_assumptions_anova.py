import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import statsmodels.api as sm
from statsmodels.formula.api import ols
from scipy.stats import shapiro
from statsmodels.stats.diagnostic import het_breuschpagan
from statsmodels.stats.diagnostic import het_white
from statsmodels.stats.diagnostic import lilliefors
from statsmodels.stats.api import anova_lm
from scipy.stats import levene

# --- CONFIGURACIÓN ---
archivo_excel = "resultados_experimento.xlsx"
variables = ["TotalEffort", "TotalTime_s", "MinDistance_m"]
categorias = ["Condition", "ObstacleClass"]

# --- CARGA DE DATOS ---
df = pd.read_excel(archivo_excel)
df["Condition"] = df["Condition"].astype("category")
df["ObstacleClass"] = df["ObstacleClass"].astype("category")

# --- ANALISIS POR VARIABLE ---
for var in variables:
    print(f"\n🔍 Análisis de supuestos para: {var}")

    # Modelo lineal
    modelo = ols(f"{var} ~ C(Condition) * C(ObstacleClass)", data=df).fit()

    # --- Normalidad de residuos ---
    residuos = modelo.resid
    stat, p_shapiro = shapiro(residuos)
    print(f"  📊 Shapiro-Wilk test (normalidad de residuos): p = {p_shapiro:.4f} → {'✅ Normalidad' if p_shapiro > 0.05 else '❌ No normal'}")

    # --- QQ-plot ---
    sm.qqplot(residuos, line='45')
    plt.title(f"QQ Plot - {var}")
    plt.tight_layout()
    plt.show()

    # --- Homogeneidad de varianzas: Levene test ---
    grupos = df.groupby(["Condition", "ObstacleClass"])[var].apply(list)
    stat_levene, p_levene = levene(*grupos)
    print(f"  📊 Levene’s test (homogeneidad de varianzas): p = {p_levene:.4f} → {'✅ Homogeneidad' if p_levene > 0.05 else '❌ Varianzas desiguales'}")

