import pandas as pd
import statsmodels.api as sm
from statsmodels.formula.api import ols
import argparse

def anova_sobre_variable(df, variable):
    print(f"\n📊 ANOVA para: {variable}")

    # Asegurar que las columnas categóricas están bien codificadas
    df["Condition"] = df["Condition"].astype("category")
    df["ObstacleClass"] = df["ObstacleClass"].astype("category")

    model = ols(f"{variable} ~ C(Condition) * C(ObstacleClass)", data=df).fit()
    anova_table = sm.stats.anova_lm(model, typ=2)
    print(anova_table)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("excel_path", help="Ruta al archivo Excel generado por el script de análisis.")
    args = parser.parse_args()

    df = pd.read_excel(args.excel_path)

    # Selección de métricas a analizar
    variables = ["TotalEffort", "TotalTime_s", "MinDistance_m"]

    for var in variables:
        anova_sobre_variable(df, var)

if __name__ == "__main__":
    main()

