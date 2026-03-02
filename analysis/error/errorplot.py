import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

IS_NON_ACCUM = True

EXPECTED_FILES = [
    "pathtracing.csv",
    "hybridshift.csv",
    "manifoldhybridshift.csv"
]

if IS_NON_ACCUM:
    EXPECTED_FILES = [
        "pathtracing_non.csv",
        "hybridshift_non.csv",
        "manifoldhybridshift_non.csv"
    ]

def load_csv_file(file_path: Path) -> pd.DataFrame:
    """
    Load a CSV file and ensure it has exactly four columns.
    Assumes first line is header.
    """
    if not file_path.exists():
        raise FileNotFoundError(f"File not found: {file_path}")

    df = pd.read_csv(file_path)

    # Normalize timestamp
    first_value = df["timestamp"].iloc[0]
    df["timestamp"] = (df["timestamp"] - first_value) * 1e-3    # to second

    return df


def main():
    if len(sys.argv) != 2:
        print("Usage: python load_results.py <folder_path>")
        sys.exit(1)

    folder_path = Path(sys.argv[1])

    if not folder_path.exists() or not folder_path.is_dir():
        print(f"Invalid folder path: {folder_path}")
        sys.exit(1)

    data = []
    max_y = float('-inf')
    min_y = float('inf')

    # Load
    for filename in EXPECTED_FILES:
        file_path = folder_path / filename
        df = load_csv_file(file_path)
        data.append(df)
        print(f"Loaded {filename}: {df.shape[0]} rows")
        max_y = max(max_y, df.iloc[0, 1])
        min_y = min(min_y, df.iloc[:, 1].min())

    # Plot
    fig, ax = plt.subplots()

    ax.plot(data[0].iloc[:, 0], data[0].iloc[:, 1], label='Pathtracing')
    ax.plot(data[1].iloc[:, 0], data[1].iloc[:, 1], label='HybridShift')
    ax.plot(data[2].iloc[:, 0], data[2].iloc[:, 1], label='ManifoldHybridShift')

    ax.set_xlabel('Time (sec)')
    ax.set_ylabel('MSE')

    ax.set_xscale('log')   # log scale on x-axis
    ax.set_yscale('log')   # log scale on y-axis

    ax.grid(True, which='both')  # grid on major and minor ticks

    ax.set_xlim(left = 1e-1, right= 100)

    if not IS_NON_ACCUM:
        ax.set_aspect('equal')
        ax.set_ylim(bottom= min_y, top= max_y)

    else:
        ax.set_aspect('equal')
        ax.set_ylim(bottom= min_y, top= 1)

    ax.legend()

    filePath = folder_path
    if IS_NON_ACCUM:
        filePath = filePath / 'mse_non.pdf'

    else:
        filePath = filePath / 'mse.pdf'

    plt.savefig(filePath, format="pdf", bbox_inches="tight")
    plt.show()
    plt.close()

if __name__ == "__main__":
    main()