import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    df = pd.read_csv("pose2.csv")
    if df.empty:
        raise ValueError("The CSV file is empty.")
    
    print("Columns:", df.columns.tolist())

    time = df['%time'].to_numpy()
    pos_x = df['field.pose.position.x'].to_numpy()
    pos_y = df['field.pose.position.y'].to_numpy()
    pos_z = df['field.pose.position.z'].to_numpy()

    plt.figure(figsize=(10,6))
    plt.plot(time, pos_x, label='Position X')
    plt.plot(time, pos_y, label='Position Y')
    plt.plot(time, pos_z, label='Position Z')

    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position Evolution X, Y and Z from ZED bag')
    plt.grid()
    plt.legend()
    plt.savefig("ZED_xyz_plot.png")
    print("Plot saved as xyz_plot.png")

except FileNotFoundError:
    print("Error: The file pose.csv was not found.")
except pd.errors.EmptyDataError:
    print("Error: The CSV file is empty.")
except Exception as e:
    print(f"An error occurred: {e}")
