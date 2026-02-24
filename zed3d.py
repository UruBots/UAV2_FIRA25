import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import needed for 3D plotting
try:
    df = pd.read_csv("pose2.csv")
    if df.empty:
        raise ValueError("The CSV file is empty.")

    print("Columns:", df.columns.tolist())
    time = df['%time'].to_numpy()
    pos_x = df['field.pose.position.x'].to_numpy()
    pos_z = df['field.pose.position.z'].to_numpy()

    fig = plt.figure(figsize=(10,7))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(time, pos_x, pos_z, label='X-Z Trajectory vs Time')
   
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position X')
    ax.set_zlabel('Position Z')
    ax.set_title('3D Plot of Position X and Z vs Time from ZED bag')
    ax.legend()
    plt.savefig("3d_plotZED.png")
    print("3D plot saved as 3d_plot.png")

except FileNotFoundError:
    print("Error: pose.csv file not found.")
except pd.errors.EmptyDataError:
    print("Error: The CSV file is empty.")
except Exception as e:
    print(f"An error occurred: {e}")
