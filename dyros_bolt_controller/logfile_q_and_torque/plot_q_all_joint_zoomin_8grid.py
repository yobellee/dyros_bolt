import matplotlib.pyplot as plt
import pandas as pd

# Read the log file
log_file_path = '/home/dyros2/bolt_ws/src/dyros_bolt/dyros_bolt_controller/logfile_q_and_torque/log_q_and_torque.csv'
data = pd.read_csv(log_file_path)

# Create a figure with subplots for each joint in a 2x4 grid
fig, axes = plt.subplots(2, 4, figsize=(20, 5), sharex=True)

# Plot data for each joint
for i in range(8):
    row = i // 4
    col = i % 4
    axes[row, col].plot(data['time'].values, data[f'q_[{i}]'].values, label=f'q_[{i}]')
    axes[row, col].plot(data['time'].values, data[f'desired_q_[{i}]'].values, label=f'desired_q_[{i}]')
    # axes[row, col].plot(data['time'].values, data[f'desired_torque_[{i}]'].values, label=f'desired_torque_[{i}]')
    axes[row, col].set_title(f'Joint {i}')
    axes[row, col].legend()
    axes[row, col].grid()

    # Calculate the y-axis range based on desired_q_[i]
    desired_q_range = [data[f'desired_q_[{i}]'].values[0] - 5, data[f'desired_q_[{i}]'].values[0] + 5]
    axes[row, col].set_ylim(desired_q_range)
    axes[row, col].set_xlim(0, 1.52)#x-axis limit

fig.text(0.5, 0.04, 'Time [s]', ha='center')
fig.text(0.04, 0.5, 'Position', va='center', rotation='vertical')
plt.suptitle('Joint Data Over Time')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
