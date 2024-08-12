import matplotlib.pyplot as plt
import pandas as pd

# Read the log file
log_file_path = '/home/dyros2/bolt_ws/src/dyros_bolt/dyros_bolt_controller/logfile_q_and_torque/log_q_and_torque.csv'
data = pd.read_csv(log_file_path)

# Create a figure with subplots for each joint
fig, axes = plt.subplots(8, 1, figsize=(12, 16), sharex=True)

# Plot data for each joint
for i in range(8):
    axes[i].plot(data['time'].values, data[f'q_[{i}]'].values, label=f'q_[{i}]')
    axes[i].plot(data['time'].values, data[f'desired_q_[{i}]'].values, label=f'desired_q_[{i}]')
    axes[i].plot(data['time'].values, data[f'desired_torque_[{i}]'].values, label=f'desired_torque_[{i}]')
    axes[i].set_ylabel(f'Joint {i}')
    axes[i].legend()
    axes[i].grid()

axes[-1].set_xlabel('Time [s]')
plt.suptitle('Joint Data Over Time')
plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()
