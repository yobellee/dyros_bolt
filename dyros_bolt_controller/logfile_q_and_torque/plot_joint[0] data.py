import matplotlib.pyplot as plt
import pandas as pd

# Read the log file
log_file_path = '/home/dyros2/bolt_ws/src/dyros_bolt/dyros_bolt_controller/logfile_q_and_torque/log_q_and_torque.csv'
data = pd.read_csv(log_file_path)

# Define the joint index to focus on
joint_index = 0

# Create a figure with subplots for the joint in a 2x1 grid
fig, axes = plt.subplots(2, 1, figsize=(15, 10), sharex=True)

# Plot desired_q_ and q_ in the first subplot
axes[0].plot(data['time'].values, data[f'q_[{joint_index}]'].values, label=f'q_[{joint_index}]')
axes[0].plot(data['time'].values, data[f'desired_q_[{joint_index}]'].values, label=f'desired_q_[{joint_index}]')
axes[0].set_title(f'Joint {joint_index} Position Data')
axes[0].legend()
axes[0].grid()
desired_q_mean = data[f'desired_q_[{joint_index}]'].mean()
axes[0].set_ylim([desired_q_mean - 5, desired_q_mean + 5]) #y-axis limit
axes[0].set_xlim([0, 1.52])#range of x-axis
axes[0].set_ylabel('Position')

# Plot desired_torque_ in the second subplot
axes[1].plot(data['time'].values, data[f'desired_torque_[{joint_index}]'].values, label=f'desired_torque_[{joint_index}]', color='r')
axes[1].set_title(f'Joint {joint_index} Torque Data')
axes[1].legend()
axes[1].grid()
axes[1].set_ylabel('Torque')

# Common x-axis label
fig.text(0.5, 0.04, 'Time [s]', ha='center')

plt.suptitle(f'Joint {joint_index} Data Over Time')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
