import matplotlib.pyplot as plt
import pandas as pd

# Read the log file
log_file_path = '/home/dyros2/bolt_ws/src/dyros_bolt/dyros_bolt_controller/logfile_q_and_torque/log_q_and_torque.csv'
data = pd.read_csv(log_file_path)

# Plot all data for each joint on the same graph
for i in range(8):
    plt.figure()
    plt.plot(data['time'].values, data[f'q_[{i}]'].values, label=f'q_[{i}]')
    plt.plot(data['time'].values, data[f'desired_q_[{i}]'].values, label=f'desired_q_[{i}]')
    plt.plot(data['time'].values, data[f'desired_torque_[{i}]'].values, label=f'desired_torque_[{i}]')
    plt.xlabel('Time [s]')
    plt.ylabel('Values')
    plt.title(f'Joint {i} Data')
    plt.legend()
    plt.grid()
    plt.show()
