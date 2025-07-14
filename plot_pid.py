import re
import matplotlib.pyplot as plt

# Read the simulation output
with open('sim_output.txt') as f:
    lines = f.readlines()

iterations = []
torques = []
powers = []
setpoints = []
pid_outputs = []
pid_ps = []
pid_is = []
pid_ds = []

for i, line in enumerate(lines):
    # Parse iteration, torque, power
    m = re.match(r"Iteration\s+(\d+)\s+\| RPM=.*\| Torque=(\d+) deci-Nm \(([\d\.]+) Nm\)", line)
    if m:
        iterations.append(int(m.group(1)))
        torques.append(float(m.group(2))/10.0)  # Convert deci-Nm to Nm
        continue
    m = re.match(r"\s+\| Power: ([\d\.]+) W \(([\d\.]+) kW\)", line)
    if m:
        powers.append(float(m.group(2)))
        continue
    m = re.match(r"\s+\| PID Setpoint=(\d+), Output=(-?\d+) \(P=(-?\d+), I=(-?\d+), D=(-?\d+)\)", line)
    if m:
        setpoints.append(float(m.group(1))/10.0)
        pid_outputs.append(float(m.group(2))/10.0)
        pid_ps.append(float(m.group(3))/10.0)
        pid_is.append(float(m.group(4))/10.0)
        pid_ds.append(float(m.group(5))/10.0)

# Plot
plt.figure(figsize=(12, 8))

plt.subplot(3,1,1)
plt.plot(iterations, torques, label='Torque (Nm)')
plt.plot(iterations, setpoints, label='Setpoint (Nm)', linestyle='--')
plt.ylabel('Torque (Nm)')
plt.legend()

plt.subplot(3,1,2)
plt.plot(iterations, powers, label='Power (kW)', color='orange')
plt.ylabel('Power (kW)')
plt.legend()

plt.subplot(3,1,3)
plt.plot(iterations, pid_outputs, label='PID Output')
plt.plot(iterations, pid_ps, label='P')
plt.plot(iterations, pid_is, label='I')
plt.plot(iterations, pid_ds, label='D')
plt.xlabel('Iteration')
plt.ylabel('PID Terms (Nm)')
plt.legend()

plt.tight_layout()
plt.show() 