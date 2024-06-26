import numpy as np
import matplotlib.pyplot as plt

# Model Parameters

K = 3
T = 4
a = -(1/T)
b = K/T

# Simulation Parameters

Ts = 0.1 # Sampling Time
Tstop = 20 # End of Simulation Time
N = int(Tstop/Ts) # Simulation length
y = np.zeros(N+2) # Initialization the Tout vector
y[0] = 0 # Initial Vaue

# PI Controller Settings

Kp = 0.5
Ti = 5
r = 3 # Reference value
e = np.zeros(N+2) # Initialization
u = np.zeros(N+2) # Initialization

# Simulation

for k in range(N+1):
  e[k] = r - y[k] # Error Calculation
  u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + (Kp/Ti)*Ts*e[k] # PID Controller
  y[k+1] = (1+Ts*a)*y[k] + Ts*b*u[k] # System Model

# Plot the Simulation Results

t = np.arange(0,Tstop+2*Ts,Ts) #Create the Time Series

# Plot Process Value
plt.figure(1)
plt.plot(t,y)
# Formatting the appearance of the Plot
plt.title('Control of Dynamic System')
plt.xlabel('t [s]')
plt.ylabel('y')
plt.grid()
xmin = 0
xmax = Tstop
ymin = 0
ymax = 8
plt.axis([xmin, xmax, ymin, ymax])
plt.savefig("plots/sample-pid-control.png")
# Plot Control Signal
plt.figure(2)
plt.plot(t,u)
# Formatting the appearance of the Plot
plt.title('Control Signal')
plt.xlabel('t [s]')
plt.ylabel('u [V]')
plt.grid()
plt.savefig("plots/sample-pid-control-u.png")