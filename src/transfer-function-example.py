import control
import numpy as np
import matplotlib.pyplot as plt

K = 4
T = 4

num = np.array([K])
den = np.array([T , 1])

H = control.tf(num , den)

print ('H(s) =', H)

t, y = control.step_response(H)
plt.plot(t,y)
plt.title("Step Response")
plt.grid()
plt.savefig("plots/step-response-ts.png")