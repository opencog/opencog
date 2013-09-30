__author__ = 'keyvan'

from scipy.stats import norm, t
import matplotlib.pyplot as plt
x = [1, 2, 3, 4, 5]
y = t.pdf(x, 1, 3)

plt.plot(x, y)
plt.xlabel('T')
plt.ylabel('P')
plt.show()