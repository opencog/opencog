__author__ = 'keyvan'

from scipy.stats import norm, t, uniform
import matplotlib.pyplot as plt
x = 'abcde'
y1 = uniform.pdf(x)
#y1 = t.pdf(x, 1, '3')
#y2 = norm.pdf(x, '3')

plt.plot(x, y1)
#plt.plot(x, y2)
plt.xlabel('T')
plt.ylabel('P')
plt.show()