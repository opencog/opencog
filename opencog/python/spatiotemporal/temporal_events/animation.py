from scipy.stats import uniform
from spatiotemporal.temporal_events import RelationFormulaGeometricMean

f = RelationFormulaGeometricMean()

A = uniform(loc=0, scale=4)
B = uniform(1, 1)
C = uniform(2, 1)

print 'A, B', f.compare(A, B)
print 'B, C', f.compare(B, C)
print 'A, C', f.compare(A, C)


Bh = uniform(0, 1)
# Ah = uniform(-0.95, 3.84)
Ah = uniform(0, 1)
print 'Ah, Bh', f.compare(Ah, Bh)

Ch = uniform(1, 1)
print 'Bh, Ch', f.compare(Bh, Ch)
print 'Ah, Ch', f.compare(Ah, Ch)