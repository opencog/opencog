__author__ = 'keyvan'
from generic import TemporalEvent, TemporalInstance, TemporalEventPiecewiseLinear
from trapezium import TemporalEventTrapezium

import numpy as np
import ffx

train_X = np.array([(2, 7, 3), (3, 2, 8)]).T
train_y = np.array([5, 9, 11])

test_X = np.array([(1.5, 2.5, 3.5), (2.4, 3.6, 5.2)]).T
test_y = np.array([3.9, 6.1, 8.7])

models = ffx.run(train_X, train_y, test_X, test_y, ["predictor_a", "predictor_b"])
for model in models:
    yhat = model.simulate(test_X)
    print model
