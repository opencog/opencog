from mpl_toolkits.mplot3d import Axes3D
from spatiotemporal.temporal_events import TemporalEvent
from spatiotemporal.temporal_events.formulas import TemporalRelation
from spatiotemporal.temporal_events.trapezium import generate_random_events
import os
import csv
import numpy as np
import ffx

__author__ = 'keyvan'


class Bin(object):
    def __init__(self):
        self.count = 0

    def update(self, value):
        if self.count == 0:
            self.value = float(value)

        self.value = (self.value * self.count + value) / (self.count + 1)
        self.count += 1


def new():
    discretise_factor = 1.0
    a = 0
    train_a_b, train_b_c, train_a_c = [], [], []
    bins = {}
    for i in xrange(100):
        events = generate_random_events(3)
        a_b = int((events[0] * events[1])['p'] * discretise_factor)
        b_c = int((events[1] * events[2])['p'] * discretise_factor)
        a_c = (events[0] * events[2])['p']

        # if a_b != discretise_factor:
        #     a += 1
        #     continue

        if (a_b, b_c) not in bins:
            bins[(a_b, b_c)] = Bin()

        bins[(a_b, b_c)].update(a_c)

    print len(bins)
    print a

    for key in bins:
        a_b, b_c = key
        train_a_b.append(a_b / discretise_factor)
        train_b_c.append(b_c / discretise_factor)
        train_a_c.append(bins[key].value)

    from matplotlib import pyplot as plt
    fig = plt.figure()
    ax = Axes3D(fig)
    # #
    ax.scatter(train_a_b, train_b_c, train_a_c)
    #
    # # plt.plot(sorted(train_a_c))
    #
    # # plt.scatter(train_b_c, train_a_c)
    #
    plt.show()

    return train_a_b, train_b_c, train_a_c


def create_sample_file(number_of_events=100):
    csv_writer = csv.writer(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~', 'w+'))

    temporal_events = generate_random_events(number_of_events)

    for i in xrange(10):
        for A in temporal_events:
            for B in temporal_events:
                for C in temporal_events:
                    csv_writer.writerow((A * B).to_list() + (B * C).to_list() + (A * C).to_list())


def create_learning_arguments(predicate='p', size=None):
    train_x = [[], []]
    train_y = []
    test_x = [[], []]
    test_y = []
    csv_reader = csv.reader(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~'))

    predicate_index = TemporalRelation.all_relations.index(predicate)

    row_number = -1
    for row in csv_reader:
        row_number += 1
        print row
        if row_number == size:
            break
        for i, element in enumerate(row):
            row[i] = np.float64(element)

        x, y = None, None
        if row_number < size / 2:
            x, y = train_x, train_y
        else:
            x, y = test_x, test_y

        a_b_value = row[predicate_index]
        b_c_value = row[13 + predicate_index]
        a_c_value = row[26 + predicate_index]

        x[0].append(a_b_value)
        x[1].append(b_c_value)
        y.append(a_c_value)

    train_x = np.array(train_x).T
    test_x = np.array(test_x).T
    train_y = np.array(train_y)
    test_y = np.array(test_y)

    return train_x, train_y, test_x, test_y


def learn_ffx():
    train_x, train_y, test_x, test_y = create_learning_arguments(size=10)
    models = ffx.run(train_x, train_y, test_x, test_y, list(TemporalRelation.all_relations))
    for model in models:
        print model


def learn_lasso():
    size = 3
    train_x, train_y, test_x, test_y = create_learning_arguments(size=size)
    # train_a_b, train_b_c, train_a_c = new()
    from matplotlib import pyplot as plt

    from sklearn.linear_model import Lasso, Ridge, ElasticNet, LinearRegression, LassoLars, BayesianRidge, ElasticNetCV, SGDRegressor
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from random import randrange
    import numpy as np

    # clf = KNeighborsRegressor()
    # clf.fit(np.array([train_a_b, train_b_c]).T, train_a_c)

    for i in [randrange(0, len(train_y) / 2) for x in xrange(200)]:
        # print 'learning:', clf.predict(test_x[i]), 'a_b: {0}, b_c: {1} actual: {2}'.format(test_x[i, 0], test_x[i, 1],  test_y[i])
        print 'learning:', np.max([test_x[i, 0], test_x[i, 1]]), 'actual:', test_y[i], test_x[i, 0], test_x[i, 1]


def interpolate():
    from scipy.stats import uniform
    r = []
    a = uniform(2, 5)
    b = uniform(8, 7)
    c = uniform(16, 2)
    d = uniform(21, 1.5)
    e = uniform(26, 4)
    f = uniform(18.5, 0.4)
    g = uniform(19, 1)
    r.append( (uniform(23, 2), e) )
    r.append( (d, e) )
    r.append( (f, e) )
    r.append( (f, d) )
    r.append( (f, g) )
    r.append( (c, e) )
    r.append( (c, d) )
    r.append( (c, f) )
    r.append( (b, e) )
    r.append( (b, d) )
    r.append( (b, f) )
    r.append( (b, c) )
    r.append( (a, b) )

    A = TemporalEvent(a, b)
    B = TemporalEvent(c, d)
    C = []
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    C.append([1.0/5, 1.0/5, 1.0/5, 0, 0, 1.0/5, 0, 0, 1.0/5, 0, 0, 0, 0])
    C.append([1.0/5, 1.0/5, 1.0/5, 0, 0, 1.0/5, 0, 0, 1.0/5, 0, 0, 0, 0])
    C.append([1.0/5, 1.0/5, 1.0/5, 0, 0, 1.0/5, 0, 0, 1.0/5, 0, 0, 0, 0])
    C.append([1.0/5, 1.0/5, 1.0/5, 0, 0, 1.0/5, 0, 0, 1.0/5, 0, 0, 0, 0])
    C.append([1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13, 1.0/13])
    D = []
    E = []
    F = []
    for t in r:
        h = TemporalEvent(*t)
        D.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        E.append((B*h).to_list())
        F.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + (B*h).to_list())

    D = np.array(D)
    C = np.array(C)
    E = np.array(E)
    F = np.array(F)

    from scipy.interpolate import LinearNDInterpolator as nn
    f = nn(F, C)

    return f

if __name__ == '__main__':
    import time

    start = time.time()

    f = interpolate()

    for i in xrange(100):
        events = generate_random_events(3)
        a, b, c = events
        a_b = a * b
        if a_b['p'] != 1:
            continue
        sample = [a_b.to_list() + (b * c).to_list()]
        print f(sample)

    print 'Performance:', time.time() - start, 'seconds'
