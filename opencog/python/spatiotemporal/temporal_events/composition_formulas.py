from math import fabs
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
    is_empty = True

    def __init__(self, bin_collection):
        self.count = 0
        self.bin_collection = bin_collection

    def update(self, value):
        if self.is_empty:
            self.bin_collection.number_of_empty_bins -= 1
            self.is_empty = False
        if self.count == 0:
            self.value = float(value)

        self.value = (self.value * self.count + value) / (self.count + 1)
        self.count += 1


class BinCollection(dict):
    def __init__(self, divide_factor):
        self.divide_factor = divide_factor
        for i in xrange(divide_factor + 1):
            for j in xrange(divide_factor + 1):
                self[i * 1.0 / divide_factor, j * 1.0 / divide_factor] = Bin(self)
        self.number_of_empty_bins = len(self)

    def observe(self, a_b, b_c, a_c):
        key_a_b, key_b_c = 0, 0
        least_distance_a_b, least_distance_b_c = 10, 10
        for i in xrange(self.divide_factor + 1):
            key = i * 1.0 / self.divide_factor
            distance = fabs(a_b - key)
            if distance < least_distance_a_b:
                key_a_b = key
                least_distance_a_b = distance
            distance = fabs(b_c - key)
            if distance < least_distance_b_c:
                key_b_c = key
                least_distance_b_c = distance

        self[key_a_b, key_b_c].update(a_c)


def new(size=1000000):
    train_a_b, train_b_c, train_a_c = [], [], []
    bins = BinCollection(10)
    csv_reader = csv.reader(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~'))

    predicate_index = TemporalRelation.all_relations.index('p')

    row_number = -1
    for row in csv_reader:
        row_number += 1
        if row_number == size:
            break
        for i, element in enumerate(row):
            row[i] = np.float64(element)

        a_b = row[predicate_index]
        b_c = row[13 + predicate_index]
        a_c = row[26 + predicate_index]

        bins.observe(a_b, b_c, a_c)

    number_of_none_empty_bins = len(bins) - bins.number_of_empty_bins
    print 'non-empty bins:', number_of_none_empty_bins
    print 'empty bins:', bins.number_of_empty_bins
    print 'average count of non-empty bins:', size / float(number_of_none_empty_bins)
    print 'bin 1,1 count:', bins[1, 1].count
    print 'bin 0,0 count:', bins[0, 0].count

    result = []
    for key in bins:
        a_b, b_c = key
        bin = bins[key]

        if not bin.is_empty:
            a_c = bin.value
            train_a_b.append(a_b)
            train_b_c.append(b_c)
            train_a_c.append(a_c)
            result.append((a_b, b_c, bin.count, a_c))

    for a_b, b_c, count, a_c in sorted(result):
        print 'A-B =', a_b, 'B-C =', b_c, 'Count:', count, 'Value:', a_c

    from matplotlib import pyplot as plt

    # fig = plt.figure()
    # ax = Axes3D(fig)
    # # #
    # ax.scatter(train_a_b, train_b_c, train_a_c)
    # #
    # # # plt.plot(sorted(train_a_c))
    # #
    # # # plt.scatter(train_b_c, train_a_c)
    # #
    # plt.show()

    return train_a_b, train_b_c, train_a_c


def create_sample_file(size=1000000):
    csv_writer = csv.writer(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~', 'w+'))

    for i in xrange(size):
        if i % (size / 200) == 0:
            print '%{0} complete'.format(float(i) / size * 100)
        A, B, C = generate_random_events(3)
        row = (A * B).to_list() + (B * C).to_list() + (A * C).to_list()
        csv_writer.writerow(row)
        del A
        del B
        del C
        del row


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
    train_a_b, train_b_c, train_a_c = new()

    from sklearn.linear_model import Lasso, Ridge, ElasticNet, LinearRegression, LassoLars, BayesianRidge, ElasticNetCV, SGDRegressor
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from random import randrange
    import numpy as np

    # clf = ElasticNet(alpha=0.001)
    # clf.fit(np.array([train_a_b, train_b_c]).T, train_a_c)

    csv_reader = csv.reader(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~'))
    predicate_index = TemporalRelation.all_relations.index('p')

    from scipy.interpolate import Rbf as interpolate
    # f = interpolate(np.array([train_a_b, train_b_c]).T, np.array(train_a_c).T)
    f = interpolate(train_a_b, train_b_c, train_a_c, function='multiquadric')

    row_number = -1
    for row in csv_reader:
        row_number += 1
        if row_number == 10:
            break
        for i, element in enumerate(row):
            row[i] = np.float64(element)

        a_b_value = row[predicate_index]
        b_c_value = row[13 + predicate_index]
        a_c_value = row[26 + predicate_index]

        print 'A-B =', a_b_value, 'B-C =', b_c_value, 'learning:', f(a_b_value, b_c_value), 'actual:', a_c_value
        # print 'A-B =', a_b_value, 'B-C =', b_c_value, 'learning:', clf.predict([a_b_value, b_c_value]), 'actual:', a_c_value

    x = y = np.arange(0, 1, 0.006)
    X, Y = np.meshgrid(x, y)
    zs = np.array([f(x, y) for x, y in zip(np.ravel(X), np.ravel(Y))])
    Z = zs.reshape(X.shape)
    from matplotlib import pyplot as plt
    from matplotlib import cm

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlabel('A{p}B')
    ax.set_ylabel('B{p}C')
    ax.set_zlabel('A{p}C')
    # http://matplotlib.org/examples/color/colormaps_reference.html
    # rstride=1, cstride=1,
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.gist_stern_r, linewidth=0, antialiased=True)

    # http://stackoverflow.com/questions/11777381/invert-an-axis-in-a-matplotlib-grafic
    ax = plt.gca()
    ax.invert_yaxis()
    ax.invert_zaxis()
    plt.show()


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
    r.append((uniform(23, 2), e))
    r.append((d, e))
    r.append((f, e))
    r.append((f, d))
    r.append((f, g))
    r.append((c, e))
    r.append((c, d))
    r.append((c, f))
    r.append((b, e))
    r.append((b, d))
    r.append((b, f))
    r.append((b, c))
    r.append((a, b))

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
    C.append([1.0 / 5, 1.0 / 5, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 0, 0])
    C.append([1.0 / 5, 1.0 / 5, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 0, 0])
    C.append([1.0 / 5, 1.0 / 5, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 0, 0])
    C.append([1.0 / 5, 1.0 / 5, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 1.0 / 5, 0, 0, 0, 0])
    C.append(
        [1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13, 1.0 / 13,
         1.0 / 13, 1.0 / 13])
    D = []
    E = []
    F = []
    for t in r:
        h = TemporalEvent(*t)
        D.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        E.append((B * h).to_list())
        F.append([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] + (B * h).to_list())

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

    new(size=3000000)

    print 'Performance:', time.time() - start, 'seconds'
