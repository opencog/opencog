from math import fabs
from mpl_toolkits.mplot3d import Axes3D
from spatiotemporal.temporal_events import TemporalEvent
from spatiotemporal.temporal_events.relation_formulas import TemporalRelation
from spatiotemporal.temporal_events.trapezium import generate_random_events
import os
import csv
import numpy as np

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


def read_data(size=100):
    train_x, train_y = [], []
    csv_reader = csv.reader(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~'))

    row_number = -1
    for row in csv_reader:
        row_number += 1
        if row_number == size:
            break
        for i, element in enumerate(row):
            row[i] = np.float64(element)

        train_x.append(row[:26])
        train_y.append(row[26:39])

    return np.array(train_x), np.array(train_y)


def create_sample_file(size=100000):
    csv_writer = csv.writer(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~', 'w+'))

    for i in xrange(size):
        if i % (size / 200.0) == 0:
            print '%{0} complete'.format(float(i) / size * 100)
        A, B, C = generate_random_events(3)
        row = (A * B).to_list() + (B * C).to_list() + (A * C).to_list()
        csv_writer.writerow(row)
        del A
        del B
        del C
        del row


def learn(size=10000):
    train_x, train_y = read_data(size)

    from sklearn.linear_model import Lasso, Ridge, ElasticNet, LinearRegression, LassoLars, BayesianRidge, ElasticNetCV, SGDRegressor
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from random import randrange

    predicate_index = TemporalRelation.all_relations.index('p')

    clf = KNeighborsRegressor()
    clf.fit(train_x, train_y[:, predicate_index])
    for i in xrange(10):
        A, B, C = generate_random_events(3)
        print 'learning', clf.predict((A * B).to_list() + (B * C).to_list())
        print 'actual', (A * C).to_list()[predicate_index], '\n-------------\n'

    # from scipy.interpolate import Rbf as interpolate
    # # f = interpolate(np.array([train_a_b, train_b_c]).T, np.array(train_a_c).T)
    # f = interpolate(train_a_b, train_b_c, train_a_c, function='multiquadric')
    #
    # row_number = -1
    # for row in csv_reader:
    #     row_number += 1
    #     if row_number == 10:
    #         break
    #     for i, element in enumerate(row):
    #         row[i] = np.float64(element)
    #
    #     a_b_value = row[predicate_index]
    #     b_c_value = row[13 + predicate_index]
    #     a_c_value = row[26 + predicate_index]
    #
    #     print 'A-B =', a_b_value, 'B-C =', b_c_value, 'learning:', f(a_b_value, b_c_value), 'actual:', a_c_value
    #     # print 'A-B =', a_b_value, 'B-C =', b_c_value, 'learning:', clf.predict([a_b_value, b_c_value]), 'actual:', a_c_value
    #
    # x = y = np.arange(0, 1, 0.006)
    # X, Y = np.meshgrid(x, y)
    # zs = np.array([f(x, y) for x, y in zip(np.ravel(X), np.ravel(Y))])
    # Z = zs.reshape(X.shape)
    # from matplotlib import pyplot as plt
    # from matplotlib import cm
    #
    # fig = plt.figure()
    # ax = Axes3D(fig)
    # ax.set_xlabel('A{p}B')
    # ax.set_ylabel('B{p}C')
    # ax.set_zlabel('A{p}C')
    # # http://matplotlib.org/examples/color/colormaps_reference.html
    # # rstride=1, cstride=1,
    # ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.gist_stern_r, linewidth=0, antialiased=True)
    #
    # # http://stackoverflow.com/questions/11777381/invert-an-axis-in-a-matplotlib-grafic
    # ax = plt.gca()
    # ax.invert_yaxis()
    # ax.invert_zaxis()
    # plt.show()


if __name__ == '__main__':
    import time

    start = time.time()
    size = 4000000
    create_sample_file(size)
    learn(size)
    print 'Performance:', time.time() - start, 'seconds'
