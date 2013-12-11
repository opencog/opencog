from spatiotemporal.temporal_events.formulas import TemporalRelation
from spatiotemporal.temporal_events.trapezium import generate_random_events
import os
import csv
import numpy as np
import ffx

__author__ = 'keyvan'


def create_sample_file(number_of_events=100):
    csv_writer = csv.writer(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~', 'w+'))

    temporal_events = generate_random_events(number_of_events)

    for i in xrange(10):
        for A in temporal_events:
            for B in temporal_events:
                for C in temporal_events:
                    csv_writer.writerow((A * B).to_list() + (B * C).to_list() + (A * C).to_list())


def create_learning_arguments(predicate='p', size=None):
    train_x = []
    for i in xrange(26):
        train_x.append([])
    train_y = []
    test_x = []
    for i in xrange(26):
        test_x.append([])
    test_y = []
    csv_reader = csv.reader(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~'))

    predicate_index = TemporalRelation.all_relations.index(predicate)
    no_of_ones = 0
    no_of_zeros = 0
    no_of_fracs = 0
    training_size = 0

    if size is None:
        size = -1

    row_number = -1
    for row in csv_reader:
        row_number += 1

        if row_number == size:
            break
        for i, element in enumerate(row):
            row[i] = np.float64(element)

        x, y = None, None
        if row_number < size / 2:
            x, y = train_x, train_y
        else:
            x, y = test_x, test_y

        predicate_value = row[26 + predicate_index]

        if 0 < predicate_value < 1:
            if no_of_fracs > no_of_ones or no_of_fracs > no_of_zeros:
                continue
            no_of_fracs += 1
        elif predicate_value == 0:
            if no_of_zeros > no_of_ones or no_of_zeros > no_of_fracs:
                continue
            no_of_zeros += 1
        elif predicate_value == 1:
            if no_of_ones > no_of_zeros or no_of_ones > no_of_fracs:
                continue
            no_of_ones += 1

        training_size += 1

        for i in xrange(26):
            x[i].append(row[i])

        y.append(predicate_value)

    train_x = np.array(train_x).T
    test_x = np.array(test_x).T
    train_y = np.array(train_y)
    test_y = np.array(test_y)

    print no_of_fracs, no_of_ones, no_of_zeros
    print 'training size:', training_size

    return train_x, train_y, test_x, test_y


def learn_ffx():
    train_x, train_y, test_x, test_y = create_learning_arguments(size=10)
    models = ffx.run(train_x, train_y, test_x, test_y, list(TemporalRelation.all_relations))
    for model in models:
        print model


def learn_lasso():
    size = 1000000
    train_x, train_y, test_x, test_y = create_learning_arguments(size=size)
    from sklearn.linear_model import Lasso, Ridge, ElasticNet, LinearRegression, LassoLars, BayesianRidge, ElasticNetCV, SGDRegressor
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from random import randrange

    clf = KNeighborsRegressor(n_neighbors=20)
    clf.fit(train_x, train_y)

    for i in [randrange(0, len(train_y) / 2) for x in xrange(20)]:
        print 'learning:', clf.predict(test_x[i]), ', actual:', test_y[i]


if __name__ == '__main__':
    import time

    start = time.time()

    learn_lasso()

    print 'Performance:', time.time() - start, 'seconds'
