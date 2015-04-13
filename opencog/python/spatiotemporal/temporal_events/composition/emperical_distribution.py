import csv
import numpy
from spatiotemporal.temporal_events.relation_formulas import TemporalRelation
from spatiotemporal.temporal_events.trapezium import TemporalEventTrapezium, generate_random_events
from spatiotemporal.time_intervals import TimeInterval

__author__ = 'keyvan'


def trim_float(float_object, no_digits=12):
    return int(float_object * 10 ** no_digits) / float(10 ** no_digits)


def overlaps(bounds_1, bounds_2):
    a_1, b_1 = bounds_1
    a_2, b_2 = bounds_2
    a_1, b_1, a_2, b_2 = trim_float(a_1), trim_float(b_1), trim_float(a_2), trim_float(b_2)
    return a_1 < a_2 < b_1 or a_1 < b_2 < b_1 or a_2 < a_1 < b_2 or a_2 < b_1 < b_2 or a_1 == a_2 or b_1 == b_2


def generate_random_relations(size=1000):
    relations = []
    A = TemporalEventTrapezium(1000, 1008, 1002, 1004)
    B_as = TimeInterval(991, 1008, size)
    for B_a in B_as:
        B = TemporalEventTrapezium(B_a, B_a + 9, B_a + 3, B_a + 8)
        relations.append((A * B).to_list())
    return relations


def generate_random_relations_file(size=20):
    from datetime import datetime
    from spatiotemporal.time_intervals import TimeInterval

    csv_writer = csv.writer(open('relations.csv~', 'w'))

    year_2010 = TimeInterval(datetime(2010, 1, 1), datetime(2011, 1, 1))

    i = size
    specifications = [None, None]
    while i >= 0:
        for j in xrange(2):
            a = year_2010.random_time()
            beg = year_2010.random_time(start=a)
            end = year_2010.random_time(start=beg)  #(start=a)
            b = year_2010.random_time(start=end)    #(start=max(end, beg))

            specifications[j] = (a, b, beg, end)

        a_beg, a_end = (specifications[0][0], specifications[0][2]), (specifications[0][3], specifications[0][1])
        b_beg, b_end = (specifications[1][0], specifications[1][2]), (specifications[1][3], specifications[1][1])

        valid = False
        for bounds_1, bounds_2 in [
            (a_beg, b_beg), (a_beg, b_end), (a_end, b_beg), (a_end, b_end)
        ]:
            if overlaps(bounds_1, bounds_2):
                valid = True
                break

        if not valid:
            continue

        event_1, event_2 = TemporalEventTrapezium(*specifications[0]), TemporalEventTrapezium(*specifications[1])
        csv_writer.writerow((event_1 * event_2).to_list())

        percentage = (size - i + 1) / float(size) * 100
        if (size - i + 1) % 10**3 == 0:
            print '%' + str(int(percentage))
        i -= 1


def read_data(size=1000):
    csv_reader = csv.reader(open('relations.csv~', 'r'))
    relations = []
    i = size
    ps, ms, os = [], [], []
    for row in csv_reader:
        p, m, o = row[0:3]
        p, m, o = float(p), float(m), float(o)
        if i < 0:
            break
        ps.append(p)
        ms.append(m)
        os.append(o)
        i -= 1

    from matplotlib import pylab as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(ps, ms, os)

    ax.set_xlabel('p')
    ax.set_ylabel('m')
    ax.set_zlabel('o')

    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_zlim(0, 1)


    plt.show()


def classify(size=10000, iterable=None):
    csv_reader = iterable
    if iterable is None:
        csv_reader = csv.reader(open('relations.csv~', 'r'))
    classes = {}

    for i, row in enumerate(csv_reader):
        if i > size - 1:
            print 'number of classes:', len(classes)
            for class_type in classes:
                print classes[class_type][0].type, len(classes[class_type])
            return classes
        relation = TemporalRelation.from_list(row)
        if relation.type not in classes:
            classes[relation.type] = [relation]
        else:
            classes[relation.type].append(relation)

    print 'number of classes:', len(classes)
    for class_type in classes:
        print classes[class_type][0].type, len(classes[class_type])
    return classes


def learn(size=10000):
    classes = classify(size)

    relations = classes['DSOMP']
    size = len(relations)
    train_size = size - size / 4
    train_data = relations[0:train_size]
    test_data = relations[train_size:]
    train_x, train_y = [], []

    for relation in train_data:
        train_x.append([relation['O'], relation['M']])
        train_y.append(relation['P'])

    train_x = numpy.array(train_x)
    train_y = numpy.array(train_y).T

    from sklearn.linear_model import Lasso, Ridge, ElasticNet, LinearRegression, LassoLars, BayesianRidge, ElasticNetCV, SGDRegressor
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from random import randrange

    clf = KNeighborsRegressor(8)#alpha=0.000001)
    clf.fit(train_x, train_y)

    test_x, test_y = [], []
    for relation in test_data:
        test_x.append([relation['O'], relation['M']])
        test_y.append(relation['P'])

    print '\n', '///////// tests ////////'

    for i in xrange(5):
        print 'learning', clf.predict(train_x[i])
        print 'actual', train_y[i], '\n-------------\n'

    print '***************************'

    for i in xrange(5):
        print 'learning', clf.predict(test_x[i])
        print 'actual', test_y[i], '\n-------------\n'


def learn_all(size=10000):
    relations = read_data(size)
    size = len(relations)
    train_size = size - size / 4
    train_data = relations[0:train_size]
    test_data = relations[train_size:]
    train_x, train_y = [], []

    for relation in train_data:
        train_x.append(numpy.array([relation['F']]))
        train_y.append(numpy.array([relation['o']]))

    train_x = numpy.array(train_x)
    train_y = numpy.array(train_y)

    from sklearn.linear_model import Lasso, Ridge, ElasticNet, LinearRegression, LassoLars, BayesianRidge, ElasticNetCV, SGDRegressor
    from sklearn.svm import SVR
    from sklearn.neighbors import KNeighborsRegressor
    from random import randrange

    clf = KNeighborsRegressor()#alpha=0.000001)
    clf.fit(train_x, train_y)

    test_x, test_y = [], []
    for relation in test_data:
        test_x.append(numpy.array([relation['F']]))
        test_y.append(numpy.array([relation['o']]))

    print '\n', '///////// tests ////////'

    for i in xrange(5):
        print 'F:', train_x[i]
        print 'learning', clf.predict(train_x[i])
        print 'actual', train_y[i], '\n-------------\n'

    print '***************************'

    for i in xrange(5):
        print 'F:', test_x[i]
        print 'learning', clf.predict(test_x[i])
        print 'actual', test_y[i], '\n-------------\n'
