from spatiotemporal.temporal_events.trapezium import generate_random_events
import os
import csv
# from ffx import run

__author__ = 'keyvan'


def create_composition_table(temporal_events):
    print os.path.dirname(os.path.abspath(__file__)) + "/data.csv~"
    csv_writer = csv.writer(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~', 'w+'))
    for A in temporal_events:
        for B in temporal_events:
            for C in temporal_events:
                csv_writer.writerow((A * B).to_list() + (B * C).to_list() + (A * C).to_list())


if __name__ == '__main__':
    create_composition_table(generate_random_events(220))
    # import numpy as np
    # import ffx
    #
    # train_X = np.array([(2, 7, 3), (3, 2, 8)]).T
    # train_y = np.array([5, 9, 11])
    #
    # test_X = np.array([(1.5, 2.5, 3.5), (2.4, 3.6, 5.2)]).T
    # test_y = np.array([3.9, 6.1, 8.7])
    #
    # models = ffx.run(train_X, train_y, test_X, test_y, ["predictor_a", "predictor_b"])
    # for model in models:
    #     yhat = model.simulate(test_X)
    #     print model
    pass
