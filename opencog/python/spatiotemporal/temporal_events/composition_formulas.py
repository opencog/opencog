from spatiotemporal.temporal_events.trapezium import generate_random_events
import os
import csv

__author__ = 'keyvan'


def create_composition_table(temporal_events):
    print os.path.dirname(os.path.abspath(__file__)) + "/data.csv~"
    csv_writer = csv.writer(open(os.path.dirname(os.path.abspath(__file__)) + '/data.csv~', 'w+'))
    for A in temporal_events:
        for B in temporal_events:
            for C in temporal_events:
                csv_writer.writerow((A * B).to_list() + (B * C).to_list() + (A * C).to_list())


if __name__ == '__main__':
    create_composition_table(generate_random_events(2))
    pass
