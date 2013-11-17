from spatiotemporal.temporal_events.trapezium import generate_random_events
import ffx

__author__ = 'keyvan'


def create_composition_table(temporal_events):
    table = []

    for A in temporal_events:
        for B in temporal_events:
            for C in temporal_events:
                table.append(((A * B).to_list() + (B * C).to_list(), (A * C).to_list))

    return table


if __name__ == '__main__':
    a = create_composition_table(generate_random_events(20))
    ffx.run()
    pass
