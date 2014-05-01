from copy import deepcopy
from numpy import PINF, NINF
from spatiotemporal.temporal_events.composition.emperical_distribution import overlaps

__author__ = 'keyvan'


class Wagon(object):
    def __init__(self, a, b):
        self._a = float(a)
        self.length = float(b - a)
        self.bound_wagons = []
        self._parent = None
        self.heads = []
        self.tails = []

    @property
    def parent(self):
        return self._parent

    @property
    def is_root(self):
        return self._parent is None

    @property
    def a(self):
        return self._a

    @a.setter
    def a(self, value):
        self.move(self.a - value)
    
    @property
    def b(self):
        return self._a + self.length

    @b.setter
    def b(self, value):
        self.move(self.b - value)

    @property
    def root(self):
        wagon = self
        while not wagon.is_root:
            # print wagon.name
            wagon = wagon._parent
        return wagon

    @property
    def bounds(self):
        return self.a, self.b
    
    def bind(self, other):
        root = other.root
        self.bound_wagons.append(root)
        root._parent = self

    def move_to(self, a, b=None):
        if b is None:
            b = a + self.length
        root = self.root
        scale = (b - a) / self.length
        bias = a - scale * self.a
        if scale == 1.0 and bias == 0.0:
            return

        root.transform(scale, bias)

    def _move_to(self, a, b):
        head_bounds, tail_bounds = (min(self.b, b), max(self.b, b)), (min(self.a, a), max(self.a, a))

        for head in self.heads:
            if overlaps(head.bounds, head_bounds):
                head.move_to(b, b + head.length)
        for tail in self.tails:
            if overlaps(tail.bounds, tail_bounds):
                tail.move_to(a - tail.length, a)

        self._a, self.length = a, b - a

    def transform(self, scale, bias):
        transform_start = self.a * scale + bias
        transform_end = self.b * scale + bias

        tail_bounds = min(self.a, transform_start), max(self.a, transform_start)
        head_bounds = min(self.b, transform_end), max(self.b, transform_end)

        for head in self.heads:
            if overlaps(head.bounds, head_bounds):
                head.move_to(transform_end, transform_end + head.length)
        for tail in self.tails:
            if overlaps(tail.bounds, tail_bounds):
                tail.move_to(transform_start - tail.length, transform_start)

        for wagon in self.bound_wagons:
            wagon.transform(scale, bias)

        self._a, self.length = transform_start, transform_end - transform_start

    def __str__(self, indent=0):
        children = indent * ' ' + repr(self) + '\n'
        for wagon in self.bound_wagons:
            children += (indent + 1) * ' ' + wagon.__str__(indent + 1)
        return children

    def __repr__(self):
        return 'Wagon(a: {0}, b: {1})'.format(self.a, self.b)


class Rail(list):
    def __init__(self, iterable=None):
        wagons = None
        if iterable is not None:
            for wagon in iterable:
                if wagons is None:
                    wagons = [wagon]
                    previous_wagon = wagon
                    continue
                previous_wagon.heads.append(wagon)
                wagons.append(wagon)
                wagon.tails.append(previous_wagon)
                previous_wagon = wagon
        list.__init__(self, wagons)

    def append(self, wagon):
        list.append(self, wagon)
        wagon.tails.append(list.__getitem__(self, -2))
        list.__getitem__(self, -2).heads.append(wagon)

    def insert(self, index, wagon):
        if index < 0:
            raise NotImplementedError
        list.insert(self, index, wagon)
        if index > 0:
            wagon.tails.append(list.__getitem__(self, index - 1))
            list.__getitem__(self, index - 1).heads.append(wagon)
            if index + 1 < len(self):
                list.__getitem__(self, index - 1).heads.remove(list.__getitem__(self, index + 1))

        if index < len(self):
            wagon.heads.append(list.__getitem__(self, index + 1))
            list.__getitem__(self, index + 1).tails.append(wagon)
            if index - 1 >= 0:
                list.__getitem__(self, index + 1).tails.remove(list.__getitem__(self, index - 1))


class RailwaySystem(object):
    def __init__(self):
        self.rails = {}
        self.memo = []

    def add_rail(self, rail_key):
        self.memo.append(('add_rail', (rail_key)))
        self.rails[rail_key] = Rail([Wagon(0, 1), Wagon(1, 2)])

    def move_wagon(self, rail_key, wagon_index, a, b):
        self.memo.append(('move_wagon', (rail_key, wagon_index, a, b)))
        self.rails[rail_key][wagon_index].move_to(a, b)

    def bind_wagons_before_horizontal(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        self.memo.append(('bind_wagons_before_horizontal', (rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)))
        wagon_1 = self.rails[rail_1_key][wagon_1_index]
        wagon_2 = self.rails[rail_2_key][wagon_2_index]

        if wagon_1.b > wagon_2.a:
            wagon_2.move_to(wagon_1.b)

        wagon_2.tails.append(wagon_1)
        wagon_1.heads.append(wagon_2)

    def bind_wagons_after_horizontal(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        self.bind_wagons_before_horizontal(rail_2_key, wagon_2_index, rail_1_key, wagon_1_index)

    def bind_wagons_vertical(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        self.memo.append(('bind_wagons_vertical', (rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)))
        self.rails[rail_1_key][wagon_1_index].bind(self.rails[rail_2_key][wagon_2_index])

    def move_and_bind_vertical(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index, a, b):
        self.move_wagon(rail_1_key, wagon_1_index, a, b)
        self.bind_wagons_vertical(rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)

    def __getitem__(self, rail_key):
        return self.rails[rail_key]

    def __str__(self):
        result = ''
        for key, rail in self.rails.items():
            result += str(key) + ': ' + str(rail) + '\n'
        return result

    def __deepcopy__(self, memo):
        copy = RailwaySystem()
        for action in self.memo:
            copy.__getattribute__(action[0])(*action[1])
        return copy

    def __repr__(self):
        return str(self)


if __name__ == '__main__':
    rails = RailwaySystem()
    a, b, c = 'A', 'B', 'C'
    rails.add_rail(a)
    rails.add_rail(b)
    rails.add_rail(c)

    rails.bind_wagons_before_horizontal(a, 1, b, 1)
    rails.bind_wagons_after_horizontal(b, 1, c, 0)
    rails.bind_wagons_before_horizontal(a, 0, b, 0)
    rails.bind_wagons_before_horizontal(b, 0, c, 1)
    rails.bind_wagons_before_horizontal(a, 1, b, 0)



    # rails.move_and_bind_vertical(a, 0, b, 0, -1, 0.3)
    # # rails[0][0].move_to(-1, 0.3)
    # # rails[0][0].bind(rails[1][0])

    # rails.move_and_bind_vertical(a, 1, b, 0, 0.7, 3)
    # # rails[0][1].move_to(0.7, 3)
    # # rails[0][1].bind(rails[1][0])
    #
    # rails.move_and_bind_vertical(c, 0, b, 0, -0.5, 0.5)
    # # rails[2][0].move_to(-0.5, 0.5)
    # # rails[2][0].bind(rails[1][0])
    #
    # rails.move_and_bind_vertical(b, 1, a, 1, 2, 5)
    # # rails[1][1].move_to(2, 5)
    # # rails[1][1].bind(rails[0][1])
    #
    # rails.move_and_bind_vertical(c, 1, b, 1, 4, 6)
    # # rails[2][1].move_to(4, 6)
    # # rails[2][1].bind(rails[1][1])

    # rails_2 = deepcopy(rails)
    # # rails.move_wagon(b, 1, 7, 13)
    # rails_2.move_wagon(b, 1, 7, 13)

    print rails
    # print rails_2