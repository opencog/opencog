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
        self.head = None
        self.tail = None

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

    def move_to(self, a, b):
        root = self.root
        scale = (b - a) / self.length
        bias = a - scale * self.a
        root.transform(scale, bias)

    def _move_to(self, a, b):
        head_bounds, tail_bounds = (min(self.b, b), max(self.b, b)), (min(self.a, a), max(self.a, a))
        if overlaps(self.head.bounds, head_bounds):
            self.head.move_to(b, b + self.head.length)
        if overlaps(self.tail.bounds, tail_bounds):
            self.tail.move_to(a - self.tail.length, a)

        self._a, self.length = a, b - a

    def transform(self, scale, bias):
        transform_start = self.a * scale + bias
        transform_end = self.b * scale + bias

        tail_bounds = min(self.a, transform_start), max(self.a, transform_start)
        head_bounds = min(self.b, transform_end), max(self.b, transform_end)

        if overlaps(self.head.bounds, head_bounds):
            self.head.move_to(transform_end, transform_end + self.head.length)
        if overlaps(self.tail.bounds, tail_bounds):
            self.tail.move_to(transform_start - self.tail.length, transform_start)

        self._a, self.length = transform_start, transform_end - transform_start

        for wagon in self.bound_wagons:
            wagon.transform(scale, bias)

    def __str__(self, indent=0):
        children = indent * ' ' + repr(self) + '\n'
        for wagon in self.bound_wagons:
            children += (indent + 1) * ' ' + wagon.__str__(indent + 1)
        return children

    def __repr__(self):
        return 'Wagon(a: {0}, b: {1})'.format(self.a, self.b)


class Rail(list):
    WAGON_NULL_HEAD = Wagon(PINF, PINF)
    WAGON_NULL_TAIL = Wagon(NINF, NINF)

    def __init__(self, iterable=None):
        wagons = [Rail.WAGON_NULL_TAIL]
        previous_wagon = wagons[0]
        if iterable is not None:
            for wagon in iterable:
                previous_wagon.head = wagon
                wagons.append(wagon)
                wagon.tail = previous_wagon
                previous_wagon = wagon
        previous_wagon.head = Rail.WAGON_NULL_HEAD
        wagons.append(Rail.WAGON_NULL_HEAD)
        list.__init__(self, wagons)

    def append(self, wagon):
        list.insert(self, -1, wagon)
        wagon.tail = list.__getitem__(self, -3)
        wagon.head = list.__getitem__(self, -1)
        list.__getitem__(self, -3).head = wagon

    def insert(self, index, wagon):
        list.insert(self, index, wagon)
        wagon.tail = list.__getitem__(self, index - 1)
        list.__getitem__(self, index - 1).head = wagon

        wagon.head = list.__getitem__(self, index + 1)
        list.__getitem__(self, index + 1).tail = wagon

    def __getitem__(self, index):
        if index >= 0:
            index += 1
        else:
            index -= 1

        return list.__getitem__(self, index)

    def __len__(self):
        return list.__len__(self) - 2

    def __str__(self):
        return list.__str__([item for item in self if item not in [Rail.WAGON_NULL_TAIL, Rail.WAGON_NULL_HEAD]])


class RailwaySystem(object):
    def __init__(self):
        self.rails = {}
        self.groups = []

    def add_rail(self, rail_key):
        self.rails[rail_key] = Rail([Wagon(0, 1), Wagon(1, 2)])

    def move_wagon(self, rail_key, wagon_index, a, b):
        self.rails[rail_key][wagon_index].move_to(a, b)

    def bind_wagons(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index):
        self.rails[rail_1_key][wagon_1_index].bind(self.rails[rail_2_key][wagon_2_index])
        group_found = False
        for group in self.groups:
            if (rail_2_key, wagon_2_index) in group:
                group.append((rail_1_key, wagon_1_index))
                group_found = True
                break
        if not group_found:
            self.groups.append([(rail_2_key, wagon_2_index), (rail_1_key, wagon_1_index)])

    def move_and_bind(self, rail_1_key, wagon_1_index, rail_2_key, wagon_2_index, a, b):
        self.move_wagon(rail_1_key, wagon_1_index, a, b)
        self.bind_wagons(rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)

    def __str__(self):
        result = ''
        for key, rail in self.rails.items():
            result += str(key) + ': ' + str(rail) + '\n'
        return result

    def __deepcopy__(self, memo):
        copy = RailwaySystem()
        for rail_key, rail in self.rails.items():
            rail_key_new = deepcopy(rail_key)
            copy.add_rail(rail_key_new)
            rail_new = copy.rails[rail_key_new]
            rail_new[0].move_to(*rail[0].bounds)
            rail_new[1].move_to(*rail[1].bounds)
        for group in self.groups:
            group_root = group[0]
            for item in group:
                if group_root is item:
                    continue
                rail_1_key, wagon_1_index = item
                rail_2_key, wagon_2_index = group_root
                copy.bind_wagons(rail_1_key, wagon_1_index, rail_2_key, wagon_2_index)
        return copy


if __name__ == '__main__':
    rails = RailwaySystem()
    a, b, c = 'A', 'B', 'C'
    rails.add_rail(a)
    rails.add_rail(b)
    rails.add_rail(c)

    rails.move_and_bind(a, 0, b, 0, -1, 0.3)
    # rails[0][0].move_to(-1, 0.3)
    # rails[0][0].bind(rails[1][0])

    rails.move_and_bind(a, 1, b, 0, 0.7, 3)
    # rails[0][1].move_to(0.7, 3)
    # rails[0][1].bind(rails[1][0])

    rails.move_and_bind(c, 0, b, 0, -0.5, 0.5)
    # rails[2][0].move_to(-0.5, 0.5)
    # rails[2][0].bind(rails[1][0])

    rails.move_and_bind(b, 1, a, 1, 2, 5)
    # rails[1][1].move_to(2, 5)
    # rails[1][1].bind(rails[0][1])

    rails.move_and_bind(c, 1, b, 1, 4, 6)
    # rails[2][1].move_to(4, 6)
    # rails[2][1].bind(rails[1][1])

    rails_2 = deepcopy(rails)
    rails.move_wagon(b, 1, 7, 14)
    rails_2.move_wagon(b, 1, 7, 10)

    print rails
    print rails_2