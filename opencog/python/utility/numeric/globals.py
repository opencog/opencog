__author__ = 'keyvan'


class _minus_infinity(float):
    def __gt__(self, other):
        return False

    def __lt__(self, other):
        return True

    def __eq__(self, other):
        if other is MINUS_INFINITY:
            return True
        return False

    def __repr__(self):
        return '-inf'

    def __str__(self):
        return repr(self)


class _plus_infinity(float):
    def __gt__(self, other):
        return True

    def __lt__(self, other):
        return False

    def __eq__(self, other):
        if other is PLUS_INFINITY:
            return True
        return False

    def __repr__(self):
        return '+inf'

    def __str__(self):
        return repr(self)


MINUS_INFINITY = _minus_infinity()
PLUS_INFINITY = _plus_infinity()
BIGGEST_UNSIGNED_INTEGER = 2 ** (32 - 1)

if __name__ == '__main__':
    print MINUS_INFINITY < 5
    print 5 < MINUS_INFINITY