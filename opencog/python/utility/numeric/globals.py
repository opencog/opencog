__author__ = 'keyvan'

SMALLEST_FLOAT = 0.1 ** 323

# generic epsilon
EPSILON = 0.1 ** 5


class _minus_infinity(float):
    def __cmp__(self, other):
        return -1

    def __gt__(self, other):
        return False

    def __lt__(self, other):
        return True

    def __le__(self, other):
        return True

    def __ge__(self, other):
        return False

    def __eq__(self, other):
        if other is MINUS_INFINITY:
            return True
        return False

    def __repr__(self):
        return '-inf'

    def __str__(self):
        return repr(self)


class _plus_infinity(float):
    def __cmp__(self, other):
        return +1

    def __gt__(self, other):
        return True

    def __lt__(self, other):
        return False

    def __le__(self, other):
        return False

    def __ge__(self, other):
        return True

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
    print '0 < -5', 0 < -5
    print '-inf < -5', MINUS_INFINITY < -5
    print '------------------'
    print '5 <= 0', 5 <= 0
    print '5 <= +inf ', 5 <= PLUS_INFINITY
    print '+inf >= 5 ', PLUS_INFINITY >= 5
    print '------------------'
    print '5 >= 0', 5 >= 0
    print '5 >= +inf ', 5 >= PLUS_INFINITY
    print '------------------'
    print '-5 >= 0', -5 >= 0
    print '-5 >= -inf', -5 >= MINUS_INFINITY
    print '-inf <= -5', MINUS_INFINITY <= -5
    print '------------------'
    print '5 <= 0', 5 <= 0
    print '5 <= -inf', 5 <= MINUS_INFINITY
    print '------------------'
    print '-inf <= -5 <= +inf', MINUS_INFINITY <= -5 <= PLUS_INFINITY
    print '------------------'
    print 'sort +inf -inf', sorted([PLUS_INFINITY, MINUS_INFINITY])
    print '0 == -inf', 0 == MINUS_INFINITY
    print '-inf == -inf', MINUS_INFINITY == MINUS_INFINITY
    print 'cmp -inf +inf', cmp(MINUS_INFINITY, PLUS_INFINITY)
    print 'cmp +inf -inf', cmp(PLUS_INFINITY, MINUS_INFINITY)
    print 'cmp 5 -inf', cmp(5, MINUS_INFINITY)