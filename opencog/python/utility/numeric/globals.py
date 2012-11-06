__author__ = 'keyvan'

class _minus_infinity:
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

class _plus_infinity:
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

MINUS_INFINITY = _minus_infinity()
PLUS_INFINITY = _plus_infinity()
BIGGEST_UNSIGNED_INTEGER = 2**(32 - 1)