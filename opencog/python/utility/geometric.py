__author__ = 'keyvan'


class Function(object):
    def __call__(self, x):
        return x


class LinearFunction(Function):
    def __init__(self, a, b):
            self.a = a
            self.b = b

    def __call__(self, x):
        return self.a * x + self.b