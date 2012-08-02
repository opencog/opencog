class a(object):
    pass

class b(a):
    pass

print a.__subclasses__()

class c(a):
    pass

print a.__subclasses__()