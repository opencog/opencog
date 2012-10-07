#from opencog.atomspace import AtomSpace
#from attention.HebbianMining import HebbianMiningAgent
#
#h = HebbianMiningAgent()
#a = AtomSpace()
#
#h.run(a)
#a.print_list()
#from learning.bayesian_learning.network import ConditionalProbabilityTable
#
#a = ConditionalProbabilityTable()
#
#a[('A', True), ('B', False)] = 'hello'
#
#print a[('B', False), ('A', True)]

#a = {'A':True, 'B':False}
#b = {'B':False, 'A':True, a:'Kir'}
#print b[a]

#class A(object):
#    def __init__(self):
#        print 'in A'
#
#
#class B(A):
#    def __init__(self):
#        print 'in B'
#
#a = B()

#class A(object):
#    ret = True
#    def __lt__(self, other):
#        print 'inside lt', str(self)
#        return self.ret
#
#    def __gt__(self, other):
#        print 'inside gt', str(self)
#        return self.ret
#class B(A):
#    pass
#
#a, b = A(), B()
#
#if a < b > a:
#    print "hi!"

#class A(object):
#    def a(self):
#        print 'A'
#        return self
#class B(object):
#    def a(self):
#        print 'B'
#        return self
#    def b(self):
#        print 'B.b'
#class C(A,B):
#    pass
#
#C().a().b()

#class A(object):
#    a = 'a'
#    def __init__(self):
#        self.a = 'b'
#
#    def b(self):
#        return type(self).a
#
#class B(A):
#    a = 'b'
#
#a = A()
#b = B()
#print a.b()
#print b.b()

#class A(dict):
#    loci = [0,1,2]
#
#    @property
#    def loci(self):
#        return self.keys()
#
#a = A()
#print a.loci
#a['a'] = 123
#print a.loci
#a['b'] = 456
#print a.loci
#
#print A.loci
#type(a).new = 'new_var'
#print A.new

def a(a,(b,c)):
    print a,b,c
a(1,(2,3))