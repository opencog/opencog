import urllib2
import json

class TruthValue(object):
    def __init__(self,mean,count):
        self.mean = mean
        self.count = count
        self.confidence = count_to_confidence(count)
    
    def __repr__(self):
        return '(%s %s)' % (self.mean, self.count)
    
    def __cmp__(self,other):
        if type(other) != type(self):
            return cmp(type(self), type(other))
        else:
            return cmp((self.mean,self.count),(other.mean,other.count))

# If you change the constant, make sure to replace it in SimpleTruthValue.cc
def confidence_to_count(conf):
    KKK = 800.0
    conf = min(conf, 0.9999999)
    return KKK * conf / (1.0 - conf)

def count_to_confidence(count):
    KKK = 800.0
    return count / (count + KKK)

class Atom(object):
    def __init__(self,a,handle,type_name,name,out,tv,av):
        self._a = a
        self.h = handle
        # use str so it won't be unicode - that counts as a different
        # type from str so it will break things.
        self.type_name = str(type_name)
        self.t = str(type_name)
        self.name = str(name)
        self._out = out
        assert all([isinstance(o,int) for o in out])
        self.tv = tv
        self.av = av
    
    def is_node(self):
        return self.type_name.endswith("Node")
    
    def is_a(self,t):
        # HACK
        return t == self.t
    
    def __repr__(self):
        if self.is_node():
            return self.type_name+' '+self.name
        else:
            return '<'+self.type_name+' '+repr(
                [repr(atom) for atom in self.out])+'>'
    
    def getout(self):
        try:
            return self._outcache
        except (AttributeError):
            self._outcache = [self._a._get_atom_by_handle(h) for h in self._out]
            return self._outcache
    out = property(getout)
    
    def __cmp__(self,other):
        if type(other) != type(self):
            return cmp(type(self), type(other))
        else:
            return cmp(self.h, other.h)

class AtomTypeList(object):
    def __getattr__(self,type_name):
        return type_name

types = t = AtomTypeList()

def get_type_name(t):
    return t

def get_type(t):
    return t

class AtomSpace(object):
    def __init__(self):
        self._client = AtomSpaceJSONClient(self)
        # Updated by add() and get_atoms_by_type()
        #self._handle2atom = {}
        self._get_all_atoms()
    
    def get_atoms_by_type(self, type_name):
        #self._get_all_atoms()
        # HACK
        if type_name == 'Link':
            return [atom for atom in self._all_atoms
                if not atom.is_node()]
        elif type_name == 'Atom':
            return [atom for atom in self._all_atoms]
        else:
            return [atom for atom in self._all_atoms
                    if atom.type_name == type_name]
    
    def add(self, type_name, name='', out = []):
        ## TODO currently it just returns existing Atoms
        #existing = [a for a in self._all_atoms if
        #                a.type_name == type_name and
        #                a.name == name and
        #                a.out == out]
        #assert len(existing) == 1
        #return existing[0]
        assert False
        out_handles = [o.h for o in out]
        atom = Atom(self,None,type_name,name,out_handles,TruthValue(0,0),None)
        d = self._client._dict_from_atom(atom)
        # This makes sure to get the TruthValue etc
        h = self._client.add_atom(d)
        return self._get_atom_by_handle(h)
    
    def _get_all_atoms(self):
        print "REST: fetching all atoms"
        #all_handles = self._client.get_handles_by_type('Atom')
        #all_handles.sort()
        #all_atom_dicts = [self._client.get_atom(h) for h in all_handles]
        #return map(self._atom_from_dict, all_atom_dicts)

        # Newer version, where the REST API will send the full details
        # of all the Atoms in a single message - much faster
        all_atom_dicts = self._client._get_atoms_by_type('Atom')
        all_atoms = map(self._client._atom_from_dict, all_atom_dicts)
        
        self._all_atoms = all_atoms
        self._handle2atom = {atom.h:atom for atom in all_atoms}
        
        print "REST: relax"

    
    def _get_atom_by_handle(self,h):
        try:
            return self._handle2atom[h]
        except KeyError:
            new_atom = self._client._atom_from_dict(self._client.get_atom(h))
            self._handle2atom[h] = new_atom
            return new_atom

class AtomSpaceJSONClient(object):
    '''Connects a Python process to a separate CogServer process using the REST/JSON API.'''
    # An example JSON message:
    # {u'type':u'ConceptNode', u'name':'Pleasant Goat',
    # u'truthvalue':{u'simple':{'count':0,'str':0}}}
    # WARNING: The current CogServer REST implementation doesn't actually care
    # what type you ask for, and will just give you all atoms!
    
    def __init__(self, space, url='http://localhost:17034/rest/0.2/', autoflush=False):
        self._space = space
        self.url = url
        self.data = ""
        self.autoflush = autoflush
        
    def flush(self):
        if len(self.data) > 0:
            self.__send(self.data)
            self.data = ""
        
    def __send(self, section, data):
        conn = urllib2.urlopen(self.url + section, data)
        return conn.read()
    
    def _get_atoms_by_type(self,type_name):
        '''string -> IO int'''
        section = 'list?type='+type_name+'&subtype=1&max=100000'
        response = self.__send(section,None)
        #print section
        #print response
        res = json.loads(response)
        return res['result']
    
    def get_atom(self,handle):
        '''int -> IO dict'''
        section = 'atom/'+str(handle)
        response = self.__send(section,None)
        res = json.loads(response)
        return res

    def add_atom(self,data):
        out = json.dumps(data) + '\r\n'
        response = self.__send('atom/',out)
        res = json.loads(response)
        return res['handle']
    
    # NOTE: this shouldn't be necessary - the AS should find the right Atom
    def update_atom(self, handle, data):
        out = json.dumps(data) + '\r\n'
        return self.__send('atom/'+str(handle),out)

    def _atom_from_dict(self, d):
        assert isinstance(d,dict)
        return Atom(a = self._space,
                    handle = d['handle'],
                    type_name = d['type'],
                    name = d['name'],
                    out = d['outgoing'],
                    tv = self._tv_from_dict(d['truthvalue']),
                    av = (d['sti'],d['lti']))
    
    def _tv_from_dict(self, d):
        stv_dict = d['simple']
        return TruthValue(stv_dict['str'], stv_dict['count'])

    def _dict_from_atom(self,a):
        d = {
            'type':a.type_name,
            'name':a.name,
            'outgoing':a._out,
            'truthvalue':{'simple':{'str':a.tv.mean,'count':a.tv.count}},
            # TODO set sti and lti
            }

        return d
