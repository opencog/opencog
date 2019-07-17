import urllib2
import json

class JSONClient(object):
    
    def __init__(self, url='http://127.0.0.1:8080/workspace0', autoflush=False):
        self.url = url
        self.data = ""
        self.autoflush = autoflush
        
    def flush(self):
        if len(self.data) > 0:
            self.__send(self.data)
            self.data = ""
        
    def __send(self, data):
        conn = urllib2.urlopen(self.url+ '?operation=updateGraph', data)
        return conn.read()
        
    def add_node(self, id, flush=True, **attributes):
        self.data += json.dumps({"an":{id:attributes}}) + '\r\n'
        if(self.autoflush): self.flush()
    
    def delete_node(self, id):
        self.__send(json.dumps({"dn":{id:{}}}))
    
    def change_node(self, id, **attributes):
        self.data += json.dumps({"cn":{id:attributes}}) + '\r\n'
        if(self.autoflush): self.flush()
    
    def add_edge(self, id, source, target, directed=True, **attributes):
        attributes['source'] = source
        attributes['target'] = target
        attributes['directed'] = directed
        self.data += json.dumps({"ae":{id:attributes}}) + '\r\n'
        if(self.autoflush): self.flush()
    
    def delete_edge(self, id):
        self.__send(json.dumps({"de":{id:{}}}))

    def change_edge(self, id, **attributes):
        self.data += json.dumps({"ce":{id:attributes}}) + '\r\n'
        if(self.autoflush): self.flush()

    def clean(self):
        self.__send(json.dumps({"dn":{"filter":"ALL"}}))
