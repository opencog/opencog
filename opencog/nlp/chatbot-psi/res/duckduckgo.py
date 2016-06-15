# TODO: Attribution!

import urllib2
import json

def call_duckduckgo(query):
    response = urllib2.urlopen('http://api.duckduckgo.com/?q=' + query + '&format=json').read()

    result = json.loads(response)
    print '\nAbstractText:'
    print result['AbstractText']
    print 'Answer:'
    print result['Answer']
    print 'AnswerType:'
    print result['AnswerType']

import sys
call_duckduckgo(' '.join(sys.argv[1:]))
