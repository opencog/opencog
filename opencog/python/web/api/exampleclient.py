__author__ = 'Cosmo Harrigan'

from requests import *
import json

# Define the API Endpoint - replace 127.0.0.1 with your server IP address if necessary
uri = 'http://127.0.0.1:5000/api/v1.0/'
headers = {'content-type': 'application/json'}

# Pretty print function for displaying JSON request/response information
def pprint(call, contents):
    print '\n' + call.request.method + ' ' + call.request.path_url + ':'
    print json.dumps(contents, indent=2)

####################################################################
# Example POST and GET requests to create and read Nodes and Links #
####################################################################
# POST a new node
truthvalue = {'type': 'simple', 'details': {'strength': 0.08, 'count': 0.2}}
atom = {'type': 'ConceptNode', 'name': 'giant_frog', 'truthvalue': truthvalue}
post_response = post(uri + 'atoms', data=json.dumps(atom), headers=headers)
post_result = post_response.json()['atoms']
pprint(post_response, post_result)
'''
POST /api/v1.0/atoms:
{
  "outgoing": [],
  "incoming": [],
  "name": "giant_frog",
  "truthvalue": {
    "type": "simple",
    "details": {
      "count": "0.20000000298023224",
      "confidence": "0.0002499375259503722",
      "strength": "0.07999999821186066"
    }
  },
  "attentionvalue": {
    "lti": 0,
    "sti": 0,
    "vlti": false
  },
  "handle": 57,
  "type": "ConceptNode"
}
'''

# GET the newly created node
handle_node_1 = post_result['handle']
get_response = get(uri + 'atoms/' + str(handle_node_1))
get_result = get_response.json()['atoms']
pprint(get_response, get_result)
'''
GET /api/v1.0/atoms/57:
{
  "outgoing": [],
  "incoming": [],
  "name": "giant_frog",
  "truthvalue": {
    "type": "simple",
    "details": {
      "count": "0.20000000298023224",
      "confidence": "0.0002499375259503722",
      "strength": "0.07999999821186066"
    }
  },
  "attentionvalue": {
    "lti": 0,
    "sti": 0,
    "vlti": false
  },
  "handle": 57,
  "type": "ConceptNode"
}
'''

# GET the newly created node by name
name = post_result['name']
get_response = get(uri + 'atoms?name=' + name)
get_result = get_response.json()['result']['atoms'][0]
pprint(get_response, get_result)
'''
GET /api/v1.0/atoms?name=giant_frog:
{
  "outgoing": [],
  "incoming": [],
  "name": "giant_frog",
  "truthvalue": {
    "type": "simple",
    "details": {
      "count": "0.20000000298023224",
      "confidence": "0.0002499375259503722",
      "strength": "0.07999999821186066"
    }
  },
  "attentionvalue": {
    "lti": 0,
    "sti": 0,
    "vlti": false
  },
  "handle": 57,
  "type": "ConceptNode"
}
'''

# GET the newly created node by name and type
type = post_result['type']
get_response = get(uri + 'atoms?name=' + name + '&type=' + type)
get_result = get_response.json()['result']['atoms'][0]
pprint(get_response, get_result)
'''
GET /api/v1.0/atoms?name=giant_frog&type=ConceptNode:
{
  "outgoing": [],
  "incoming": [],
  "name": "giant_frog",
  "truthvalue": {
    "type": "simple",
    "details": {
      "count": "0.20000000298023224",
      "confidence": "0.0002499375259503722",
      "strength": "0.07999999821186066"
    }
  },
  "attentionvalue": {
    "lti": 0,
    "sti": 0,
    "vlti": false
  },
  "handle": 57,
  "type": "ConceptNode"
}
'''

# POST another new node
truthvalue = {'type': 'simple', 'details': {'strength': 0.20, 'count': 0.5}}
atom = {'type': 'ConceptNode', 'name': 'animal', 'truthvalue': truthvalue}
post_response = post(uri + 'atoms', data=json.dumps(atom), headers=headers)
post_result = post_response.json()['atoms']
handle_node_2 = post_result['handle']
pprint(post_response, post_result)
'''
POST /api/v1.0/atoms:
{
  "outgoing": [],
  "incoming": [],
  "name": "animal",
  "truthvalue": {
    "type": "simple",
    "details": {
      "count": "0.5",
      "confidence": "0.0006246096454560757",
      "strength": "0.20000000298023224"
    }
  },
  "attentionvalue": {
    "lti": 0,
    "sti": 0,
    "vlti": false
  },
  "handle": 58,
  "type": "ConceptNode"
}
'''

# POST a new link between the two newly created nodes
truthvalue = {'type': 'simple', 'details': {'strength': 0.5, 'count': 0.4}}
atom = {'type': 'InheritanceLink', 'truthvalue': truthvalue, 'outgoing': [handle_node_1, handle_node_2]}
post_response = post(uri + 'atoms', data=json.dumps(atom), headers=headers)
post_result = post_response.json()['atoms']
pprint(post_response, post_result)
'''
POST /api/v1.0/atoms:
{
  "outgoing": [
    57,
    58
  ],
  "incoming": [],
  "name": "",
  "truthvalue": {
    "type": "simple",
    "details": {
      "count": "0.4000000059604645",
      "confidence": "0.0004997501382604241",
      "strength": "0.5"
    }
  },
  "attentionvalue": {
    "lti": 0,
    "sti": 0,
    "vlti": false
  },
  "handle": 59,
  "type": "InheritanceLink"
}
'''

#########################################
# Example PUT request to update an atom #
#########################################

truthvalue = {'type': 'simple', 'details': {'strength': 0.005, 'count': 0.8}}
attentionvalue = {'sti': 9, 'lti': 2, 'vlti': True}
atom_update = {'truthvalue': truthvalue, 'attentionvalue': attentionvalue}
put_response = put(uri + 'atoms/' + str(handle_node_1), data=json.dumps(atom_update), headers=headers)
put_result = put_response.json()['atoms']
pprint(post_response, post_result)

# todo: see if the av was updated

############################################
# Example DELETE request to delete an atom #
############################################

# todo: if you delete a node, does the handle disappear from an associated link's outgoing set?