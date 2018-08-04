# connector
* Directory to save the available Link connectors.

## Algorithm Files
* connect_simple.py: Connect every links to new blends.
* connect_conflict_random.py: Connect every non-conflicted links, and connect
 every conflict links by selecting randomly.
* connect_conflict_viable.py: Connect every viable cases of conflicted links 
 to 2^k of available new blend atoms if there exists k conflicts.
* connect_conflict_interaction_information.py: Connect an one link set which has
 largest interaction information value.

## Util Files
* base_connector.py: Abstract class to provide 'link_connect()' interface.
* connect_util.py: Utils for connecting link.
* connector_finder.py:Provider class to make link connector instance.
* equal_link_key.py: Defines the custom unique key of link to evaluate whether 
 links are duplicate or not.
