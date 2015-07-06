# connector
* Directory to save the available Link connectors.

## Algorithm Files
* connect_simple.py: Connect every links.
* connect_conflict_random.py: Choose one link in each of conflict links.
* connect_conflict_viable.py: Make 2^k available(viable) new blend atoms if 
 there exists k conflicts.        

## Util Files
* base_connector.py: Abstract class to provide 'link_connect()' interface.
* connect_util.py: Utils for link connecting works.
* connector_finder.py: Provide connector instance for user.
* equal_link_key.py: Defines 'EqualLinkKey' concept which helps to find 
 duplicate links by provide custom unique key.
