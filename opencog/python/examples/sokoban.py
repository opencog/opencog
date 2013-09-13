"""
Created on 5 Jun, 2012

@author: keyvan

Usage Instructions

The following methods can be used for generating Sokoban atoms from a file/url:
- add_socoban_level_to_atomspace_from_file
- add_socoban_level_to_atomspace_from_url

Use levels from http://www.sourcecode.se/sokoban/levels.php

"""
__author__ = 'keyvan-m-sadeghi'

from xml.dom.minidom import parseString
from opencog.atomspace import AtomSpace, TruthValue, Atom
from opencog.atomspace import types

def add_socoban_level_to_atomspace_from_file(atomspace, file_path, level_index):
    _add_globals_to_atomspace(atomspace)
    levels = generate_levels_from_file(file_path)  
    levels[level_index].append_to_atomspace(atomspace)

def add_socoban_level_to_atomspace_from_url(atomspace, url, level_index):
    """
    Url should point to 'http://www.sourcecode.se/sokoban/' database
    e.g. 'httpd://www.sourcecode.se/sokoban/dnldlevel.php?LevelFile=100Boxes.slc'
    """
    _add_globals_to_atomspace(atomspace)
    levels = generate_levels_from_url(url)  
    levels[level_index].append_to_atomspace(atomspace)

def _add_globals_to_atomspace(atomspace):
    global TRUTH_VALUE, TIME, BLOCK, MOVABLE_BLOCK, MOVABLE_OBJECT, SOKOBAN_GOAL
    TRUTH_VALUE = TruthValue(1,1)
    TIME = atomspace.add_node(types.TimeNode, '0')
    BLOCK = atomspace.add_node(types.ConceptNode, 'block')
    MOVABLE_BLOCK = atomspace.add_node(types.ConceptNode, 'MovableBlock')
    MOVABLE_OBJECT = atomspace.add_node(types.ConceptNode, 'MovableObject')
    SOKOBAN_GOAL = atomspace.add_node(types.ConceptNode, 'SokobanGoal')
    atomspace.add_link(types.InheritanceLink, [MOVABLE_BLOCK, MOVABLE_OBJECT], TRUTH_VALUE)

def generate_levels_from_file(xml_file_path):
    return _generate_levels(open(xml_file_path,'r'))

def generate_levels_from_url(url):
    import urllib2
    return _generate_levels(urllib2.urlopen(url))

def _generate_levels(open_xml_stream):
    data = open_xml_stream.read()
    open_xml_stream.close()
    levelTags = parseString(data).getElementsByTagName('Level')
    levels = []
    for levelTag in levelTags:
        levels.append(Level(levelTag))
    
    return levels

class Block:

    def __init__(self, x = 0, y = 0, charachter = ' '):
        self.x, self.y = x, y
        self.character = charachter

    def append_to_atomspace(self, atomspace):
        self.atomspace = atomspace
        for method in self._method_by_charachter[self.character]:
            node = method(self) # This can only be done in python!
            if node is not None:
                self._give_position_to_node(node)
    
    def _add_to_atomspace_as_player(self):
        return self.atomspace.add_node(types.PetNode, 'OAC_MrSokobanRobot')
        
    def _add_to_atomspace_as_box(self):
        box = self.atomspace.add_node(types.ObjectNode, 'id_movable_block' +
                                      str(self.x) + '_' + str(self.y) + '_99')
        self.atomspace.add_link(types.InheritanceLink, [box, MOVABLE_BLOCK], TRUTH_VALUE)
        return box

    def _add_to_atomspace_as_wall(self):
        wall = self.atomspace.add_node(types.ObjectNode, 'id_CHUNK_0_0_0_BLOCK' +
                                      str(self.x) + '_' + str(self.y) + '_99')
        self.atomspace.add_link(types.InheritanceLink, [wall, BLOCK], TRUTH_VALUE)
        return wall
    
    def _add_to_atomspace_as_goal(self):
        goal = self.atomspace.add_node(types.ObjectNode, 'id_SokobanGoal_' +
                                      str(self.x) + '_' + str(self.y) + '_99')
        
        self.atomspace.add_link(types.InheritanceLink, [goal, SOKOBAN_GOAL], TRUTH_VALUE)
        return goal
    
    def _do_nothing(self):
        return None
    
    _method_by_charachter = {'@':(_add_to_atomspace_as_player,),
                             '+':(_add_to_atomspace_as_player, _add_to_atomspace_as_goal),
                             '$':(_add_to_atomspace_as_box,),
                             '*':(_add_to_atomspace_as_box,  _add_to_atomspace_as_goal),
                             '#':(_add_to_atomspace_as_wall,),
                             '.':(_add_to_atomspace_as_goal,),
                             ' ':(_do_nothing,)
                             }
    
    def _give_position_to_node(self, node):
        atomspace, truth_value, time = self.atomspace, TRUTH_VALUE, TIME
              
        x = atomspace.add_node(types.NumberNode, str(self.x + 1.5))
        y = atomspace.add_node(types.NumberNode, str(self.y + 1.5))
        z = atomspace.add_node(types.NumberNode, '99.5')
        
        list = atomspace.add_link(types.ListLink, [node, x, y, z])
        predicate = atomspace.add_node(types.PredicateNode, "AGISIM_position")
        
        evaluation = atomspace.add_link(types.EvaluationLink, [predicate, list])
        at_time = atomspace.add_link(types.AtTimeLink, [time, evaluation], truth_value)
        atomspace.add_link(types.LatestLink, [at_time])
        
        
    def __repr__(self):
        return "'" + self.character + "' at" + ' X=' + str(self.x) + ' Y=' + str(self.y)
        

class Level:
    '''
    classdocs
    '''
    def __init__(self, xml_tag): # xml_tag specs at:
                                # http://epydoc.sourceforge.net/stdlib/xml.dom.minidom.Element-class.html
        '''
        Constructor
        '''
        self.Height = int(xml_tag.getAttribute('Height'))
        self.Width = int(xml_tag.getAttribute('Width'))
        self.Rows = []
        xmlRows = xml_tag.getElementsByTagName('L')
        
        for i, xmlRow in enumerate(xmlRows):
            blockRow = []
            rowString = str(xmlRow.toxml().\
                replace('<L>','').replace('</L>',''))
            
            for j in range(self.Width):
                if j == len(rowString):
                    rowString += ' '
                character = rowString[j]
                if character == '@' or character == '+':
                    self.PlayerCoordinates = (i, j)
                block = Block(i, j, character)
                blockRow.append(block)
            
            self.Rows.append(blockRow)
            
    def append_to_atomspace(self, atomspace):
        for block in self:
            block.append_to_atomspace(atomspace)
                    
    def __iter__(self):
        for row in self.Rows:
            for block in row:
                yield block
    
    def __getitem__(self, index):
        return self.Rows[index]

if __name__ == '__main__':
    atomspace = AtomSpace()
    add_socoban_level_to_atomspace_from_url(atomspace,
        'http://www.sourcecode.se/sokoban/dnldlevel.php?LevelFile=100Boxes.slc', 0)
    atomspace.print_list()
