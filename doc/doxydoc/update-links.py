#! /usr/bin/python

import    fileinput, string, sys
import    os
import    subprocess

# the tree of pages; each entry consists of a list
# with two elements: the depth (1 based) and the name
lay_tree = []

def    readLayout(lay_p):
    ''' Reads the layout.ini file into a global data structure - lay_tree

    lay_p indicates the path to the layout file.
    Returns False for error, True for success
    '''
    global lay_tree
    try:
        f = open( lay_p, "r" )
        layout = f.read()
        f.close()
    except Exception, e:
        print ( e )
        return False
    # the level of the last item that we've seen
    crt_lev = 1
    # iterate in each line
    for lay_line in layout.split("\n"):
        # text only
        lay_lcont = lay_line.strip()
        if ( len( lay_lcont ) == 0 ):
            continue
        # comments that start with #
        if ( lay_lcont.startswith( "#" ) ):
            continue
        # we're building the tree based on indentation
        it_level = 1
        for char_itr in lay_line:
            if ( char_itr != " " ):
                if ( char_itr != "\t" ):
                    break
            it_level += 1
        # make a sane value
        if ( it_level >= crt_lev + 1 ):
            # adding a child to last item
            it_level = crt_lev + 1
        # elif ( it_level == crt_lev ): # adding a sibling
        # else: # adding a sibling to some parent
        lay_tree.append([it_level,lay_lcont])
            
        # save our level for next iteration
        crt_lev = it_level

    return True

def    pageLink(page_name, link_text):
    ''' get the text to be used as a link to the page
    '''
    if ( page_name == "main" ):
        ret_txt = "\\ref index"
        if ( len(link_text) != 0 ):
            ret_txt += " \"" + link_text + "\""
        else:
            ret_txt += " \"Index\""
    else:
        ret_txt = "\\ref " + page_name
        if ( len(link_text) != 0 ):
            ret_txt += " \"" + link_text + "\""
    return ret_txt

def    updateAFile(dox_file_p,fileindex):
    ''' updats dynamic data between the markers

    The function reads the file and modified the content between the
    markers.
    '''
    print( "    Processing " + dox_file_p + "..." )
    # read old content
    try:
        f = open( dox_file_p, "r" )
        dox_cont = f.read()
        f.close()
    except Exception, e:
        print ( e )
        return False

    # we accumulate new content here
    new_cont = "\n"
    # first, the list of kids
    my_level = lay_tree[fileindex][0]
    i = fileindex + 1
    itr_max = len(lay_tree)
    while (i < itr_max):
        crt_level = lay_tree[i][0]
        # only kids and grandkids
        if (crt_level <= my_level):
            break
        if (crt_level == my_level+1):
            new_cont += "- \\subpage " + lay_tree[i][1] + "\n"
        else:
            new_cont += "\t"*(crt_level-my_level-1)
            new_cont += "- " + pageLink(lay_tree[i][1],"") + "\n"
        i = i + 1
        
    # now the next/previous navigation
    new_cont += "\n<TABLE width=\"100%\" border=\"0\"><TR>\n<TD>"
    if ( fileindex != 0 ):
        new_cont += pageLink(lay_tree[fileindex-1][1], "Previous")
    new_cont += "</TD>\n<TD width=\"100%\"></TD>\n<TD>"
    if ( fileindex != itr_max-1 ):
        new_cont += pageLink(lay_tree[fileindex+1][1], "Next")
    new_cont += "</TD>\n</TR></TABLE>\n"

    # locate markers
    while ( True ):
        # ignore the files that have no markers
        idx_st = dox_cont.find( "\\if MARKER_TREE_START" )
        if ( idx_st == -1 ): break
        idx_st = dox_cont.find( "\\endif", idx_st)
        if ( idx_st == -1 ): break
        idx_st += len( "\\endif" )
        idx_end = dox_cont.find( "\\if MARKER_TREE_END", idx_st )
        if ( idx_end == -1 ): break
        # we reach this point only if all the markers were found
        # do we really need an update of this file?
        if ( dox_cont[idx_st:idx_end] == new_cont ):
            print( "    ... skipped" )
            return True
        # now change the content
        dox_cont = dox_cont[:idx_st] + new_cont + dox_cont[idx_end:]
        # and save new content
        try:
            f = open( dox_file_p, "w" )
            f.write( dox_cont )
            f.close()
        except Exception, e:
            print ( e )
            return False
        break;

    print( "    ... done" )
    return True

def    main():
    ''' Main function that is called when the module is loaded by itself

    The function assumes that the documentation is in the same directory.
    '''
    global lay_tree
    # read the layout file
    lay_p = "layout.ini"
    if (readLayout(lay_p)==False):
        return -1
    # print( lay_tree )
    lay_tree.insert(0,[0,"main"])
    idx = 0
    for itr_lay in lay_tree:
        updateAFile(itr_lay[1]+".dox", idx)
        idx = idx + 1

    return 0

if __name__ == "__main__":
    sys.exit( main() )
