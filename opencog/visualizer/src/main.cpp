/*
 * opencog/visualizer/src/Main.cpp
 *
 * Copyright (C) 2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Erwin Joosten <eni247@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "../include/AtomTypes.h"
#include "../include/AtomSpaceInterface.h"
#include "../include/Graph.h"
#include "../include/Vertex.h"

#include <stdlib.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <string.h>
#include <limits>

/*
	To build in eclipse:
		-download eclipse CDT for c/c++ from the eclipse web site
		-add arguments for location of atom types and cogserver server
 		-add /usr/include/gtk-3.0 and /usr/include/glib-2.0 and /usr/include/cairo to include directories
 		-add `pkg-config --cflags gtk+-3.0` to settings/compiler
 		-add `pkg-config --libs gtk+-3.0` to settings/linker/miscelaneous and put ${FLAGS} at the end. see http://stackoverflow.com/questions/8493403/eclipse-and-gtkmm-undefined-reference-to
		-add pthread, boost_system and boost_filesystem to settings/linker/libraries

   More information:
	-great introductory talk about opencog (talks about atomspace from 48:30) http://www.youtube.com/watch?v=x18yaOXBSQA&feature=related
	-more in-depth talk about opencog http://agi-school.org/2009/dr-joel-pitt-with-dr-ben-goertzel-opencog-software-framework
	-OpenCog Wiki entry on atomspace http://wiki.opencog.org/w/AtomSpace
	-OpenCog Wiki entry on web interface http://wiki.opencog.org/w/Web_interface
	-source of OpenCog atomspace http://bazaar.launchpad.net/~opencog-dev/opencog/trunk/files/head:/opencog/atomspace/
	-http headers http://net.tutsplus.com/tutorials/other/http-headers-for-dummies/
	-get opencog
   	    bzr branch lp:opencog
		bzr branch opencog ochack (Make a local scratch copy)
	-compile opencog
   	    cd ochack; mkdir bin; cd bin
		cmake ..
		make -k (skip things that can't be compiled)
	-launch cogserver
		cd $HOME/ochack; bin/opencog/server/cogserver -c lib/opencog.conf
   	-location of schema file
   	 	~/ochack/opencog/scm/febcorpus_updated.scm
	-telnet to cogserver
		telnet localhost 17001

   TODO
   -keep connection alive (probably requires changes to the server because it seems to ignore http 1.1 keep-alive header)
   -details pane: allow updating of sti, lti and truthvalue (implemented but doesn't work... might be a problem with the POST or a bug in the server?)
   -alter thickness/color of lines according to strength?
   -alter symbol size/color according to STI/LTI?
   -add zoom level (changes the shape size and the repulsive force)?
*/

GtkWidget *window;
GtkWidget *notebook;
GtkWidget *drawingArea;
GtkWidget *entryAtomName;
GtkWidget *entryAtomUUID;
GtkWidget *spinbuttonAtomLevels;
GtkWidget *comboboxAtomType;
GtkWidget *checkboxAtomIncludeSubtypes;
GtkWidget *comboboxNodeType;
GtkWidget *checkboxNodeIncludeSubtypes;
GtkWidget *entryNodeMinimumSTI;
GtkWidget *entryNodeMinimumLTI;
GtkWidget *entryNodeMinimumStrength;
GtkWidget *entryNodeMinimumConfidence;
GtkWidget *comboboxLinkType;
GtkWidget *checkboxLinkIncludeSubtypes;
GtkWidget *entryLinkMinimumSTI;
GtkWidget *entryLinkMinimumLTI;
GtkWidget *entryLinkMinimumStrength;
GtkWidget *entryLinkMinimumConfidence;
GtkWidget *entrySelectedName;
GtkWidget *entrySelectedType;
GtkWidget *entrySelectedUUID;
GtkWidget *entrySelectedSTI;
GtkWidget *entrySelectedLTI;
GtkWidget *entrySelectedStrength;
GtkWidget *entrySelectedConfidence;
GtkWidget *entrySelectedTruthValue;
GtkWidget *buttonNextPage;
GtkWidget *comboboxSort;

AtomTypes atomTypes;
AtomSpaceInterface atomSpaceInterface(&atomTypes);
Graph graph(&atomSpaceInterface);

bool forceDisplayUpdate = false;
Vertex* draggingVertex=NULL;
bool draggingVertexHasMoved=false;
Vertex* selectedVertex=NULL;
bool verbose = true; //show help messages
int skipNextPage=0;

static void ShowError(const gchar *errorMessage)
{
	GtkWidget *dialog = NULL;

    dialog = gtk_message_dialog_new (GTK_WINDOW (window), GTK_DIALOG_MODAL, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, errorMessage);
    gtk_window_set_title (GTK_WINDOW (dialog), "OpenCog Atomspace Visualizer");
    gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
    gtk_dialog_run (GTK_DIALOG (dialog));
    gtk_widget_destroy (dialog);
}

static void ShowUnknownError()
{
	ShowError(" I'm sorry, Dave. I'm afraid I can't do that. (unknown error)");
}

static void ShowWarning (const gchar *message)
{
    GtkWidget *dialog = NULL;

    dialog = gtk_message_dialog_new (GTK_WINDOW (window), GTK_DIALOG_MODAL, GTK_MESSAGE_WARNING, GTK_BUTTONS_CLOSE, message);
    gtk_window_set_title (GTK_WINDOW (dialog), "OpenCog Atomspace Visualizer");
    gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
    gtk_dialog_run (GTK_DIALOG (dialog));
    gtk_widget_destroy (dialog);
}

static void ShowInfo (const gchar *message)
{
	if(!verbose)
		return;

    GtkWidget *dialog = NULL;

    dialog = gtk_message_dialog_new (GTK_WINDOW (window), GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE, message);
    gtk_window_set_title (GTK_WINDOW (dialog), "OpenCog Atomspace Visualizer");
    gtk_window_set_position (GTK_WINDOW (dialog), GTK_WIN_POS_CENTER);
    gtk_dialog_run (GTK_DIALOG (dialog));
    gtk_widget_destroy (dialog);
}

AtomFilter* GetAtomFilter()
{
    AtomFilter* atomFilter = new AtomFilter();
    atomFilter->name = gtk_entry_get_text(GTK_ENTRY(entryAtomName));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryAtomUUID)))==0)
    	atomFilter->uuid =-numeric_limits<unsigned long>::max();
    else
    	atomFilter->uuid = lexical_cast<UUID>(gtk_entry_get_text(GTK_ENTRY(entryAtomUUID)));
    atomFilter->type = gtk_combo_box_get_active (GTK_COMBO_BOX(comboboxAtomType));
    atomFilter->includeSubtypes = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(checkboxAtomIncludeSubtypes));
    atomFilter->sortOrder = gtk_combo_box_get_active (GTK_COMBO_BOX(comboboxSort));
    return atomFilter;
}

NodeFilter* GetNodeFilter()
{
    NodeFilter* nodeFilter = new NodeFilter();
    nodeFilter->type = atomTypes.ConvertNodeTypeToAtomType(gtk_combo_box_get_active (GTK_COMBO_BOX(comboboxNodeType)));
    nodeFilter->includeSubtypes = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(checkboxNodeIncludeSubtypes));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumSTI)))==0)
    	nodeFilter->minimumSTI=-numeric_limits<short>::max();
    else
    	nodeFilter->minimumSTI=lexical_cast<short>(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumSTI)));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumLTI)))==0)
    	nodeFilter->minimumLTI=-numeric_limits<short>::max();
    else
        nodeFilter->minimumLTI=lexical_cast<short>(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumLTI)));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumStrength)))==0)
    	nodeFilter->minimumStrength=-numeric_limits<double>::max();
    else
        nodeFilter->minimumStrength=lexical_cast<double>(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumStrength)));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumConfidence)))==0)
    	nodeFilter->minimumConfidenceValue=-numeric_limits<double>::max();
    else
        nodeFilter->minimumConfidenceValue=lexical_cast<double>(gtk_entry_get_text(GTK_ENTRY(entryNodeMinimumConfidence)));

    return nodeFilter;
}

LinkFilter* GetLinkFilter()
{
    LinkFilter* linkFilter = new LinkFilter();
    linkFilter->type = atomTypes.ConvertLinkTypeToAtomType(gtk_combo_box_get_active (GTK_COMBO_BOX(comboboxLinkType)));
    linkFilter->includeSubtypes = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(checkboxLinkIncludeSubtypes));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumSTI)))==0)
    	linkFilter->minimumSTI=-numeric_limits<short>::max();
    else
        linkFilter->minimumSTI=lexical_cast<short>(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumSTI)));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumLTI)))==0)
    	linkFilter->minimumLTI=-numeric_limits<short>::max();
    else
        linkFilter->minimumLTI=lexical_cast<short>(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumLTI)));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumStrength)))==0)
    	linkFilter->minimumStrength=-numeric_limits<double>::max();
    else
        linkFilter->minimumStrength=lexical_cast<double>(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumStrength)));
    if(strlen(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumConfidence)))==0)
    	linkFilter->minimumConfidenceValue=-numeric_limits<double>::max();
    else
    	linkFilter->minimumConfidenceValue=lexical_cast<double>(gtk_entry_get_text(GTK_ENTRY(entryLinkMinimumConfidence)));

    return linkFilter;

}

void ConvertMouseToPosition(GdkEventButton* event, int &row, int &col)
{
	int x=event->x;
	int y=event->y;
    double width = gtk_widget_get_allocated_width(drawingArea);
    double height = gtk_widget_get_allocated_height(drawingArea);

	double xFactor = width / (double)graph.positions.maxCol;
	double yFactor = height / (double)graph.positions.maxRow;
	row = (int)(y / yFactor + 0.5);
	if (row < 0)
		row = 0;
	if (row >= graph.positions.maxRow)
		row = graph.positions.maxRow - 1;
	col = (int)(x / xFactor + 0.5);
	if (col < 0)
		col = 0;
	if (col >= graph.positions.maxCol)
		col = graph.positions.maxCol - 1;
}

Vertex* ConvertMouseToNearestVertex(GdkEventButton* event)
{
	int row;
	int col;
	ConvertMouseToPosition(event, row, col);

	Vertex* closestVertex = graph.positions.FindClosestVertex(row, col);
	return closestVertex;
}

void StartNewGraph(vector<Vertex*> vertices)
{
	draggingVertex=NULL;
	selectedVertex=NULL;

	GdkWindow *gdkWindow = gtk_widget_get_window(window); //TODO still doesn't work...
	GdkCursor *gdkCursor = gdk_cursor_new(GDK_WATCH);
	gdk_window_set_cursor(gdkWindow,gdkCursor);

	graph = Graph(&atomSpaceInterface);
	graph.AddVertices(vertices);

	for(int i=0;i<vertices.size();i++)
		vertices[i]->positionLocked = TRUE;

	if(vertices.size()==1)
	{
		int expansionDepth = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spinbuttonAtomLevels));
		graph.ExpandVertex(vertices[0], expansionDepth, GetNodeFilter(), GetLinkFilter());
	}

	forceDisplayUpdate = true;

	gdk_window_set_cursor(gdkWindow,NULL);
}

void ShowDetails(Vertex* vertex, bool switchPane=false)
{
	selectedVertex=vertex;
	forceDisplayUpdate=true;

	if(switchPane)
		gtk_notebook_set_current_page(GTK_NOTEBOOK(notebook), 3);

	gtk_entry_set_text(GTK_ENTRY(entrySelectedName),vertex->name.c_str());
	gtk_entry_set_text(GTK_ENTRY(entrySelectedType),
			atomTypes.atomTypeNames[vertex->type].c_str());
	gtk_entry_set_text(GTK_ENTRY(entrySelectedUUID), lexical_cast<string>(vertex->uuid).c_str());
	if(vertex->STI==-numeric_limits<short>::max())
		gtk_entry_set_text(GTK_ENTRY(entrySelectedSTI), "(undefined)");
	else
		gtk_entry_set_text(GTK_ENTRY(entrySelectedSTI), lexical_cast<string>(vertex->STI).c_str());
	if(vertex->LTI==-numeric_limits<short>::max())
		gtk_entry_set_text(GTK_ENTRY(entrySelectedLTI), "(undefined)");
	else
		gtk_entry_set_text(GTK_ENTRY(entrySelectedLTI), lexical_cast<string>(vertex->LTI).c_str());
	if(vertex->strength==-numeric_limits<double>::max())
		gtk_entry_set_text(GTK_ENTRY(entrySelectedStrength), "(undefined)");
	else
		gtk_entry_set_text(GTK_ENTRY(entrySelectedStrength), lexical_cast<string>(vertex->strength).c_str());
	if(vertex->confidenceValue==-numeric_limits<double>::max())
		gtk_entry_set_text(GTK_ENTRY(entrySelectedConfidence), "(undefined)");
	else
		gtk_entry_set_text(GTK_ENTRY(entrySelectedConfidence), lexical_cast<string>(vertex->confidenceValue).c_str());
	gtk_entry_set_text(GTK_ENTRY(entrySelectedTruthValue), vertex->truthValue.c_str());
}

gboolean drawingArea_button_press (GtkWidget* widget, GdkEventButton * event, GdkWindowEdge edge)
{
	try
	{
		if (event->type == GDK_BUTTON_PRESS && event->button == 1) //left button down start dragging
		{
			Vertex* closestVertex = ConvertMouseToNearestVertex(event);
			if (closestVertex != NULL)
			{
				draggingVertex = closestVertex;
				draggingVertexHasMoved=false;
			}
		}

		if (event->type == GDK_BUTTON_RELEASE && event->button == 1) //left button up stop dragging and expand/collapse
		{
			draggingVertex = NULL;

			Vertex* closestVertex = ConvertMouseToNearestVertex(event);
			if (closestVertex != NULL && !draggingVertexHasMoved)
			{
				if(closestVertex->isEllipsis)
				{
					Vertex *owningVertex = closestVertex->connectedVertices[0];
					owningVertex->isEllipsisClicked=true;
					owningVertex->DisconnectVertex(closestVertex);
					graph.RemoveVertex(closestVertex);
					graph.ExpandVertex(owningVertex, 1, GetNodeFilter(), GetLinkFilter());
				}
				else
			    {
					if (closestVertex->isExpanded)
						graph.CollapseVertex(closestVertex);
					else
						graph.ExpandVertex(closestVertex, 1, GetNodeFilter(), GetLinkFilter());
				}
				forceDisplayUpdate = true;
			}
		}

		if (event->type == GDK_2BUTTON_PRESS && event->button == 1) //left button double click
		{
		}

		if (event->type == GDK_BUTTON_PRESS && event->button == 3) //right button down recenter graph on the clicked node
		{
			draggingVertex = NULL;
			Vertex* closestVertex = ConvertMouseToNearestVertex(event);
			if (closestVertex != NULL && !closestVertex->isEllipsis)
			{
				Vertex *copyOfclosestVertex = new Vertex();
				copyOfclosestVertex->CopyVertex(*closestVertex);
				vector<Vertex*> vertices;
				vertices.push_back(copyOfclosestVertex);
				StartNewGraph(vertices);
				ShowDetails(closestVertex, true);
			}
		}
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}
    return FALSE;
}

gboolean drawingArea_motion_notify (GtkWidget* widget, GdkEventButton * event, GdkWindowEdge edge)
{
	try
	{
		if (draggingVertex == NULL)
		{
			Vertex* closestVertex = ConvertMouseToNearestVertex(event);
			if (closestVertex != NULL)
			{
				ShowDetails(closestVertex);
			}
		}
		else
		{
			int row;
			int col;
			ConvertMouseToPosition(event, row, col);
			if (graph.positions.GetAt(row, col) == draggingVertex)
			{
				//not moved
			}
			else
			{
				if (graph.positions.GetAt(row, col) != NULL)
					graph.positions.FindNearestFreePosition(row, col, &row, &col);
				graph.positions.MoveTo(draggingVertex, row, col);
				forceDisplayUpdate = true;
				draggingVertex->positionLocked = true;
				draggingVertexHasMoved=true;
			}
		}
	}
	catch(exception& e)
	{
		draggingVertex = NULL;
		ShowError(e.what());
	}
	catch(...)
	{
		draggingVertex = NULL;
		ShowUnknownError();
	}

    return FALSE;
}

gboolean drawingArea_draw (GtkWidget *widget, cairo_t *cr, gpointer data)
{
	try
	{
		const GdkRGBA red{1.0,0.0,0.0,1.0};
		const GdkRGBA blue{0.0,0.0,1.0,1.0};
		const GdkRGBA black{0.0,0.0,0.0,1.0};
		const GdkRGBA white{1.0,1.0,1.0,1.0};

		const guint symbolRadius = 10;

		guint width = gtk_widget_get_allocated_width(drawingArea);
		guint height = gtk_widget_get_allocated_height(drawingArea);

		double xFactor = (double)width / (double)graph.positions.maxCol;
		double yFactor = (double)height / (double)graph.positions.maxRow;

		for (int i = 0; i < graph.vertices.vertices.size(); i++)
		{
			Vertex* vertex = graph.vertices.vertices[i];
			double x = (double)vertex->col * xFactor;
			double y = (double)vertex->row * yFactor;

			for (int j = 0; j < vertex->connectedVertices.size(); j++)
			{
				Vertex* connectedVertex = vertex->connectedVertices[j];
				if (vertex->uuid <= connectedVertex->uuid) //avoid drawing edges twice
				{
					cairo_move_to (cr, x, y);
					cairo_line_to (cr, (guint)((double)connectedVertex->col * xFactor), (guint)((double)connectedVertex->row * yFactor));
					if(vertex==selectedVertex || connectedVertex==selectedVertex)
						gdk_cairo_set_source_rgba (cr, &red);
					else
						gdk_cairo_set_source_rgba (cr, &black);
					cairo_stroke(cr);
				}
			}
		}

		for (int i = 0; i < graph.vertices.vertices.size(); i++)
		{
			Vertex* vertex = graph.vertices.vertices[i];
			double x = (double)vertex->col * xFactor;
			double y = (double)vertex->row * yFactor;

			if (vertex->isEllipsis)
				cairo_arc (cr, x, y, symbolRadius, 0.16 * G_PI, 1.83 * G_PI);
			else if (vertex->isNode)
				cairo_rectangle (cr, x-symbolRadius, y-symbolRadius, 2*symbolRadius, 2*symbolRadius);
			else
				cairo_arc (cr, x, y, symbolRadius, 0, 2 * G_PI);

			gdk_cairo_set_source_rgba (cr, &white);
			cairo_fill_preserve (cr);

			if(vertex==selectedVertex)
				gdk_cairo_set_source_rgba (cr, &red);
			else if (vertex->isExpanded)
				gdk_cairo_set_source_rgba (cr, &blue);
			else
				gdk_cairo_set_source_rgba (cr, &black);
			cairo_stroke(cr);

			gdk_cairo_set_source_rgba (cr, &black);
			cairo_select_font_face (cr, "Georgia", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
			cairo_set_font_size (cr, symbolRadius*4/3);
			cairo_move_to (cr,x+symbolRadius+3,y+symbolRadius/2);
			cairo_show_text (cr, vertex->name.c_str());

			cairo_move_to (cr,x-symbolRadius+1,y+symbolRadius/2);
			if (vertex->isEllipsis)
				cairo_show_text (cr, " ...");
			else
				cairo_show_text (cr, atomTypes.atomTypeSymbols[vertex->type].c_str());

			cairo_stroke(cr);
		}
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}

    return FALSE;
}

static void buttonSearch_clicked (GtkWidget *wid, GtkWidget *window)
{
	try
	{
		vector<Vertex*> foundVertices;
		bool isFinished=true;
		atomSpaceInterface.SearchAtom(GetAtomFilter(), foundVertices, isFinished, 0);
		StartNewGraph(foundVertices);

		if (foundVertices.size() == 0)
		{
			gtk_widget_hide(buttonNextPage);
			skipNextPage=0;
			ShowInfo("No atoms were found.");
		}
		else
		{
			if (foundVertices.size() == 1)
			{
				gtk_widget_hide(buttonNextPage);
				skipNextPage=0;
				ShowDetails(foundVertices[0], true);
			}
			else
				if(isFinished)
				{
					gtk_widget_hide(buttonNextPage);
					skipNextPage=0;
					ShowInfo("More than one atom was found. Right click on the one you want.");
				}
				else
				{
					gtk_widget_show(buttonNextPage);
					skipNextPage = foundVertices.size();
					ShowInfo("There are too many results to display at once. Click Next page to show more results.");
				}
		}
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}


}

static void buttonNextPage_clicked (GtkWidget *widget, GtkWidget *win)
{
	try
	{
		vector<Vertex*> foundVertices;
		bool isFinished=true;
		atomSpaceInterface.SearchAtom(GetAtomFilter(), foundVertices, isFinished,skipNextPage);
		StartNewGraph(foundVertices);

		if (foundVertices.size() == 0)
		{
			gtk_widget_hide(buttonNextPage);
			skipNextPage=0;
			ShowInfo("No more results were found.");
		}
		else
		{
			if(isFinished)
			{
				gtk_widget_hide(buttonNextPage);
				skipNextPage=0;
			}
			else
			{
				gtk_widget_show(buttonNextPage);
				skipNextPage += foundVertices.size();
			}
		}
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}
}

static void buttonUpdate_clicked (GtkWidget *widget, GtkWidget *win)
{
	try
	{
		draggingVertex = NULL;

		UUID uuid=lexical_cast<UUID>(gtk_entry_get_text(GTK_ENTRY(entrySelectedUUID)));
		short lti=lexical_cast<short>(gtk_entry_get_text(GTK_ENTRY(entrySelectedLTI)));
		short sti=lexical_cast<short>(gtk_entry_get_text(GTK_ENTRY(entrySelectedSTI)));
		string truthValue=gtk_entry_get_text(GTK_ENTRY(entrySelectedTruthValue));
		atomSpaceInterface.UpdateAtom(uuid, lti, sti, truthValue);
		ShowInfo("Update successful.");
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}
}

static gint entryAtomName_keypress( GtkWidget *widget, GdkEventKey *event, gpointer func_data)
{
  switch(event->keyval)
  {
    case GDK_KEY_Return:
    case GDK_KEY_KP_Enter:
    {
    	buttonSearch_clicked (widget, window);
    	return TRUE;
    }
  }
  return FALSE;
}

static gint entryAtomUUID_keypress( GtkWidget *widget, GdkEventKey *event, gpointer func_data)
{
  switch(event->keyval)
  {
    case GDK_KEY_Return:
    case GDK_KEY_KP_Enter:
    {
    	buttonSearch_clicked (widget, window);
    	return TRUE;
    }
  }
  return FALSE;
}

static gboolean timer_tick(GtkWidget *widget)
{
	try
	{
		if(forceDisplayUpdate)
		{
			forceDisplayUpdate = graph.OptimizeLayout();
			gtk_widget_queue_draw(window);
		}
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}

    return TRUE;
}

gboolean window_configure(GtkWindow *window, GdkEvent *event, gpointer data)
{
    forceDisplayUpdate=true;
    return FALSE;
}

void BuildWindow()
{
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_container_set_border_width (GTK_CONTAINER (window), 8);
    gtk_window_set_title (GTK_WINDOW (window), "OpenCog Atomspace Visualizer 0.4 BETA");
    gtk_window_set_position (GTK_WINDOW (window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size (GTK_WINDOW (window), 1024, 768);
    gtk_widget_realize (window);
    g_signal_connect (window, "destroy", gtk_main_quit, NULL);
    gtk_widget_add_events (window, GDK_CONFIGURE);
    g_signal_connect(window, "configure-event", G_CALLBACK(window_configure), NULL);

    GtkWidget *vbox = gtk_vbox_new (TRUE, 6);
    gtk_box_set_homogeneous (GTK_BOX (vbox),FALSE);
    gtk_container_add (GTK_CONTAINER (window), vbox);

    entryAtomName = gtk_entry_new ();
    gtk_widget_set_events (entryAtomName, GDK_KEY_PRESS_MASK);
    g_signal_connect(entryAtomName, "key-press-event", G_CALLBACK(entryAtomName_keypress), NULL);

    entryAtomUUID = gtk_entry_new ();
    gtk_widget_set_events (entryAtomUUID, GDK_KEY_PRESS_MASK);
    g_signal_connect(entryAtomUUID, "key-press-event", G_CALLBACK(entryAtomUUID_keypress), NULL);

    GtkWidget *buttonSearch = gtk_button_new_with_label ("Search");
    g_signal_connect (G_OBJECT (buttonSearch), "clicked", G_CALLBACK (buttonSearch_clicked), (gpointer) window);

    spinbuttonAtomLevels = gtk_spin_button_new_with_range (0.0,5.0,1.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(spinbuttonAtomLevels),1.0);

    comboboxAtomType = gtk_combo_box_text_new();

    checkboxAtomIncludeSubtypes = gtk_check_button_new_with_label("Include Subtypes");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(checkboxAtomIncludeSubtypes), TRUE);

    comboboxSort = gtk_combo_box_text_new();

    buttonNextPage = gtk_button_new_with_label ("Next page");
    g_signal_connect (G_OBJECT (buttonNextPage), "clicked", G_CALLBACK (buttonNextPage_clicked), (gpointer) window);

    GtkWidget *tableSearch = gtk_table_new(2, 8, TRUE);
    gtk_table_set_row_spacings(GTK_TABLE(tableSearch), 2);
    gtk_table_set_col_spacings(GTK_TABLE(tableSearch), 2);
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), gtk_label_new ("Name"), 0, 1, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), entryAtomName, 1, 3, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), gtk_label_new ("Handle"), 3, 4, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), entryAtomUUID, 4, 5, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), gtk_label_new ("Auto expand levels"), 5, 6, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), spinbuttonAtomLevels, 6, 7, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), buttonSearch, 7, 8, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), gtk_label_new ("Type"), 0, 1, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), comboboxAtomType, 1, 3, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), checkboxAtomIncludeSubtypes, 3, 4, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), gtk_label_new ("Sort by"), 5, 6, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), comboboxSort, 6, 7, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableSearch), buttonNextPage, 7, 8, 1, 2 );

    comboboxNodeType = gtk_combo_box_text_new();

    checkboxNodeIncludeSubtypes = gtk_check_button_new_with_label("Include Subtypes");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(checkboxNodeIncludeSubtypes), TRUE);

    entryNodeMinimumSTI = gtk_entry_new ();

    entryNodeMinimumLTI = gtk_entry_new ();

    entryNodeMinimumStrength = gtk_entry_new ();

    entryNodeMinimumConfidence = gtk_entry_new ();

    GtkWidget *tableNodeFilter = gtk_table_new(2, 8, TRUE);
    gtk_table_set_row_spacings(GTK_TABLE(tableNodeFilter), 2);
    gtk_table_set_col_spacings(GTK_TABLE(tableNodeFilter), 2);
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), gtk_label_new ("Type"), 0, 1, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), comboboxNodeType, 1, 3, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), checkboxNodeIncludeSubtypes, 1, 3, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), gtk_label_new ("Minimum STI"), 3, 4, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), entryNodeMinimumSTI, 4, 5, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), gtk_label_new ("Minimum Strength"), 5, 6, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), entryNodeMinimumStrength, 6, 7, 0, 1 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), gtk_label_new ("Minimum LTI"), 3, 4, 1, 2 );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), entryNodeMinimumLTI, 4, 5, 1, 2  );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), gtk_label_new ("Minimum Confidence"), 5, 6, 1, 2  );
    gtk_table_attach_defaults(GTK_TABLE(tableNodeFilter), entryNodeMinimumConfidence, 6, 7, 1, 2  );

    comboboxLinkType = gtk_combo_box_text_new();

	checkboxLinkIncludeSubtypes = gtk_check_button_new_with_label("Include Subtypes");
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(checkboxLinkIncludeSubtypes), TRUE);

	entryLinkMinimumSTI = gtk_entry_new ();

	entryLinkMinimumLTI = gtk_entry_new ();

	entryLinkMinimumStrength = gtk_entry_new ();

	entryLinkMinimumConfidence = gtk_entry_new ();

	GtkWidget *tableLinkFilter = gtk_table_new(2, 8, TRUE);
	gtk_table_set_row_spacings(GTK_TABLE(tableLinkFilter), 2);
	gtk_table_set_col_spacings(GTK_TABLE(tableLinkFilter), 2);
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), gtk_label_new ("Type"), 0, 1, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), comboboxLinkType, 1, 3, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), checkboxLinkIncludeSubtypes, 1, 3, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), gtk_label_new ("Minimum STI"), 3, 4, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), entryLinkMinimumSTI, 4, 5, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), gtk_label_new ("Minimum Strength"), 5, 6, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), entryLinkMinimumStrength, 6, 7, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), gtk_label_new ("Minimum LTI"), 3, 4, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), entryLinkMinimumLTI, 4, 5, 1, 2  );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), gtk_label_new ("Minimum Confidence"), 5, 6, 1, 2  );
	gtk_table_attach_defaults(GTK_TABLE(tableLinkFilter), entryLinkMinimumConfidence, 6, 7, 1, 2  );

	entrySelectedName = gtk_entry_new ();
	gtk_editable_set_editable(GTK_EDITABLE(entrySelectedName), FALSE);

	entrySelectedType = gtk_entry_new ();
	gtk_editable_set_editable(GTK_EDITABLE(entrySelectedType), FALSE);

	entrySelectedUUID = gtk_entry_new ();
	gtk_editable_set_editable(GTK_EDITABLE(entrySelectedUUID), FALSE);

	entrySelectedSTI = gtk_entry_new ();
	//gtk_editable_set_editable(GTK_EDITABLE(entrySelectedSTI), FALSE);

	entrySelectedLTI = gtk_entry_new ();
	//gtk_editable_set_editable(GTK_EDITABLE(entrySelectedLTI), FALSE);

	entrySelectedStrength = gtk_entry_new ();
	gtk_editable_set_editable(GTK_EDITABLE(entrySelectedStrength), FALSE);

	GtkWidget *buttonUpdate = gtk_button_new_with_label ("Update");
    g_signal_connect (G_OBJECT (buttonUpdate), "clicked", G_CALLBACK (buttonUpdate_clicked), (gpointer) window);

	entrySelectedConfidence = gtk_entry_new ();
	gtk_editable_set_editable(GTK_EDITABLE(entrySelectedConfidence), FALSE);

	entrySelectedTruthValue = gtk_entry_new ();
	//gtk_editable_set_editable(GTK_EDITABLE(entrySelectedTruthValue), FALSE);

	GtkWidget *tableSelectedAtom = gtk_table_new(2, 10, TRUE);
	gtk_table_set_row_spacings(GTK_TABLE(tableSelectedAtom), 2);
	gtk_table_set_col_spacings(GTK_TABLE(tableSelectedAtom), 2);
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("Name"), 0, 1, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedName, 1, 3, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("Type"), 0, 1, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedType, 1, 3, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("Handle"), 3, 4, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedUUID, 4, 5, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("STI"), 5, 6, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedSTI, 6, 7, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("Strength"), 7, 8, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedStrength, 8, 9, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), buttonUpdate, 9, 10, 0, 1 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("Truth value"), 3, 4, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedTruthValue, 4, 5, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("LTI"), 5, 6, 1, 2 );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedLTI, 6, 7, 1, 2  );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), gtk_label_new ("Confidence"), 7, 8, 1, 2  );
	gtk_table_attach_defaults(GTK_TABLE(tableSelectedAtom), entrySelectedConfidence, 8, 9, 1, 2  );

	notebook = gtk_notebook_new();
    gtk_notebook_append_page (GTK_NOTEBOOK(notebook), tableSearch,gtk_label_new ("Search"));
    gtk_notebook_append_page (GTK_NOTEBOOK(notebook), tableNodeFilter,gtk_label_new ("Node Filter"));
    gtk_notebook_append_page (GTK_NOTEBOOK(notebook), tableLinkFilter,gtk_label_new ("Link Filter"));
    gtk_notebook_append_page (GTK_NOTEBOOK(notebook), tableSelectedAtom,gtk_label_new ("Selected Atom"));
    gtk_box_pack_start (GTK_BOX (vbox), notebook, FALSE, FALSE, 0);

    drawingArea=gtk_drawing_area_new ();
    gtk_widget_set_events (drawingArea, GDK_EXPOSURE_MASK | GDK_LEAVE_NOTIFY_MASK | GDK_POINTER_MOTION_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK);
    g_signal_connect (G_OBJECT (drawingArea), "draw", G_CALLBACK (drawingArea_draw), NULL);
    g_signal_connect (G_OBJECT (drawingArea), "motion_notify_event", G_CALLBACK ( drawingArea_motion_notify), NULL);
    g_signal_connect (G_OBJECT (drawingArea), "button_press_event", G_CALLBACK ( drawingArea_button_press), NULL);
    g_signal_connect (G_OBJECT (drawingArea), "button_release_event", G_CALLBACK ( drawingArea_button_press), NULL);
    gtk_box_pack_start (GTK_BOX (vbox), drawingArea, TRUE, TRUE, 0);

    g_timeout_add(50, (GSourceFunc) timer_tick, (gpointer) window);
}

void FillComboboxes()
{
	atomTypes.LoadAtomTypeScript();
	for (int i = 0; i < atomTypes.atomTypeNames.size(); i++)
	{
		gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxAtomType), "", atomTypes.atomTypeNames[i].c_str());
	}
	gtk_combo_box_set_active(GTK_COMBO_BOX(comboboxAtomType), 1);
	for (int i = 0; i < atomTypes.nodeTypeNames.size(); i++)
	{
		gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxNodeType), "", atomTypes.nodeTypeNames[i].c_str());
	}
	gtk_combo_box_set_active(GTK_COMBO_BOX(comboboxNodeType), 0);
	for (int i = 0; i < atomTypes.linkTypeNames.size(); i++)
	{
		gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxLinkType), "", atomTypes.linkTypeNames[i].c_str());
	}
	gtk_combo_box_set_active(GTK_COMBO_BOX(comboboxLinkType), 0);

	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "(unsorted)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "STI (high to low)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "LTI (high to low)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "Strength (high to low)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "Confidence (high to low)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "STI (low to high)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "LTI (low to high)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "Strength (low to high)");
	gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(comboboxSort), "", "Confidence (low to high)");
	gtk_combo_box_set_active(GTK_COMBO_BOX(comboboxSort), 0);
}

int main (int argc, char *argv[])
{
	try
	{
		g_log_set_handler ("Gtk", G_LOG_LEVEL_WARNING, (GLogFunc) gtk_false, NULL);
		gtk_init (&argc,&argv);
		g_log_set_handler ("Gtk", G_LOG_LEVEL_WARNING, g_log_default_handler, NULL);

		for(int i=1;i<argc;i++)
		{
			bool error=true;
			if(argv[i][0]=='-')
			{
				string str;
				switch(argv[i][1])
				{
				case 'q': //quiet
					verbose=false;
					error=false;
					break;
				case 's': //CogServer
					str=argv[i];
					str.erase(0,2);
					atomSpaceInterface.server=str;
					error=false;
					break;
				case 'p': //path to atom_types.script
					str=argv[i];
					str.erase(0,2);
					atomTypes.atomTypesScriptPath=str;
					error=false;
					break;
				}
			}
			if(error)
			{
				cout << "Invalid option. Allowed options are:" << endl;
				cout << "-s (server) Name of the CogServer. Default is localhost. e.g. -sMyServer" << endl;
				cout << "-p (path) Path to the atom_types.script file. Default is application directory. e.g. -p/usr/include/atom_types.script" << endl;
				cout << "-q (quiet) Turn off help messages" << endl;
			}
		}

		BuildWindow();
		gtk_widget_show_all (window);
		gtk_widget_hide(buttonNextPage);

		ShowInfo("Getting started:\n"
				"-Enter name, type or handle and press search button\n"
				"-Hover over an atom to see its attributes\n"
				"-Left click to show or hide connected atoms\n"
				"-Right click to recenter the graph\n"
				"-Click and move to drag an atom to a new position\n"
				"-The node and link filters select which of the connected atoms are shown. New filter settings are applied the next time you click, right click or search.\n"
				"-Auto expand level determines how many levels of connected atoms are shown. If level > 1 it may take a while.\n"
				"\n"
				"Known bugs:\n"
				"-Update button doesn't work yet\n"
				"\n"
				"Command line options:\n"
				"-s (server) Name of the CogServer. Default is localhost. e.g. -sMyServer\n"
				"-p (path) Path to the atom_types.script file. Default is application directory. e.g. -p/usr/include/atom_types.script\n"
				"-q (quiet) Turn off help messages\n"
				"\n"
				"Written by Erwin Joosten (eni247@gmail.com)");

		FillComboboxes();

		/* Enter the main loop */
		gtk_main ();
	}
	catch(exception& e)
	{
		ShowError(e.what());
	}
	catch(...)
	{
		ShowUnknownError();
	}

	return 0;
}

