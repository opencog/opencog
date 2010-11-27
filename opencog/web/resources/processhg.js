/* OpenCog graph visualiser
 Copyright 2010 Joel Pitt
*/
/* For processing instance */
var p = {};
 
/* For graphview methods/details */
var g = {};

g.nodes = new Object;
g.links = new Object;

g.max_nodes = 100;
g.max_links = 500;
g.current_atom;

g.kinetic_energy = 0.1; /* just some value above threshold to begin */
g.kinetic_threshold = 0.02;

g.timestep = 1.0;
g.damping = 0.95;
g.max_depth = 2;

g.ready = true;
g.mass_multiplier = 10;

g.selected=-1;

g.hover_node=-1;

// function so that processing println logs to firefox js console
g.lnPrinted = function(){ console.log( ln ); }

// All atoms are added using this method
g.add_atom = function(data,textStatus,XMLHttpRequest,depth,source,handle) {
    if (depth > g.max_depth) return;
    // add links as Nodes, then just iterate through ALL outgoing sets of
    // Node as each element in the outgoing sets corresponds to exactly one
    if (handle == undefined) {
        handle = data.handle;
        g.nodes[data.handle] = g.create_node(data);
        //console.log( "Adding new atom " + handle + " d=" + depth + " s=" + source);
    } else {
        //console.log( "Traversing atom " + handle + " d=" + depth + " s=" + source);
    }
    var n = g.nodes[handle];
    //console.log( "atom with handle " + handle + " is " + n);
    // add nodes in outgoing - this means the atom is a link
    for (var i=0; i<n.outgoing.length; i++)
    {
        node_uuid = n.outgoing[i];
        /*if (node_uuid == undefined) {
            console.log("node uuid is undefined");
        }*/
        if (node_uuid != source) {
            if (!(node_uuid in g.nodes)) {
                $.ajax({url:"/rest/0.2/atom/"+n.outgoing[i],
                    dataType:"json",success:function (d,textStatus,XMLHttpRequest) {
                        g.add_atom(d,textStatus,XMLHttpRequest,depth+1,handle);
                        g.kinetic_energy = g.kinetic_threshold+1;
                    } });
            } else {
                g.add_atom(undefined,undefined,undefined,depth+1,handle,node_uuid);
            }
        }
    }
    // nodes or links can have incoming links
    for (var i=0; i<n.incoming.length; i++)
    {
        link_uuid = n.incoming[i];
        /*if (link_uuid == undefined) {
            console.log("link uuid is undefined");
        }*/
        if (link_uuid != source) {
            //console.log("Following " + handle + "'s incoming link " + link_uuid);
            if (!(link_uuid in g.nodes)) {
                $.ajax({url:"/rest/0.2/atom/"+n.incoming[i],
                    dataType:"json",success:function (d,textStatus,XMLHttpRequest) {
                        g.add_atom(d,textStatus,XMLHttpRequest,depth+1,handle)
                    } });
            } else {
                g.add_atom(undefined,undefined,undefined,depth+1,handle,link_uuid);
                //console.log("Returned from " + handle + "'s incoming link " + link_uuid);
            }
        } /*else {
            console.log("Ignoring " + handle + "'s incoming link from source " + link_uuid);
        }*/
    }
    //if (!(data.handle in g.nodes)) { g.nodes[data.handle] = new Node(data); }
}

g.add_root = function(data,textStatus,XMLHttpRequest) {
    console.log( "Loading focus " + data.handle );
    // Handle adding root node and retrieving incoming/outgoing links
    g.add_atom(data,textStatus,XMLHttpRequest,0,undefined,undefined);
    g.nodes[data.handle].x = p.width/2;
    g.nodes[data.handle].y = p.height/2;
    //g.nodes[data.handle].pinned = true;
    g.nodes[data.handle].mass = 3;
    g.current_atom=data.handle;
    g.kinetic_energy = g.kinetic_threshold+1;
    g.setAtomInfo(g.current_atom);
}

g.random_graph = function(N, connectedness) {
  // Used to generate a random graph for testing, but
  // not designed to work with new hypergraph data
  if (N > max_links) N = max_links;
  for (var i=0; i < N ; i++) {
    var data = {"handle": i,
            "outgoing": [],
            "incoming": [],
            "name": "test",
            "truthvalue": "", "sti":0, "lti":0, "type":"Node"
    };
    g.nodes[i] = new Node(data);
    //g.nodes[i].mass = random(1,3);
  }
  //cur_nodes = N;
  var n_links = N*connectedness;
  for (var i=0; i < n_links && i < max_links; i++)
  {
      var n1,n2;
      n1 = g.nodes[parseInt(p.random(N))];
      n2 = n1;
      while (n2 == n1) { n2 = g.nodes[parseInt(p.random(N))]; }
      var data = {"handle": N+i,
              "outgoing": [n1.handle,n2.handle],
              "incoming": [],
              "name": "test",
            "truthvalue": "", "sti":0, "lti":0, "type":"Link"
          };
      g.nodes[N+i] = new Node(data);
      n1.incoming.push(N+i);
      n2.incoming.push(N+i);
  }
  //cur_links = n_links;
  g.current_atom=0;
  g.kinetic_energy = g.kinetic_threshold+1;
}

g.draw = function() {
  p.background(55,55,55);
  if (!g.ready) return;
  if (g.kinetic_energy > g.kinetic_threshold || g.selected >= 0 ) {
      g.update();
      //println(g.kinetic_energy);
  }
  for (var i in g.nodes)
  {
      l = g.nodes[i];
      if (l.outgoing.length > 0) {
          for (var j=0; j < l.outgoing.length; j++) {
              if (g.current_atom == i) { p.stroke(0x1F,0x7C,0xFF); }
              n = l.outgoing[j];
              if (g.nodes[n] == undefined) continue; // sometimes happens if not finished loading
              p.line(l.x,l.y, g.nodes[n].x,g.nodes[n].y);
              var dx = g.nodes[n].x-l.x;
              var dy = g.nodes[n].y-l.y;
              if (j > 0) {
                  // Draw arrows towards non-source g.nodes
                  // TODO: these should only be drawn for directed links
                  p.stroke(0);
                  p.fill(255);
                  p.pushMatrix();
                  p.translate(g.nodes[n].x,g.nodes[n].y);
                  angle=p.atan2(dy,dx);
                  p.rotate(-Math.PI/2.0 + angle);
                  radius = g.nodes[n].mass*g.mass_multiplier;
                  p.translate(0,-radius/2.0);
                  p.triangle(0,0,-5,-5,5,-5);
                  p.popMatrix();
              }
              p.stroke(0);
          }
      }
  }

  for (var i in g.nodes)
  {
      n = g.nodes[i];
      if (g.hover_node == i) {
          p.fill(255,0,0);
      } else if (g.current_atom == i) {
          p.fill(0x1F,0x7C,0xFF);
      }else if (n.outgoing.length > 0) {
          p.fill(128);
      } else if (g.selected == i) {
          p.fill(0x2F,0x8C,0xFF);
      } else if (n.pinned) {
          p.fill(255,0,0);
      } else {
          p.fill(255);
      }
	  p.ellipse(n.x, n.y, n.mass*g.mass_multiplier,n.mass*g.mass_multiplier);
  }
  p.fill(255);
}

g.mouseMoved = function() {
    // Example method for showing details of the atom the cursor is over
    var near = g.findNearest(p.mouseX, p.mouseY);
    if (near.distance < g.nodes[near.index].mass*g.mass_multiplier) {
        // update info using jquery
        g.hover_node = near.index;
        //setAtomInfo(near.index);
    } else {
        g.hover_node = -1;
    }
}

g.mousePressed = function() {
    var near = g.findNearest(p.mouseX,p.mouseY);
    //println("nearest" + near.toSource());
    var magnet = 1.5;
    if (near.distance < g.nodes[near.index].mass*g.mass_multiplier*magnet) {
        g.setAtomInfo(near.index);
        g.selected = near.index;
        g.nodes[g.selected].x = p.mouseX;
        g.nodes[g.selected].y = p.mouseY;
    }
    g.drag_time = 0;
}

g.mouseReleased = function() {
    if (g.selected != -1 && g.drag_time < 5) {
        //window.location = '/opencog/atom/' + g.selected;
        //return;
	    //nodes[g.selected].pinned=!nodes[g.selected].pinned;
        g.nodes[g.current_atom].mass=1.0;
        g.current_atom=g.selected;
        g.nodes[g.selected].mass=3.0;
        //nodes[g.selected].x = width/2;
        //nodes[g.selected].y = height/2;
        console.log("Centering on " + g.selected);
        g.add_atom(undefined, undefined, undefined, 0, undefined, g.selected);

    }
    g.selected = -1;
    g.kinetic_energy = g.kinetic_threshold+1;
}

g.mouseDragged = function() {
    /* if mouse pressed then fix p.mouseX */
    if (g.selected >= 0) {
        g.nodes[g.selected].x = p.mouseX;
        g.nodes[g.selected].y = p.mouseY;
    }
    g.drag_time += 1;
    g.kinetic_energy = g.kinetic_threshold+1;
}

g.repulsion = function(n1, n2) {
    var repulse = 0.25 * (n1.mass + n2.mass);
    var dx = (n1.x - n2.x);
    var dy = (n1.y - n2.y);
    var distance = p.sqrt(dx*dx + dy*dy);
    var angle = p.atan2(dy,dx);
    diff = repulse * (1.0 / distance);
    dx = diff * p.cos(angle);
    dy = diff * p.sin(angle);
    var r = [dx, dy];
    return r
}

g.attraction = function(n, l) {
    attract = 1.5;
    n1 = l.outgoing[0]; n2 = l.outgoing[1];
    dx = (n1.x - n2.x) / p.width;
    dy = (n1.y - n2.y) / p.height;
    distance = p.sqrt(dx*dx + dy*dy);
    inverse=-1;
    if (n2 == n) { inverse=1; }
    var a = [inverse * attract * (dx*dx),inverse * attract * (dy*dy)];
    /* squaring removes the sign, so restore it */
    if (dx < 0.0) a[0] = -a[0];
    if (dy < 0.0) a[1] = -a[1];
    return a;
}

g.spring = function(n, l) {
    var attract = .05;
    var n1 = n; var n2 = l;
    var ideal_dist = 20.0 / n1.mass + 20.0 / n2.mass;
    var dx = (n1.x - n2.x);
    var dy = (n1.y - n2.y);
    var distance = p.sqrt(dx*dx + dy*dy);
    var diff = distance - ideal_dist;
    var angle = p.atan2(dy,dx);
    dx = diff * p.cos(angle);
    dy = diff * p.sin(angle);
    if (n2 != n) { dx = -dx; dy = -dy; }
    var a = [attract * dx,attract * dy];
    //println ("spring: " + a[0] + " " + a[1]);
    /* squaring removes the sign, so restore it */
    //if (dx < 0.0) a[0] = -a[0];
    //if (dy < 0.0) a[1] = -a[1];
    return a;
}

g.update = function() {
    /* do force directed layout */
    g.kinetic_energy = 0.0;
    for (var i in g.nodes) {
        n = g.nodes[i];
        if (g.selected == i || n.pinned) { continue; }
        if (g.current_atom == i) {
            mx = p.width/2;
            my = p.height/2;
            if (n.x != mx || n.y != my){
                dx = (n.x - mx);
                dy = (n.y - my);
                if (dx*dx <= 25) n.x = mx;
                else n.x = n.x - (dx/2);
                if (dy*dy <= 25) n.y = my;
                else n.y = n.y - (dy/2);
            }   
            continue;
        }
        if (n.outgoing.length == 2) {
            // if there are two outgoing links and both end points exist
            // then just place node midway between end points
            x = g.nodes[n.outgoing[0]];
            y = g.nodes[n.outgoing[1]];
            if (x != undefined && y != undefined) {
                n.x = (x.x + y.x) / 2;
                n.y = (x.y + y.y) / 2;
                continue;
            }
        }
        net_force = [ 0.0, 0.0 ];
        for (var j in g.nodes) {
            // Electro static repulsion
            if(j == i) continue;
            r = g.repulsion(n,g.nodes[j]);
            net_force[0] += r[0];
            net_force[1] += r[1];
        }

        // Spring ideal length
        for (j in n.outgoing) {
            l = n.outgoing[j];
            //a = attraction(n, l);
            if (g.nodes[l] == undefined) continue;
            a = g.spring(n, g.nodes[l]);
            net_force[0] += a[0];
            net_force[1] += a[1];
        }
        // Spring ideal length
        for (j in n.incoming) {
            l = n.incoming[j];
            //a = attraction(n, l);
            if (g.nodes[l] == undefined) continue;
            a = g.spring(n,g.nodes[l]);
            net_force[0] += a[0];
            net_force[1] += a[1];
        }
        /* limit maximum force applied */
        //net_force[0] = constrain(net_force[0],-3,3);
        //net_force[1] = constrain(net_force[1],-3,3);
        /* prevent leaving view */
        var border = 10
        if (n.x > p.width-border) {
            net_force[0] = -0.1;
        }
        if (n.x < 0 + border) {
            net_force[0] = 0.1;
        }
        if (n.y > p.height - border) {
            net_force[1] = -0.1;
        }
        if (n.y < 0 + border) {
            net_force[1] = 0.1;
        }
        /* change velocity */
        n.vx = (n.vx + (g.timestep * net_force[0])) * g.damping;
        n.vy = (n.vy + (g.timestep * net_force[1])) * g.damping;
        /* limit maximum speed */
        //n.vx = constrain(n.vx,-3,3);
        //n.vy = constrain(n.vy,-3,3);
        /*float v = p.sqrt(n.vx*n.vx + n.vy*n.vy);
        if (v > 5) {
            println ("large velocity: " + i + " v=" + v);
        }*/
        /* update position */
        n.x += (g.timestep * n.vx);
        n.y += (g.timestep * n.vy);
        g.kinetic_energy += n.energy();
    }
    //damping -= 0.01;

}

g.create_node = function(data) {
    var n = new Object();
    n.handle = data.handle;
    n.x = p.random(p.width);
    n.y = p.random(p.height);
    n.mass = 1.0;
    n.vx = 0.0, n.vy = 0.0;
    n.pinned = false;
    n.type = data.type;
    n.name = data.name;
    n.sti = data.sti;
    n.lti = data.lti;
    n.incoming = data.incoming;
    n.outgoing = data.outgoing;
    n.tv = data.truthvalue;

    n.energy = function() {
        /* v^2 * mass */
        return (this.vx*this.vx + this.vy*this.vy)*this.mass
    }

    n.toString = function() {
        var desc = "Node";
        if (this.outgoing.length > 0) desc = "Link";
        var s = desc + " " + this.handle + ": out=[" + this.outgoing + "] in=[" + this.incoming + "]";
        return s;
    }
    return n;
}

g.create_link = function(n1, n2) {
    handle;
    var outgoing = new Array();
    outgoing[0] = n1;
    outgoing[1] = n2;
    var owner; // handle of the "node" that owns this link
}

g.findNearest = function (mousex, mousey) {
    var result = new Object;
    var nearest = -1;
    var nearest_dist = p.width*p.height;
    /* find nearest node */
    for (var i in g.nodes)
    {
        dx = g.nodes[i].x - mousex;
        dy = g.nodes[i].y - mousey;
        distance = p.sqrt(dx*dx + dy*dy);
        if (distance < nearest_dist) {
            nearest_dist = distance;
            nearest = i;
        }
    }
    result.index = parseInt(nearest);
    result.distance = nearest_dist;
    return result;
}

g.setAtomInfo = function (h) {
    $('#atomuuid').html(g.nodes[h].handle);
    $('#atomtype').html(g.nodes[h].type);
    $('#atomname').html(g.nodes[h].name);
    $('#atomsti').html(g.nodes[h].sti);
    $('#atomlti').html(g.nodes[h].lti);
    $('#atomtv').html(tvToString(g.nodes[h].tv));
}

var tvToString = function (tv) {
    var result = "";
    if (tv.hasOwnProperty("simple")) {
        result += "SimpleTV [strength: " + tv.simple.str;
        result += " confidence: " + tv.simple.conf + "]";
    }
    return result;
}

$(document).ready(function(){
 p = new Processing( document.getElementById( "graphview" ), function( p, c ) {  
     p.setup = function() {
          p.size(600, 300);
          var fontA = p.loadFont("Arial");
          p.textFont(fontA, 16);
          $.ajax({url:"/rest/0.2/atom/"+handle,
            dataType:"json",success:g.add_root});
          //random_graph(20,0.9);
      }
      /*function( ) {  
          p.size( 1250, 250 );  
          p.background( 55 );  
          p.frameRate( 10 );  
          p.cursor( c.HAND );  
      }; */   
       
      p.draw = function( ) {  
         p.background( 155 );  
      };
   
      Processing.addInstance( p );  
    });  
    
    p.draw = g.draw;
    p.mouseMoved = g.mouseMoved;
    p.mousePressed = g.mousePressed;
    p.mouseDragged =  g.mouseDragged;
    p.mouseReleased = g.mouseReleased;
});

