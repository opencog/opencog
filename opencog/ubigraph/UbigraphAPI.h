/**
 * This is a C language API.
 */
#ifndef UBIGRAPHAPI_H
#define UBIGRAPHAPI_H

#include <stdint.h>

typedef int32_t result_t;
typedef int32_t vertex_id_t;
typedef int32_t edge_id_t;
typedef int32_t style_id_t;

/* Basic API methods */
vertex_id_t ubigraph_new_vertex();
edge_id_t   ubigraph_new_edge(vertex_id_t x, vertex_id_t y);
result_t    ubigraph_remove_vertex(vertex_id_t x);
result_t    ubigraph_remove_edge(edge_id_t e);

/* Vertex/edge creation when user wants to use their own id's */
result_t    ubigraph_new_vertex_w_id(vertex_id_t x);
result_t    ubigraph_new_edge_w_id(edge_id_t e, vertex_id_t x, vertex_id_t y);

/* Delete all vertices and edges */
void        ubigraph_clear();

/* Set a vertex attribute */
result_t    ubigraph_set_vertex_attribute(vertex_id_t x,
              const char* attribute, const char* value);

/* Vertex styles */
result_t    ubigraph_change_vertex_style(vertex_id_t x, style_id_t s);
style_id_t  ubigraph_new_vertex_style(style_id_t parent_style);
result_t    ubigraph_new_vertex_style_w_id(style_id_t s, 
              style_id_t parent_style);
result_t    ubigraph_set_vertex_style_attribute(style_id_t s,
              const char* attribute, const char* value);

/* Set an edge attribute */
result_t    ubigraph_set_edge_attribute(edge_id_t x,
              const char* attribute, const char* value);

/* Edge styles */
result_t    ubigraph_change_edge_style(edge_id_t x, style_id_t s);
style_id_t  ubigraph_new_edge_style(style_id_t parent_style);
result_t    ubigraph_new_edge_style_w_id(style_id_t s,
              style_id_t parent_style);
result_t    ubigraph_set_edge_style_attribute(style_id_t s,
              const char* attribute, const char* value);

#define UBIGRAPH_SUCCESS   0
#define UBIGRAPH_FAIL      -1

#endif

