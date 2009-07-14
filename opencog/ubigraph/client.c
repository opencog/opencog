#include <stdlib.h>
#include <stdio.h>
#include <xmlrpc.h>
#include <xmlrpc_client.h>
#include <assert.h>

//#include "config.h"  /* information about this build environment */

#include <UbigraphAPI.h>

#define NAME "XML-RPC C Test Client"
#define VERSION "1.0"

void die_if_fault_occurred (xmlrpc_env *env)
{
    if (env->fault_occurred) {
        fprintf(stderr, "XML-RPC Fault: %s (%d)\n",
                env->fault_string, env->fault_code);
        exit(1);
    }
}

int xmlrpc_initialized = 0;
xmlrpc_env env;

const char* ubigraph_url = "http://localhost:20738/RPC2";

void init_xmlrpc(const char* url)
{
  if (url != NULL)
    ubigraph_url = url;

  /* Initialize our error-handling environment. */
  xmlrpc_env_init(&env);

  /* Start up our XML-RPC client library. */
  xmlrpc_client_init2(&env, XMLRPC_CLIENT_NO_FLAGS, NAME, VERSION, NULL, 0);
  die_if_fault_occurred(&env);

  xmlrpc_initialized = 1;
}

void close_xmlrpc()
{
  assert(xmlrpc_initialized);

  /* Clean up our error-handling environment. */
  xmlrpc_env_clean(&env);
    
  /* Shutdown our XML-RPC client library. */
  xmlrpc_client_cleanup();
}

result_t ubigraph_new_vertex_w_id(vertex_id_t x)
{
  char* const methodName = "ubigraph.new_vertex_w_id";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(i)", (xmlrpc_int32) x);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

vertex_id_t ubigraph_new_vertex()
{
  char* const methodName = "ubigraph.new_vertex";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "()");
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_new_edge_w_id(edge_id_t e, vertex_id_t x, vertex_id_t y)
{
  char* const methodName = "ubigraph.new_edge_w_id";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(iii)", (xmlrpc_int32)e, (xmlrpc_int32)x, (xmlrpc_int32)y);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

edge_id_t ubigraph_new_edge(vertex_id_t x, vertex_id_t y)
{   
  char* const methodName = "ubigraph.new_edge";
  xmlrpc_value * resultP;
  
  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(ii)", (xmlrpc_int32)x, (xmlrpc_int32)y);
  die_if_fault_occurred(&env);
  
  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_remove_vertex(vertex_id_t x)
{
  char* const methodName = "ubigraph.remove_vertex";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(i)", (xmlrpc_int32) x);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_remove_edge(edge_id_t e)
{
  char* const methodName = "ubigraph.remove_edge";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(i)", (xmlrpc_int32) e);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

void ubigraph_clear()
{ 
  char* const methodName = "ubigraph.clear";
  xmlrpc_value * resultP;
    
  if (!xmlrpc_initialized)
    init_xmlrpc(0);
  
  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "()");
  die_if_fault_occurred(&env);
  
  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);
  
  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);
}

result_t ubigraph_set_vertex_attribute(vertex_id_t x, const char* attribute, 
  const char* value)
{
  char* const methodName = "ubigraph.set_vertex_attribute";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
    "(iss)", (xmlrpc_int32)x, attribute, value);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

style_id_t ubigraph_new_vertex_style(style_id_t parent_style)
{
  char* const methodName = "ubigraph.new_vertex_style";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(i)", (xmlrpc_int32)parent_style);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_new_vertex_style_w_id(style_id_t s, style_id_t parent_style)
{
  char* const methodName = "ubigraph.new_vertex_style_w_id";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(ii)", (xmlrpc_int32)s, (xmlrpc_int32)parent_style);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_change_vertex_style(vertex_id_t x, style_id_t s)
{
  char* const methodName = "ubigraph.change_vertex_style";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(ii)", (xmlrpc_int32)x, (xmlrpc_int32)s);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_set_vertex_style_attribute(style_id_t s, 
  const char* attribute, const char* value)
{ 
  char* const methodName = "ubigraph.set_vertex_style_attribute";
  xmlrpc_value * resultP;
  
  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
    "(iss)", (xmlrpc_int32)s, attribute, value);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);
  
  /* Dispose of our result value. */ 
  xmlrpc_DECREF(resultP);
  
  return status;
}

style_id_t ubigraph_new_edge_style(style_id_t parent_style)
{
  char* const methodName = "ubigraph.new_edge_style";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(i)", (xmlrpc_int32)parent_style);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_new_edge_style_w_id(style_id_t s, style_id_t parent_style)
{
  char* const methodName = "ubigraph.new_edge_style_w_id";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(ii)", (xmlrpc_int32)s, (xmlrpc_int32)parent_style);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_change_edge_style(edge_id_t x, style_id_t s)
{
  char* const methodName = "ubigraph.change_edge_style";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
               "(ii)", (xmlrpc_int32)x, (xmlrpc_int32)s);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_set_edge_style_attribute(style_id_t s, const char* attribute,
  const char* value)
{
  char* const methodName = "ubigraph.set_edge_style_attribute";
  xmlrpc_value * resultP;

  if (!xmlrpc_initialized)
    init_xmlrpc(0);

  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
    "(iss)", (xmlrpc_int32)s, attribute, value);
  die_if_fault_occurred(&env);

  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);

  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);

  return status;
}

result_t ubigraph_set_edge_attribute(edge_id_t s, const char* attribute,
  const char* value)
{
  char* const methodName = "ubigraph.set_edge_attribute";
  xmlrpc_value * resultP;
  
  if (!xmlrpc_initialized)
    init_xmlrpc(0);
  
  /* Make the remote procedure call */
  resultP = xmlrpc_client_call(&env, ubigraph_url, methodName,
    "(iss)", (xmlrpc_int32)s, attribute, value);
  die_if_fault_occurred(&env);
               
  xmlrpc_int32 status;
  xmlrpc_parse_value(&env, resultP, "i", &status);
  die_if_fault_occurred(&env);
  
  /* Dispose of our result value. */
  xmlrpc_DECREF(resultP);
  
  return status;
}

