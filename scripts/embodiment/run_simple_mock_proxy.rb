#!/usr/bin/env ruby 

require File.join(File.dirname(__FILE__), "simple_mock_proxy")
require 'optparse'

mock_port = 16315
router_port = 16312
#router_port = 16316
router_host = 'localhost'

op = OptionParser.new do |opts|
  opts.on("--port PROXY_PORT", Integer, "Mock proxy port number") do |mp|
    mock_port = mp
  end
  
  opts.on("--router-port ROUTER_PORT", Integer, "Router port number") do |rp|
    router_port = rp
  end
  
  opts.on("--router-host [HOST]", "Router hostname, defaults to localhost") do |h|
    router_host = h
  end
  
  opts.on_tail("-h", "--help", "Show this message") do
    puts opts
    exit
  end
end

op.parse!(ARGV)

MockProxy.new(mock_port, router_host, router_port)
