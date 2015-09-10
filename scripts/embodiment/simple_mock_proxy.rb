require "gserver"
require "net/telnet"
require "rexml/document"

SESSION_EXPIRATION = 12000000.0

class MessageReceiver < GServer
  def initialize(handler, port, host = GServer::DEFAULT_HOST)
    @handler = handler
    super(port, host, 10, $stderr, true)
  end
  
  def serve(socket)
    message = [socket.gets.chomp[1..-1]]
    if message.first =~ /^START_MESSAGE/
      message << socket.gets.chomp[1..-1] until message.last =~ /^NO_MORE_MESSAGES/
    end
    puts "Received message"
    @handler.process message
    socket.puts "OK"
  end
end

class ActionExtractor
  attr_reader :planId
  
  def initialize(message)
    begin
      #puts message
      doc = REXML::Document.new(message.to_s)
      petActions = doc.get_elements("pet:action-plan")
      @planId = petActions.first.attributes["id"]
      puts "--> Got plan ID: #@planId"
    rescue
      @planId = "0"
      puts "--> Setting plan ID to 0"
    end
    puts "--> Action plan ID: #@planId"
  end
end

class PerceptionSimulator
  TEMPLATE = %{<oc:embodiment-msg xmlns:pet="http://www.opencog.org/brain">
    <map-info region="ESC25">
        <blip timestamp="%s" detector="false">
            <entity id="Wynx" type="avatar"/>
            <position x="5" y="64" z="4"/>
            <rotation pitch="0" roll="0" yaw="2.5"/>
            <velocity x="0" y="0" z="0"/>
        </blip>
        <blip timestamp="%s" detector="true">
            <entity id="Stick" type="accessory"/>
            <position x="5.05" y="64.25" z="4.25"/>
            <rotation pitch="0" roll="0.5" yaw="2.5"/>
            <velocity x="0" y="0" z="0"/>
        </blip>
        <blip timestamp="%s" detector="true">
            <entity id="1" type="pet" owner-id="Wynx" owner-name="Wynx"/>
            <position x="5" y="65.1" z="3.2"/>
            <rotation pitch="0" roll="0" yaw="182.5"/>
            <velocity x="0" y="0" z="0"/>
        </blip>
    </map-info>
    <pet-signal pet-id="1" name="hunger-change" timestamp="%s"> 
	    <param name="old-level" type="float" value="%.2f"/> 
        <param name="new-level" type="float" value="%.2f"/> 
    </pet-signal>
    <pet-signal pet-id="1" name="whatever" status="done" action-plan-id="%s" timestamp="%s" />
    <instruction pet-id="1" avatar-id="10" timestamp="%s">
        <value>%s</value>
    </instruction>
</oc:embodiment-msg>}
  
  def initialize
    @hunger = 0.0
  end
  
  def next_perception(planId)
    # TODO: Timezone format is not compatible with xsd:dateTime required by ESC's XSD
    #ts = Time.now.strftime("%Y-%m-%dT%H:%M:%S-%Z")
    ts = Time.now.strftime("%Y-%m-%dT%H:%M:%S")
   
    # current actions sent by OAC for test purposes:
    #insertAtoms(ActionType::DROP().getName());
    #insertAtoms(ActionType::SNIFF().getName());
    #insertAtoms(ActionType::BARK().getName());
    #insertAtoms(ActionType::HOWL().getName());
    #insertAtoms(ActionType::BARE_TEETH().getName());
    #insertAtoms(ActionType::SIT().getName());
    #insertAtoms(ActionType::BEG().getName());
    #insertAtoms(ActionType::HEEL().getName());

    if rand() > 0.5
        reward = "Bad boy!!" 
        @hunger_old = @hunger
        @hunger = [@hunger + 0.05, 1.0].min

    else
        reward = "Good boy!!" 
        @hunger_old = @hunger
        @hunger = [@hunger - 0.05, 0.0].max
    end

#    if action =~ /rollOver/
#        reward = "Good boy!!" 
#        @hunger_old = @hunger
#        @hunger = [@hunger + 0.1, 1.0].min
#
#    elsif action =~ /eat/
#        reward = "Bad boy!!" 
#        @hunger_old = @hunger
#        @hunger = [@hunger - 0.25, 0.0].max
#
#    elsif action =~ /bark/
#        reward = "Bad boy!!" 
#        @hunger_old = @hunger
#        @hunger = [@hunger + 0.05, 1.0].min
#
#    else # walk 
#        reward = "Good boy!!" 
#        @hunger_old = @hunger
#        @hunger = [@hunger + 0.2, 1.0].min
#    end

    TEMPLATE % [ts, ts, ts, ts, @hunger_old, @hunger, planId, ts, ts, reward]
  end
end

class MockProxy
  def initialize(my_port, router_host, router_port)
    @my_port, @router_host, @router_port = my_port, router_host, router_port
    @start_time = Time.now
    @simulator = PerceptionSimulator.new
    @listener = MessageReceiver.new(self, @my_port)
    @listener.start
    greet_router
    logon
    @listener.join
  end
  
  def process(msg)
    puts msg
    if Time.now - @start_time > SESSION_EXPIRATION
      logoff
    elsif msg.join('\n') =~ /NOTIFY_NEW_MESSAGE/
      request_messages
    elsif msg[0] =~ /SPAWNER/ && msg[1] =~ /FAIL/
      raise "Spawner failed: #{msg.join('\n')}"
    else
      # TODO: Support multiple messages
      send_next_perception(ActionExtractor.new(msg[1..-2]).planId)
    end
  end

  def greet_router
    send "LOGIN PROXY 127.0.0.1 #@my_port"
  end
  
  def logon
    send "NEW_MESSAGE PROXY SPAWNER 1 1\nLOAD_PET 1"
  end
  
  def logoff
    send "NEW_MESSAGE PROXY SPAWNER 1 1\nUNLOAD_PET 1"
  end
  
  def request_messages
    send "REQUEST_UNREAD_MESSAGES PROXY 100"
  end
  
  def send_next_perception(planId)
    message = @simulator.next_perception(planId)
    lines = message.split("\n").size
    puts lines
    send "NEW_MESSAGE PROXY 1 1 #{lines}\n#{message}"
  end
  
  private
  
  def send(msg)
    router = Net::Telnet::new('Host' => @router_host,
                              'Port' => @router_port,
                              'Waittime' => 1.00,
                              'Prompt' => /.*/,
                              'Telnetmode' => false)
    router.cmd(msg) do |answer|
      raise "Router doesn't like: #{msg}" unless answer =~ /OK/
    end
    router.close
  end
end
