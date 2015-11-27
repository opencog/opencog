# -*- mode: ruby -*-
# # vi: set ft=ruby :

# QuickStart
# 1. vagrant up
# 2. vagrant ssh
#
# Windows note: before running the above commands, you must make sure that 'ssh'
# is in your system path. After downloading 'git', you should go to your control
# panel, and edit your Environment Variables and append the folder containing
# ssh to the system path. For example: C:\Program Files (x86)\Git\bin
#
# Optional
# 1. Change Ubuntu archive mirror to a local mirror
# 2. Change vb.customize memory and cpus values
# 3. On Linux hosts, use provider vagrant-lxc
# More
# http://wiki.opencog.org/w/Setup_OpenCog_development_environment_VM_using_Vagrant
# http://wiki.opencog.org/w/Building_OpenCog

Vagrant.configure("2") do |config|

  # Enable networking; see http://stackoverflow.com/a/18457420/1695962
  config.vm.provider "virtualbox" do |v|
    v.customize ["modifyvm", :id, "--natdnshostresolver1", "on"]
    v.customize ["modifyvm", :id, "--natdnsproxy1", "on"]
  end

  # 64-bit machine caused a fatal issue: http://stackoverflow.com/a/22575302/1695962
  config.vm.box = "ubuntu/trusty32"
  config.vm.hostname = "cogbox"
  config.vm.provision "shell", inline: "ln -v -s /vagrant /home/vagrant/opencog"
  config.vm.provision "shell", inline: "wget https://raw.githubusercontent.com/opencog/ocpkg/master/ocpkg -O /tmp/octool  && chmod +rx /tmp/octool"
  config.vm.provision "shell", inline: "/tmp/octool -rdpscalv"

  # Port forwarding for REST API
  config.vm.network "forwarded_port", guest: 5000, host: 5000

  # Set --host to 192.168.50.2 when running opencog-server.sh in RelEx,
  # to pass RelEx's OpenCog scheme output to cogbox.
  config.vm.network "private_network", ip: "192.168.50.2"

  # Configure port for telnet access to shell
  config.vm.network "forwarded_port", guest: 17001, host: 17001

  # Enable X-Forwarding
  config.ssh.forward_x11 = true

  config.vm.provider :virtualbox do |vb|
      vb.name = "cogbox"
      vb.customize [
                   "modifyvm", :id,
                   "--memory", "2048",
                   "--name", "opencog-dev-vm",
                   "--cpus", "1"
                   ]
  end
end
