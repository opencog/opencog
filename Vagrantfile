# -*- mode: ruby -*-
# # vi: set ft=ruby :

# QuickStart
# 1. vagrant up
# 2. vagrant ssh
# Optional
# 1. Change Ubuntu archive mirror to a local mirror
# 2. Change vb.customize memory and cpus values
# 3. On Linux hosts, use provider vagrant-lxc
# More
# http://wiki.opencog.org/w/Setup_OpenCog_development_environment_VM_using_Vagrant
# http://wiki.opencog.org/w/Building_OpenCog

Vagrant.configure("2") do |config|
  config.vm.box = "trusty64"
  config.vm.box_url = "http://files.vagrantup.com/trusty64.box"
  config.vm.box_url = "https://cloud-images.ubuntu.com/vagrant/trusty/current/trusty-server-cloudimg-amd64-vagrant-disk1.box"
  config.vm.hostname = "cogbox"
  config.vm.provision "shell", inline: "sed -i 's:/archive.ubuntu.com:/hk.archive.ubuntu.com:g' /etc/apt/sources.list"
  config.vm.provision "shell", inline: "apt-get update -y"
  config.vm.provision "shell", inline: "apt-get install software-properties-common -y"
  config.vm.provision "shell", inline: "ln -v -s /vagrant /usr/local/src/opencog"
  config.vm.provision "shell", inline: "ln -v -s /vagrant /home/vagrant/opencog"
  config.vm.provision "shell", inline: "cp -v /vagrant/scripts/ocpkg /install-dependencies-trusty"
  config.vm.provision "shell", inline: "/install-dependencies-trusty"

  # Port forwarding for AtomSpace Visualizer. 
  # Set IP_ADDRESS = '0.0.0.0' in /opencog/python/web/api/restapi.py and
  # run the Visualizer on host.
  config.vm.network "forwarded_port", guest: 5000, host: 5000

  # Set --host to 192.168.50.2 when running opencog-server.sh in RelEx,
  # to pass RelEx's OpenCog scheme output to cogbox.
  config.vm.network "private_network", ip: "192.168.50.2"

  config.vm.provider :virtualbox do |vb|
      vb.name = "cogbox"
      vb.customize [
                   "modifyvm", :id,
                   "--memory", "1536",
                   "--name", "opencog-dev-vm",
                   "--cpus", "1"
                   ]
  end
end
