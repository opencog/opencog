# QuickStart
# 1. vagrant box add precise64 http://files.vagrantup.com/precise64.box
# 2. vagrant up
# 3. vagrant ssh
# Optional
# 1. Change Ubuntu archive mirror
# 2. Change memory and cpus values
# 3. Use provider vagrant-lxc on Linux
# More: http://wiki.opencog.org/w/Setup_OpenCog_development_environment_VM_using_Vagrant
# More: http://wiki.opencog.org/w/Building_OpenCog

Vagrant.configure("2") do |config|
  config.vm.box = "precise64"
  config.vm.hostname = "cogbox"
  config.vm.provision "shell", inline: "sed -i 's/us.archive.ubuntu.com/hk.archive.ubuntu.com/g' /etc/apt/sources.list"
  config.vm.provision "shell", inline: "apt-get update -y"
  config.vm.provision "shell", inline: "apt-get install python-software-properties -y"
  config.vm.provision "shell", inline: "ln -v -s /vagrant /usr/local/src/opencog"
  config.vm.provision "shell", inline: "ln -v -s /vagrant /home/vagrant/opencog"
  config.vm.provision "shell", path:   "http://raw.github.com/opencog/ocpkg/master/ocpkg"

  # Port forwarding for AtomSpace Visualizer. 
  # Set IP_ADDRESS = '0.0.0.0' in /opencog/python/web/api/restapi.py and
  # run the Visualizer on host.
  config.vm.network "forwarded_port", guest: 5000 , host: 5000

  config.vm.provider :virtualbox do |vb|
      vb.name = "cogbox"
      vb.customize [
                   "modifyvm", :id,
                   "--memory", "1536",
                   "--name", "opencog-dev-vm",
                   "--cpus", "2"
                   ]
  end
end
