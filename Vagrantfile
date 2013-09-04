# -*- mode: ruby -*-
# vi: set ft=ruby :

# Documentation: http://wiki.opencog.org/w/Setup_OpenCog_development_environment_VM_using_Vagrant

Vagrant.configure("2") do |config|
  # All Vagrant configuration is done here. For a complete reference of
  # available configuration options, please see the online documentation
  # at vagrantup.com.

  # Every Vagrant virtual environment requires a box to build off of.
  config.vm.box = "precise-base"
  # Hostname of the virtual machine:
  config.vm.hostname = "opencog-dev-vm"

  # Enable provisioning with Shell (script):
  config.vm.provision :shell, :path => "vagrant-configs/vagrant-vm-setup.sh"

  # Port forwarding
  #config.vm.network :forwarded_port, host: 4567, guest: 80

  # The url from where the 'config.vm.box' box will be fetched if it
  # doesn't already exist on the user's system.
  # config.vm.box_url = "http://domain.com/path/to/above.box"

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine. In the example below,
  # accessing "localhost:8080" will access port 80 on the guest machine.
  # config.vm.network :forwarded_port, guest: 80, host: 8080

  # Create a private network, which allows host-only access to the machine
  # using a specific IP.
  # config.vm.network :private_network, ip: "192.168.33.10"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  # config.vm.network :public_network

  # Share an additional folder to the guest VM. The first argument is
  # the path on the host to the actual folder. The second argument is
  # the path on the guest to mount the folder. And the optional third
  # argument is a set of non-required options.
  # config.vm.synced_folder "../data", "/vagrant_data"

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider :virtualbox do |vb|
  #   # Don't boot with headless mode
  #   vb.gui = true
  #
      vb.name = "opencog-dev-vm"
  #   # Use VBoxManage to customize the VM. For example to change memory:
      vb.customize [
                   "modifyvm", :id,
                   "--memory", "1550",
                   "--name", "opencog-dev-vm",
                   # When the VM isn't running, you can change the "1" below to the number of host cores you want to assign to VM:
                   "--cpus", "1"
                   ]
  end
end
