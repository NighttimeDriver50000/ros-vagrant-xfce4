# vim: set filetype=ruby :
Vagrant.configure("2") do |config|
    config.vm.define "xiaofeng-ekf" do |xe|
        xe.vm.provider "virtualbox" do |v, override|
            v.cpus = 3
            v.gui = true
            v.linked_clone = true
            v.memory = 8192
            v.name = 'ros-vagrant-xiaofeng-ekf'
            v.customize ["modifyvm", :id, "--ostype", "Ubuntu_64", "--vram", "256",
                        "--acpi", "on", "--ioapic", "on", "--hwvirtex", "on"]
            override.vm.box = "bento/ubuntu-16.04"
            override.vm.synced_folder "catkin_ws/", "/home/vagrant/catkin_ws"
            override.vm.provision "file", source: "av8pves.jpg", destination: "/tmp/av8pves.jpg"
            override.vm.provision "file", source: "catkin_ws/vimrc", destination: "/tmp/vimrc"
            override.vm.provision "file", source: "Default", destination: "/tmp/Default"
            #override.vm.provision "file", source: "wipe_test_rover.bash", destination: "/tmp/wipe_test_rover.bash"
            #override.vm.provision "file", source: "StartInWM.bash", destination: "/tmp/StartInWM.bash"
            #override.vm.provision "file", source: "roscore.bash", destination: "/tmp/roscore.bash"
            override.vm.provision "shell", path: "provision.sh"
        end
    end
    config.vm.define "ros-vagrant" do |rv|
        config.vm.provider "docker" do |d, override|
            d.build_dir = "."
            d.create_args = ["--restart", "unless-stopped", "--add-host", "firmware.ardupilot.org:127.0.0.1"]
            d.name = "ros-vagrant"
            d.ports = ["11311:11311"]
            override.vm.synced_folder "catkin_ws/", "/home/ros/catkin_ws"
        end
    end
    config.vm.define "rv-client" do |rc|
        config.vm.provider "docker" do |d, override|
            d.build_dir = "."
            d.create_args = ["--restart", "unless-stopped", "--add-host", "firmware.ardupilot.org:127.0.0.1", "--env", "GCS_CLIENT_MODE=1"]
            d.name = "ros-vagrant-client"
            #d.ports = ["11311:11311"]
        end
    end
end
