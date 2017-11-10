# vim: set filetype=ruby :
Vagrant.configure("2") do |config|
    config.vm.provider "docker" do |d|
        d.build_dir = "."
        d.create_args = ["--restart", "unless-stopped", "--add-host", "firmware.ardupilot.org:127.0.0.1"]
        d.name = "ros-vagrant"
    end
    config.vm.synced_folder "catkin_ws/", "/home/ros/catkin_ws"
end
