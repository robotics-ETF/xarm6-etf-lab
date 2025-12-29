.PHONY: clean build dependencies source-dirs sim

dependencies:
	rosdep update
	sudo apt update
	rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

clean:
	rm -r ./build/ ./install/ ./log/

build:
	colcon build --symlink-install --cmake-args " -DCMAKE_BUILD_TYPE=RelWithDebInfo"

source-dirs:
	/bin/bash 
	/root/etf-xarm-lab/source-dirs.bash

sim_static:
	ros2 launch sim_bringup static_xarm6_etflab.launch.py

sim_dynamic:
	ros2 launch sim_bringup dynamic_xarm6_etflab.launch.py
	
real:
	ros2 launch real_bringup real_xarm6_etflab.launch.py
	
cameras:
	ros2 launch real_bringup realsense_etflab.launch.py

packages:
	pip3 install git+https://github.com/dirk-thomas/vcstool.git
	sudo apt-get -y install python3-vcstool python3-rosdep python3-colcon-common-extensions 
	sudo apt-get -y install ros-humble-gazebo-ros-pkgs ros-humble-rviz2
	sudo apt-get -y install libnanoflann-dev libgoogle-glog-dev libkdl-parser-dev libeigen3-dev octomap-tools

submodules:
	$(foreach folder, src, \
		vcs import --force -w 1 $(folder) < $(folder)/repositories.yaml && \
		vcs pull -w 1 $(folder) < $(folder)/repositories.yaml && \
		(cd $(folder) && git submodule update --init --recursive) \
	;)

make full_build_container:
	make packages
	make submodules
	cd src/etf_modules/RPMPLv2 && git checkout main && cd ../../..
	cd src/etf_modules/RPMPLv2/external/ruckig && git checkout v0.15.3 && cd ../../../../..
	cd src/external_modules/gazebo_ros2_control && git checkout humble && cd ../../..
	cd src/external_modules/xarm_ros2 && git checkout humble && cd ../../..
	cd src/external_modules/xarm_ros2/xarm_sdk/cxx && git checkout master && cd ../../../../..
	make dependencies
	make build
	source source-dirs
	
