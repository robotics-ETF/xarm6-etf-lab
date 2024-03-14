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
	/bin/bash /root/etf-xarm-lab/source-dirs.bash

sim:
	# make build
	ros2 launch sim_bringup xarm6_etflab.launch.py
	
real:
	ros2 launch real_bringup real_xarm6_etflab.launch.py
	
cameras:
	ros2 launch real_bringup realsense_etflab.launch.py

submodules:
	pip3 install git+https://github.com/dirk-thomas/vcstool.git
	$(foreach folder, src, \
		vcs import --force -w 1 $(folder) < $(folder)/repositories.yaml && \
		vcs pull -w 1 $(folder) < $(folder)/repositories.yaml && \
		(cd $(folder) && git submodule update --init --recursive) \
	;)

make full_build_container:
	make submodules
	cd src/etf_modules/RPMPLv2 && git checkout main && cd ../../..
	cd src/external_modules/gazebo_ros2_control && git checkout humble && cd ../../..
	cd src/external_modules/xarm_ros2 && git checkout humble && cd ../../..
	cd src/external_modules/xarm_ros2/xarm_sdk/cxx && git checkout master && cd ../../../../..
	make dependencies
	make build
	