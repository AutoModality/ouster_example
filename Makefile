SHELL := /bin/bash

default:
	source /opt/ros/kinetic/setup.bash && cd catkin_ws && catkin clean -f -i -y && catkin build -j3  && catkin build  --catkin-make-args run_tests -- && \
	catkin  test -j3 --no-status ouster_ros
	rm -f catkin_ws/install/lib/pkgconfig/catkin_tools_prebuild.pc
	@echo "Built!!!!"

install:
	rm -rf catkin_ws/install/share/catkin_tools_prebuild
	install -d $(DESTDIR)/opt/ros/kinetic/
	cp -f -p -r catkin_ws/install/lib $(DESTDIR)/opt/ros/kinetic
	cp -f -p -r catkin_ws/install/share $(DESTDIR)/opt/ros/kinetic
	cp -f -p -r catkin_ws/install/include $(DESTDIR)/opt/ros/kinetic
	rm -f $(DESTDIR)/opt/ros/kinetic/lib/pkgconfig/catkin_tools_prebuild.pc


clean:
	@echo "Cleaning"
	rm -rf catkin_ws
