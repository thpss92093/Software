duckietown_package=$(catkin_ws)/src/00-infrastructure/duckietown
machines=$(duckietown_package)/machines


build:
	@echo "$(sep)Building commands"
	@echo
	@echo "Commands to build the software."
	@echo
	@echo '- `make build-machines`       :  Builds the machines file.'
	@echo '- `make build-machines-clean` :  Removes the machines file.'
	@echo '- `make build-clean`          :  Clean everything.'

$(machines): build-machines

build-machines:
	rosrun duckietown create-machines-file


build-machines-clean:
	@echo
	@echo Removing machines file.
	rm -f $(machines)

build-clean: \
	build-catkin-clean \
	build-machines-clean

build-catkin:
	catkin_make -C $(catkin_ws)

build-catkin-parallel:
	catkin_make -C $(catkin_ws) --make-args "-j4"

build-catkin-clean:
	@echo
	@echo Removing the directory $(catkin_ws)/build
	rm -rf $(catkin_ws)/build


.PHONY: check-environment

check-environment:
	# Put here procedures to check the environment is valid
	#-./what-the-duck