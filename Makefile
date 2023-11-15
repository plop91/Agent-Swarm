SHELL := /bin/bash

all: build_run_agent

build: FORCE
	colcon build

source_install: FORCE
	source $(CURDIR)/install/setup.bash

build_run_top_level_council: build source_install
	ros2 launch agent_swarm top_level_council_launch.py

test_message: source_install
	ros2 topic pub --once test_swarm/input_thought std_msgs/String '{data: "can you use the provided documentation to create a bot that accesses the OpenAI API?"}'

start_talking: source_install
	ros2 topic pub --once agent_1_thought std_msgs/String '{data: "how are you today?"}'

build_run_agent: build source_install
	ros2 launch agent_swarm swarm_launch.py

build_run_parent: build source_install
	ros2 launch agent_swarm parent_launch.py

build_run_correctness: build source_install
	ros2 launch agent_swarm correctness_stack_launch.py

talking_agents: build source_install
	ros2 launch agent_swarm talking_test_launch.py

clean:
	rm -rf build install log

FORCE: