# symbolic_fact_generation

Transforms robot sensor data into symbolic facts for planning. Currently only generates "on" table facts (e.g. on(multimeter_1, table_1))
using the poses of the tables (given by a config file) and the results of the object pose estimation created by DOPE (Deep Object Pose Estimation).

# Requirements

The symbolic fact generation assumes the poses of the detected objects to be stored in the pose_selector, which is why it is required.

# Usage
Simply import the package in python:

    from symbolic_fact_generation.on_fact_generator import _init_class, get_current_facts, clear_facts_and_poses_for_table, 

_init_class(query_srv_str: str, delete_srv_str: str) -> can be used to change the pose_selector service topic names

get_current_facts() -> if objects poses are stored in the pose_selector, checks their pose and collision with the tables and creates a list of facts if the "on" condition is true

clear_facts_and_poses_for_table(table: str) -> used to clear the pose_selector and facts for a specific table