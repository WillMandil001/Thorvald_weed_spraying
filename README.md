# Thorvald_weed_spraying


export GAZEBO_MODEL_PATH=/home/<usename>/<workspace_name>/src/Thorvald_weed_spraying/CMP9767M/uol_cmp9767m_base/models:${GAZEBO_MODEL_PATH}

Gregs way to launch tests:

roslaunch uol_cmp9767m_base thorvald-sim.launch world_name:=`rospack find uol_cmp9767m_base`/worlds/<world_file>.world

New way to launch test scenarios:

roslaunch uol_cmp9767m_base thorvald-sim.launch test1:=true
(test cases from 1 to 4)
