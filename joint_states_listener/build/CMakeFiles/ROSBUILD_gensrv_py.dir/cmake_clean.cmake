FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/joint_states_listener/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/joint_states_listener/srv/__init__.py"
  "../src/joint_states_listener/srv/_ReturnJointStates.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
