FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/jacobian_listener/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/jacobian_listener/srv/__init__.py"
  "../src/jacobian_listener/srv/_ReturnJacobian.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
