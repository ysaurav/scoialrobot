FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/openni_camera_deprecated/OpenNIConfig.h"
  "../docs/OpenNIConfig.dox"
  "../docs/OpenNIConfig-usage.dox"
  "../src/openni_camera_deprecated/cfg/OpenNIConfig.py"
  "../docs/OpenNIConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
