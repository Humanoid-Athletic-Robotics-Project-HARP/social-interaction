# STVL in this repo

A copy of [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
(LGPL-2.1, see LICENSE), branch `jazzy`, commit `5837b16be2028907165e1f147e72ce76d37bdadd`
(2026-10-15). RoboStack has no build of it, so `pixi run build` builds it from here.

Our only changes are two macOS build fixes in `spatio_temporal_voxel_layer/CMakeLists.txt`:
Boost 1.89+ has no `system` library to find (header-only since 1.69), and macOS's linker has
no `--no-undefined`. To update, copy a newer upstream jazzy over this folder (without
its `.git`), reapply the diff below, and update the commit above.

```diff
diff --git a/spatio_temporal_voxel_layer/CMakeLists.txt b/spatio_temporal_voxel_layer/CMakeLists.txt
index 59467f5..0a1b23f 100644
--- a/spatio_temporal_voxel_layer/CMakeLists.txt
+++ b/spatio_temporal_voxel_layer/CMakeLists.txt
@@ -32,7 +32,7 @@ find_package(openvdb_vendor REQUIRED)
 
 remove_definitions(-DDISABLE_LIBUSB-1.0)
 find_package(Eigen3 REQUIRED)
-find_package(Boost REQUIRED COMPONENTS system thread)
+find_package(Boost REQUIRED COMPONENTS thread)
 
 find_package(PCL REQUIRED COMPONENTS filters common)
 if(NOT "${PCL_LIBRARIES}" STREQUAL "")
@@ -107,7 +107,9 @@ target_link_libraries(${library_name}
 )
 
 # Get a linker error when there are undefined symbols
-target_link_options(${library_name} PRIVATE -Wl,--no-undefined)
+if(NOT APPLE)   # macOS ld has no --no-undefined
+  target_link_options(${library_name} PRIVATE -Wl,--no-undefined)
+endif()
 
 ament_target_dependencies(${library_name} ${dependencies})
 
```
