pybind11_add_module(se3_lio_pybind MODULE se3_lio_pybind.cpp)
target_link_libraries(
  se3_lio_pybind
  PRIVATE se3_lio::pipeline
          se3_lio::core
          se3_lio::synchronizer)

# Colouring/BEV is compiled in only when the core built se3_lio::visual (i.e.
# OpenCV was found). Otherwise the module stays OpenCV-less, as the wheel needs.
if(TARGET se3_lio::visual)
  target_link_libraries(se3_lio_pybind PRIVATE se3_lio::visual)
  target_compile_definitions(se3_lio_pybind PRIVATE SE3_LIO_WITH_VISUAL)
  set_target_properties(visual PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

# The core is built as SHARED libraries that the core's own CMakeLists installs
# under <install-dir>/lib/se3_lio. The module sits at <install-dir>/, so point
# its RPATH there; the core libs depend on each other in the same dir ($ORIGIN).
set_target_properties(se3_lio_pybind PROPERTIES
  INSTALL_RPATH "$ORIGIN/lib/se3_lio")
set_target_properties(pipeline core synchronizer PROPERTIES
  INSTALL_RPATH "$ORIGIN")

install(TARGETS se3_lio_pybind DESTINATION .)
