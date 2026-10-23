cmake_minimum_required(VERSION 3.10)
project(arm_bst CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pruebas de archivos, Merge Sort/Busqueda Binaria y BST (no necesitan Gazebo)
add_executable(pruebas_datos pruebas/pruebas_datos.cc)
target_include_directories(pruebas_datos PRIVATE ${CMAKE_SOURCE_DIR})
enable_testing()
add_test(NAME pruebas_datos COMMAND pruebas_datos WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# Programa de control (necesita Gazebo Harmonic: gz-transport13 y gz-msgs10).
# Si tienes otras versiones instaladas, cambia los nombres de los paquetes.
find_package(gz-transport13 QUIET COMPONENTS core)
find_package(gz-msgs10 QUIET)
if(gz-transport13_FOUND AND gz-msgs10_FOUND)
  add_executable(control_bst control_bst.cc)
  target_link_libraries(control_bst gz-transport13::core gz-msgs10::gz-msgs10)
else()
  message(WARNING "No se encontro gz-transport13/gz-msgs10: solo se compilan las pruebas.")
endif()
