cmake_minimum_required(VERSION 3.5)
project(RC10)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME})

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/resource)

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

ament_package()
