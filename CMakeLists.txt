cmake_minimum_required(VERSION 3.16)

project(ADA_PWM_DRIVER
VERSION 2.4.0
LANGUAGES CXX
)

find_package(pigpio CONFIG REQUIRED)

add_library(pwm_driver STATIC 
	"Adafruit_PWMServoDriver.cpp"
)
target_include_directories(pwm_driver PUBLIC 
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
	"$<INSTALL_INTERFACE:include>"
)
target_link_libraries(pwm_driver PUBLIC 
	pigpio::pigpio
)
# We use some threading functions, need c++11
target_compile_features(pwm_driver PUBLIC cxx_std_11)


include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(PWM_DRIVER_PACKAGE_NAME adafruit_pwm_driver)
set(RELATIVE_CONFIG_DIR "share/${PWM_DRIVER_PACKAGE_NAME}")

install(TARGETS pwm_driver
    EXPORT ${PWM_DRIVER_PACKAGE_NAME}Targets
	LIBRARY  DESTINATION lib/$<CONFIG>
	ARCHIVE  DESTINATION lib/$<CONFIG>
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/${PWM_DRIVER_PACKAGE_NAME}ConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${PWM_DRIVER_PACKAGE_NAME}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${PWM_DRIVER_PACKAGE_NAME}Targets.cmake"
  NAMESPACE adafruit::
)

install(EXPORT ${PWM_DRIVER_PACKAGE_NAME}Targets
  FILE
    ${PWM_DRIVER_PACKAGE_NAME}Targets.cmake
  NAMESPACE
    adafruit::
  DESTINATION
    ${RELATIVE_CONFIG_DIR}
)

install(
  FILES
    "${CMAKE_CURRENT_LIST_DIR}/cmake/${PWM_DRIVER_PACKAGE_NAME}-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PWM_DRIVER_PACKAGE_NAME}ConfigVersion.cmake"
  DESTINATION
    ${RELATIVE_CONFIG_DIR}
)

install(
	FILES 
		"Adafruit_PWMServoDriver.h"
	DESTINATION 
		"${CMAKE_INSTALL_INCLUDEDIR}"
)

include (CPack)