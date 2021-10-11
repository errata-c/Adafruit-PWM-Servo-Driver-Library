include(CMakeFindDependencyMacro)

# Make sure we have the necessary dependencies.
if(NOT TARGET pigpio::pigpio)
	find_dependency(pigpio CONFIG)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/adafruit_pwm_driverTargets.cmake")
