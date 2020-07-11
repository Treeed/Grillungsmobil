# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)


#Change this to what you chose during install
set(ENV{IDF_PATH} "C:\\esp32deps\\auto\\esp-idf") #The ESP-IDF location
set(PYTHON_PATH "C:\\Python37") #The systems python installation
set(GIT_PATH "C:\\Program Files\\Git\\cmd") #The systems git location
set(DOT_EXPRESSIF "C:\\esp32deps\\auto\\.espressif") #The expressif tools folder

include(espmake/esp_include.cmake)
include(C:\\esp32deps\\auto\\esp-idf/tools/cmake/project.cmake)
project(untitled C CXX ASM)