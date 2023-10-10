CC=gcc
CXX=g++

DIR = /home/c3po/cpp_robotic_arm
BUILD_DIR = /home/c3po/cpp_robotic_arm/build

INCLUDE = 	-I$(DIR)/include \
			-I$(DIR)/libfranka/include \
			-I/usr/include/eigen3/ \
			-I$(DIR)/libfranka/examples \
			-I$(DIR)/liborl/include
CXXFLAGS = -g -Wall -Wextra $(INCLUDE) 

SHARED_LIBRARIES = $(DIR)/libfranka/build/libfranka.so $(DIR)/liborl/build/liborl.so

TARGET = get_arm_status

#==============================================#
#                    MAIN                      #
#==============================================#
ifeq ($(TARGET), main)
SRCS = $(wildcard src/*.cpp)

OBJS := $(patsubst %.cpp,%.o,$(SRCS:src/%=%))

all: $(TARGET) 
$(TARGET): $(OBJS) examples_common.o
	$(CXX) $(CXXFLAGS) -o $@ $^ -lpthread $(SHARED_LIBRARIES)
%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif

#==============================================#
#                    TESTS                     #
#==============================================#

ifeq ($(TARGET),generate_1d_mvt)
generate_1d_mvt: examples_common.o generate_1d_mvt.o $(SHARED_LIBRARIES)
	$(CXX) $(CXXFLAGS) -o $@ $^

generate_1d_mvt.o: $(DIR)/tests/generate_1d_mvt.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif


ifeq ($(TARGET),comm_test)
comm_test: examples_common.o communication_test.o $(SHARED_LIBRARIES)
	$(CXX) $(CXXFLAGS) -o $@ $^

communication_test.o: $(DIR)/tests/comms_test.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif


ifeq ($(TARGET),get_arm_status)
get_arm_status: examples_common.o get_arm_status.o $(SHARED_LIBRARIES)
	$(CXX) $(CXXFLAGS) -o $@ $^

get_arm_status.o: $(DIR)/tests/get_arm_status.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif


ifeq ($(TARGET),controller_test)
controller_test: controller_test.o
	$(CXX) $(CXXFLAGS) -o $@ $^

controller_test.o: $(DIR)/tests/controller_test.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif

ifeq ($(TARGET),bezier_test)
bezier_test: bezier_test.o $(SHARED_LIBRARIES)
	$(CXX) $(CXXFLAGS) -o $@ $^

bezier_test.o: $(DIR)/tests/bezier_test.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif

ifeq ($(TARGET),tcp_ipc_test)
tcp_ipc_test: tcp_ipc_test.o $(SHARED_LIBRARIES)
	$(CXX) $(CXXFLAGS) -o $@ $^

tcp_ipc_test.o: $(DIR)/tests/tcp_ipc_test.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif

ifeq ($(TARGET),eepos_traj_test)
eepos_traj_test: eepos_traj_test.o $(SHARED_LIBRARIES)
	$(CXX) $(CXXFLAGS) -o $@ $^

eepos_traj_test.o: $(DIR)/tests/eepos_traj_test.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@
endif


examples_common.o: $(DIR)/libfranka/examples/examples_common.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm *.o
	rm $(TARGET)
