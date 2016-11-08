ifeq ($(OS),Darwin)
CC=g++
else
CC=g++
endif

OS := $(shell uname)

SRC_DIR = src
BUILD_DIR = build

CPP_FILES = $(wildcard $(SRC_DIR)/*.cpp)
#CPP_FILES += $(wildcard $(SRC_DIR)/gpio/*.cpp)
#CPP_FILES += $(wildcard $(SRC_DIR)/display/*.cpp)
OBJ_FILES := $(addprefix build/obj/,$(notdir $(CPP_FILES:.cpp=.o)))
LD_FLAGS :=  -lpthread
#LC_FLAGS := -std=c++11 -Wall 

ifneq ($(OS),Darwin)
#LD_FLAGS += -lgoldeloxSerial -lboost_thread
else 
#LD_FLAGS += -lboost_thread-mt
endif

app: $(OBJ_FILES)
	g++ $(CC_FLAGS)-o $(BUILD_DIR)/$@ $^ $(LD_FLAGS) 

build/obj/%.o: src/%.cpp 
	g++ $(CC_FLAGS) -c -o $@ $<

build/obj/%.o: src/gpio/%.cpp 
	g++ $(CC_FLAGS) -c -o $@ $<

clean:
	rm -rf build/obj/* && rm build/app

data-clean:
	sudo rm -rf ./data/*

run:
	cd build && sudo ./app

doc:
	doxygen utils/Doxyfile.in
