CXX      := -clang++
CXXFLAGS := -std=c++17 -shared -undefined dynamic_lookup `python3.9 -m pybind11 --includes`
LDFLAGS  := `python3.9-config --ldflags`
OBJ_DIR  := objects
TARGET   := liblander.so
INCLUDE  := -I../models/math/ -I../models/dyn_body -I../trick
SRC      :=                       \
   ../models/dyn_body/DynBody.cpp \
   ../models/eom/EoM.cpp          \
   ../models/fam/FaM.cpp          \
   ../models/pybind/PyBind.cpp    \
   ../models/vehicle/Lander.cpp    \
   ../models/math/MathUtils.cpp    \
   ../models/math/PIDController.cpp    \
   ../models/gnc/control/LVLHAttCtl.cpp    \
#   $(wildcard src/math/*.cpp) \
#   $(wildcard src/*.cpp)         \

OBJECTS  := $(SRC:%.cpp=$(OBJ_DIR)/%.o)
DEPENDENCIES \
         := $(OBJECTS:.o=.d)

all: build $(TARGET)

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -MMD -o $@

$(TARGET): $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $^ $(LDFLAGS)

-include $(DEPENDENCIES)

.PHONY: all build clean debug release info

build:
	@mkdir -p $(OBJ_DIR)

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O2
release: all

clean:
	-@rm -rvf $(OBJ_DIR)
	-@rm -rvf $(TARGET)

info:
	@echo "[*] Target:          ${TARGET}      "
	@echo "[*] Object dir:      ${OBJ_DIR}     "
	@echo "[*] Sources:         ${SRC}         "
	@echo "[*] Objects:         ${OBJECTS}     "
	@echo "[*] Dependencies:    ${DEPENDENCIES}"
