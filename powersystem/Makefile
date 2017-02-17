# The name of the compiler that we want to use
CXX=g++

# Compiler flags
CXXFLAGS=$(INCDIRS) -std=c++11

# Linker flags
LDFLAGS=$(LIBDIRS) $(LIBS)

# List of source files that should be compiled into the final executable program
SOURCES=OverviewOne.cpp OperationalPlan.cpp PowerConsumption.cpp PowerGeneration.cpp

# List of header files that should be included while compiling
HEADERS=OperationalPlan.h PowerConsumption.h PowerGeneration.h

all: OverviewOne

OverviewOne: $(SOURCES) $(HEADERS)
	$(CXX) $(SOURCES) $(CXXFLAGS) $(LDFLAGS) -o OverviewOne

clean:
	rm -f *.o OverviewOne

