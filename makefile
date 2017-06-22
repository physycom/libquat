FILES      = 1-2-rotazioni.cpp 3-inertial2d.cpp 4-inertial3d.cpp 
LIB_FILES  = src/quat.cpp
EXE        = $(addprefix bin/, $(addsuffix .exe, $(basename $(FILES)))) 
LIB        = $(addprefix obj/, $(addsuffix .o, $(basename $(LIB_FILES))))
OPT_CXX    = -std=c++11


all : dirtree $(EXE) $(LIB)

dirtree:
	@mkdir -p bin
	@mkdir -p obj

obj/quat.o : src/quat.cpp
	$(CXX) $(OPT_CXX) -I.. -c -o $@ $<

bin/%.exe : test/%.cpp $(LIB)
	$(CXX) $(OPT_CXX) -I.. -o $@ $< $(LIB)

clean :
	rm -f $(LIB) $(EXE)

cleanall :
	rm -rf bin obj
