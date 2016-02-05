TARGET=core
CC  = g++
CFLAGS = -Wall -O1 -std=c++11 -L$(LIB_PATH_OPENCV) -I$(INC_PATH_OPENCV) $(LIB_OPTION) `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

#sources
SOURCE = Test.cpp	\
	KFilter.cpp	\
	i2c_server.cpp	\
	SteerControl.cpp \
	LaneDetect/laneDetect.cpp

#more includes
INCLUDE = -I.
INCLUDE = -I./LaneDetect

#INC_PATH_OPENCV = /home/ubuntu/opencv-2.4.11/include/opencv
INC_PATH_OPENCV = /usr/local/include/opencv
#INC_PATH_OPENCV = /usr/local/include/opencv2
LIB_PATH_OPENCV = /usr/local/lib
LIB_OPTION = -lopencv_core -lopencv_highgui -lopencv_ml -lopencv_imgproc -lpthread 

#dependency dest
VPATH=.:./LaneDetect
OBJ=$(join $(addsuffix ../obj/, $(dir $(SOURCE))), $(notdir $(SOURCE:.cpp=.o))) 
DEPENDS=$(join $(addsuffix ../.dep/, $(dir $(SOURCE))), $(notdir $(SOURCE:.cpp=.d)))


all: $(TARGET)
	@true

clean:
	@-rm -f $(TARGET) $(OBJ) $(DEPENDS)

$(TARGET): $(OBJ)
	@echo "Linking the target $@"
	@$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

%.o : %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) -c $< -o $@ $(LIBS)

../obj/%.o : %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) -c $< -o $@ $(LIBS)

./LaneDetect/../obj/%.o : %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) -c $< -o $@ $(LIBS)

../.dep/%.d: %.cpp
	@mkdir -p $(dir $@)
	@echo $*.o
	@$(SHELL) -ec '$(CC) -M $(CFLAGS) $< | sed "s^$*.o^../obj/$*.o^" > $@' $(LIBS)

./LaneDetect/../.dep/%.d: %.cpp
	@mkdir -p $(dir $@)
	@echo $*.o
	@$(SHELL) -ec '$(CC) -M $(CFLAGS) $< | sed "s^$*.o^$(OTHERDIR)/../obj/$*.o^" > $@' $(LIBS)

-include $(DEPENDS)