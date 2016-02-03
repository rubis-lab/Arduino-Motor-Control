

TARGET=core
CC  = g++
CFLAGS = -Wall -O1 -L$(LIB_PATH_OPENCV) -I$(INC_PATH_OPENCV) $(LIB_OPTION)

#sources
SOURCE = Test.cpp	\
	AnomalyDetector.cpp	\
	i2c_server.cpp	\
	SteerControl.cpp

#more includes
INCLUDE = -I.
INC_PATH_OPENCV = /home/ubuntu/opencv-2.4.11/include/opencv
LIB_PATH_OPENCV = /usr/local/lib
LIB_OPTION = -lopencv_core -lopencv_highgui -lopencv_ml -lopencv_imgproc -lpthread 

#dependency dest
VPATH=.
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
	@$(CC) $(CFLAGS) -c $< -o $@

../obj/%.o : %.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) -c $< -o $@

../.dep/%.d: %.cpp
	@mkdir -p $(dir $@)
	@echo $*.o
	@$(SHELL) -ec '$(CC) -M $(CFLAGS) $< | sed "s^$*.o^../obj/$*.o^" > $@'

-include $(DEPENDS)