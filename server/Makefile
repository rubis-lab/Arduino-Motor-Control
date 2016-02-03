INC_PATH_OPENCV = /home/ubuntu/opencv-2.4.11/include/opencv
LIB_PATH_OPENCV = /usr/local/lib

LIB_OPTION = -lopencv_core -lopencv_highgui -lopencv_ml -lopencv_imgproc -lpthread 

CC  = g++

main :
	$(CC) -o main main.cpp -L$(LIB_PATH_OPENCV) -I$(INC_PATH_OPENCV) $(LIB_OPTION)

clean :
	@rm -rf *.o main 

