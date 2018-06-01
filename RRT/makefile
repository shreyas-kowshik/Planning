CFLAGS = `pkg-config --cflags opencv`
LIBS = -pthread `pkg-config --libs opencv`
CC = g++
% : %.cpp
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)
