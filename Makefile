UNAME := $(shell uname)

MacGLFramework = -framework Carbon -framework OpenGL -framework GLUT
LinuxGLFramework = -lglut -lGLU
GLFramework = NOTYETSET

Include = -I/usr/include/GL -I/usr/include -I /usr/local/Cellar/eigen/3.2.4/include/eigen3/
Link = -L/usr/local/lib -lm 
Flags = -g -o  
Sources = *.cpp

CC = g++
ExeName = main

# Define proper GL framework
ifeq ($(UNAME), Linux)
	GLFramework = $(LinuxGLFramework)
endif
ifeq ($(UNAME), Darwin)
	GLFramework = $(MacGLFramework)
endif

all: main

# Compiling
main:
	$(CC) $(Flags) $(ExeName) $(Include) $(Link) $(Sources) $(GLFramework) 
	./$(ExeName)

clean:
	rm -f $(ExeName)
