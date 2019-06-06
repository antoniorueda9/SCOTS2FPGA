#
# compiler
#
CC        = g++
# CC       	  = clang++
# CXXFLAGS 		= -g -Wall -Wextra -std=c++11
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG -DSCOTS_BDD
#
# scots
#
SCOTSROOT		= ../..
SCOTSINC		= -I$(SCOTSROOT)/src -I$(SCOTSROOT)/utils
#
# cudd
#
CUDD		    =  -I/usr/local/include -L/usr/local/lib -lcudd

.PHONY: aircraft simulate

TARGET = aircraft simulate

all: $(TARGET)

aircraft:
	$(CC) $(CXXFLAGS) $(SCOTSINC) aircraft.cc -o aircraft $(CUDD)

simulate:
	$(CC) $(CXXFLAGS) $(SCOTSINC) simulate.cc -o simulate $(CUDD)


clean:
	rm aircraft simulate
