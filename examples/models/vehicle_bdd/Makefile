#
# compiler
#
CC        = g++
# CC       	  = clang++
CXXFLAGS 		= -Wall -Wextra -std=c++11 -O3 -DNDEBUG -DSCOTS_BDD
#
# scots
#
SCOTSROOT		= ../../../ext/SCOTSv2.0
SCOTSINC		= -I$(SCOTSROOT)/src -I$(SCOTSROOT)/utils
#
# cudd
#
CUDD		    =  -I/usr/local/include -L/usr/local/lib -lcudd


.PHONY: vehicle simulate

TARGET = vehicle simulate

all: $(TARGET)

vehicle:
	$(CC) $(CXXFLAGS) $(SCOTSINC) vehicle.cc -o vehicle $(CUDD)

simulate:
	$(CC) $(CXXFLAGS) $(SCOTSINC) simulate.cc -o simulate  $(CUDD)

clean:
	rm vehicle simulate *.scs *.bdd
