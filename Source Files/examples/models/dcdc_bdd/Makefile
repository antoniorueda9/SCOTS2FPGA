#
# compiler
#
CC	        = g++
# CXXFLAGS	= -Wall -Wextra -std=c++11 -O3 -DNDEBUG -DSCOTS_BDD
#
CXXFLAGS 		= -Wall -Wextra -std=c++11 -DSCOTS_BDD -D_GLIBCXX_ASSERTIONS -fvar-tracking -g3

# scots
#
SCOTSROOT	= ../../../ext/SCOTSv2.0
SCOTSINC	= -I$(SCOTSROOT)/src -I$(SCOTSROOT)/utils
#
# cudd
#
CUDD		= -I/usr/local/include -L/usr/local/lib -lcudd

.PHONY: dcdc simulate

TARGET = dcdc simulate

all: $(TARGET)

dcdc:
	$(CC) $(CXXFLAGS) $(SCOTSINC) dcdc.cc -o dcdc $(CUDD)

simulate:
	$(CC) $(CXXFLAGS) $(SCOTSINC) simulate.cc -o simulate $(CUDD)

clean:
	rm dcdc simulate
