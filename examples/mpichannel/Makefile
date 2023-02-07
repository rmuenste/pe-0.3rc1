#==================================================================================================
#
#  Makefile for the mpichannel example
#
#  Copyright (C) 2014 Tobias Preclik
#
#  This file is part of pe.
#
#  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
#  General Public License as published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along with pe. If not,
#  see <http://www.gnu.org/licenses/>.
#
#==================================================================================================


# Compiler configuration
CXX      = g++
CXXFLAGS = -Wall -Werror -Wshadow -Woverloaded-virtual -ansi -O3  -Wextra -Wno-unused-local-typedefs --param inline-unit-growth=150 --param max-inline-insns-single=500 --param large-function-growth=600 -DNDEBUG
CPPFLAGS =  -include /auto-home/staff/preclik/tmp/pe-trunk/config.h -I/auto-home/staff/preclik/tmp/pe-trunk -I/auto-home/staff/preclik/tmp/pe-trunk

# Compiler environment
LDFLAGS =  -L/auto-home/staff/preclik/tmp/pe-trunk/lib
LIBS    = -lm -lrt -lpe -lboost_thread -lboost_system -lboost_filesystem -lboost_program_options

# Library search path
vpath lib%.a /auto-home/staff/preclik/tmp/pe-trunk/lib
vpath lib%.so /auto-home/staff/preclik/tmp/pe-trunk/lib


# Build rules
mpichannel: MPIChannel.cpp $(LIBS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) -o $@ MPIChannel.cpp $(LIBS)


# Clean up rules
clean:
	@echo "Cleaning up..."
	rm -f mpichannel


# ----- Setting the independent commands -----
.PHONY: clean
