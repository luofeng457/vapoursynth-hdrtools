# compile plugins and generate shared libary
#
# Author: Yanan Zhao <arthurchiao@hotmail.com>
# Date  : 2016-12-01 20:40:38
# Last Modified: 2016-12-07 17:36:06

DIR := ${CURDIR}
CFLAGS :=-I$(CURDIR)/../projects/HDRConvert/inc -I$(CURDIR)/../common/inc

SRC := HDRConvertYUVFilter
DEPENS_OBJ := ../common/obj/*.o ../projects/HDRConvert/obj/*.o
STATIC_LIB := libhdrconv.a
SHARED_LIB := libhdrconv.so

all:
	@rm -rf $(SRC).o $(STATIC_LIB) $(SHARED_LIB)
	@echo "compiling $(SRC).cpp ..."
	@g++ -fPIC -Wall $(CFLAGS) -c $(SRC).cpp -o $(SRC).o
	@echo "packing static library $(STATIC_LIB) ..."
	@ar rcs $(STATIC_LIB) $(DEPENS_OBJ) $(SRC).o
	@echo "packing dynamic library $(SHARED_LIB) ..."
	@g++ -fPIC -Wall $(CFLAGS) -shared -o $(SHARED_LIB) \
		-Wl,--whole-archive $(STATIC_LIB) -Wl,--no-whole-archive
	@echo "done"

fresh:
	cd .. && make && cd gen-plugin
	make all

clean:
	@rm -rf $(SRC).o
	@rm -rf $(STATIC_LIB) $(SHARED_LIB)
