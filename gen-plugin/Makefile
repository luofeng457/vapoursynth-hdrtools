# compile plugins and generate shared libary
#
# Author: Yanan Zhao <arthurchiao@hotmail.com>
# Date  : 2016-12-01 20:40:38

DIR := ${CURDIR}
CFLAGS :=-I$(CURDIR)/../projects/HDRConvert/inc -I$(CURDIR)/../common/inc

all:
	g++ -fPIC -Wall $(CFLAGS) -c ./HDRConvertYUVFilter.cpp -o ./HDRConvertYUVFilter.o
	g++ -fPIC -Wall $(CFLAGS) -shared -o libhdrconv.so -Wl,--whole-archive \
		../lib/HDRLib.a -Wl,--no-whole-archive \
		../projects/HDRConvert/obj/ProjectParameters.o \
		HDRConvertYUVFilter.o

clean:
	@rm ./HDRConvertYUVFilter.o
	@rm ./libhdrconv.so
