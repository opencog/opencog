CUDAROOT = /pkgs/cuda
NVCC = $(CUDAROOT)/bin/nvcc

INCLUDE = include

NVCCFLAGS = -arch=compute_13 -lm

all: kerneltest destin alloctest

destin:
	$(NVCC) -I$(INCLUDE) $(NVCCFLAGS) -o bin/destin src/node.cu src/destin.cu src/main.cu

kerneltest:
	$(NVCC) -I$(INCLUDE) $(NVCCFLAGS) -o bin/kerneltest src/node.cu src/destin.cu src/kerneltest.cu

alloctest:
	$(NVCC) -I$(INCLUDE) $(NVCCFLAGS) -o bin/allocatetest src/node.cu src/destin.cu src/allocatetest.cu

clean:
	rm bin/* 
