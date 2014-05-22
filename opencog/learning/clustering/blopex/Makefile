include Makefile.inc 

DIRS = multivector krylov utilities

HEADERS = \
multivector/interpreter.h \
multivector/multivector.h \
krylov/lobpcg.h \
multivector/temp_multivector.h \
utilities/fortran_matrix.h \
utilities/fortran_interpreter.h \
utilities/fortran_options.h

LIB = libBLOPEX.${AR_LIB_SUFFIX}

##################################################################
# Targets
##################################################################

all: $(LIB) 
	mkdir -p include
	cp -fp  ${HEADERS} include

objects: 
	@ \
        mkdir -p lib; \
        for i in ${DIRS}; \
        do \
          echo "Making $$i ..."; \
          (cd $$i && $(MAKE)); \
          (cp -fp $$i/***.o ./lib/); \
          echo ""; \
        done

clean: 
	@ \
	rm -f lib/*; \
        for i in ${DIRS}; \
        do \
          if [ -d $$i ]; \
          then \
            echo "Cleaning $$i ..."; \
            (cd $$i &&  $(MAKE) $@); \
          fi; \
        done
	

##################################################################
# Rules
##################################################################

$(LIB) : objects 
	@echo  "Building $@ ... "
	cd lib && ${AR} $@ ./*.o 
	cd lib && ${RANLIB} $@
