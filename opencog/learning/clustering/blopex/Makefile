include Makefile.inc 

DIRS = multivector utilities

HEADERS = \
multivector/interpreter.h \
multivector/multivector.h \
lobpcg.h \
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

$(LIB) : 
	@echo  "Building $@ ... "
	cd lib && ${AR} $@ ./*.o 
	cd lib && ${RANLIB} $@
