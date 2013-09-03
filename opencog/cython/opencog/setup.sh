# setup.sh
# Make the 'pymoses' module ('pymoses.so')

# ****** INSTRUCTIONS: Make sure to set the -L parameter after LDFLAGS to the directory where the moses.so shared library is located on your computer

CC="gcc"   \
CXX="g++"   \
CFLAGS="-I../../learning/moses/service/ -I../../../ -I../../../DEPENDENCIES/python2.7/inc -I../../../DEPENDENCIES/gsl-1.15"   \
#LDFLAGS="-L/home/cosmo/opencog/src/qtbin/opencog/learning/moses/"   \
    python setup.py build_ext --inplace