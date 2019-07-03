import sys
import sysconfig
import site

if __name__ == '__main__':
    # This is a hack due to the distutils in debian/ubuntu's python3 being misconfigured
    # see discussion https://github.com/opencog/atomspace/issues/1782
    #
    # If the bug is fixed, this script could be replaced by:
    #
    # from distutils.sysconfig import get_python_lib; print(get_python_lib(plat_specific=True, prefix=prefix))
    #
    # However, using this would not respect a python virtual environments, so in a way this is better!

    prefix = sys.argv[1]

    # use sites if the prefix is recognized and the sites module is available
    # (virtualenv is missing getsitepackages())
    if hasattr(site, 'getsitepackages'):
        paths = [p for p in site.getsitepackages() if p.startswith(prefix)]
        if len(paths) == 1:
            print(paths[0])
            exit(0)
    
    # use sysconfig platlib as the fall back
    print(sysconfig.get_paths()['platlib'])
