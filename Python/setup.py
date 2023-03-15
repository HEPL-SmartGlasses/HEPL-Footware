from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize

# definitions
sourcefiles = ['wrapper.pyx']
extensions = [Extension('wrapper', 
    sourcefiles,  
    library_dirs=['.'],
    runtime_library_dirs=["."],
    libraries=["processing"])]

# setup function
setup(
    name='wrapper', 
    ext_modules=cythonize(extensions),
    include_dirs=['.']
    )



