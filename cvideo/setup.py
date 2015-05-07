from distutils.core import setup, Extension

module1 = Extension('cvideo',
                    sources = ['cvideo.cpp'],
                    include_dirs = [ r'c:\Python27\Lib\site-packages\numpy\core\include',
                      r'm:\git\cvdrone\src\3rdparty\ffmpeg\include',
                      "/usr/lib/python2.7/dist-packages/numpy/core/include",
                      "/usr/include" ],
                    define_macros = [ ('__STDC_CONSTANT_MACROS', None) ],
                    library_dirs = [ r'm:\git\cvdrone\src\3rdparty\ffmpeg\lib' ],
                    libraries = [ 'avcodec', 'avutil', 'swscale' ],
                    )

setup (name = 'CVideoPackage',
       version = '0.1',
       description = 'C-video frame by frame reader',
       ext_modules = [module1])

