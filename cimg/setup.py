from distutils.core import setup, Extension

module1 = Extension('cimg',
                    sources = ['cimg.c'],
                    include_dirs = [r'c:\Python27\Lib\site-packages\numpy\core\include']
                    )

setup (name = 'CImgPackage',
       version = '0.1',
       description = 'C-image helper function',
       ext_modules = [module1])

