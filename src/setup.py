#darc, the Durham Adaptive optics Real-time Controller.
#Copyright (C) 2010 Alastair Basden.

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU Affero General Public License as
#published by the Free Software Foundation, either version 3 of the
#License, or (at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Affero General Public License for more details.

#You should have received a copy of the GNU Affero General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

#mklmodule is a module that lets one use Intel MKL SVD and GEMM routines.
import platform
from distutils.core import setup, Extension
import sys,os.path,os,string
import numpy
src=os.path.split(numpy.__file__)[0]+"/core/include"
idnumpy=[src,sys.prefix+'/include']
if not os.path.exists(idnumpy[0]):
    print "numpy headers noit in usual place - guessing..."
    idnumpy=[sys.prefix+'/lib/python%d.%d/site-packages/numpy/core/include'%(sys.version_info[0],sys.version_info[1]),sys.prefix+'/include']
if not os.path.exists(idnumpy[0]):
    idnumpy=[sys.prefix+'/lib64/python%d.%d/site-packages/numpy/core/include'%(sys.version_info[0],sys.version_info[1]),sys.prefix+'/include']
    #print "Using %s"%str(idnumpy)
if not os.path.exists(idnumpy[0]):
    idnumpy=["/Library/Python/%d.%d/site-packages/numpy/core/include/"%(sys.version_info[0],sys.version_info[1]),sys.prefix+"/include"]
if not os.path.exists(idnumpy[0]):
    print "Could not find numpy headers - trying to compile, but may get segmentation fault (depending on unix distro)"
ld=[sys.prefix+'/lib']
defines=[]
libraries=["rt"]
extraLinkArgs=["-lrt"]
if platform.system()=="Darwin":
    print "A mac - will compile without consistency"
    defines.append(("NOCONSISTENCY",None))
    libraries=[]
    extraLinkArgs=[]
shm=Extension('utilsmodule',
              include_dirs=idnumpy,
              library_dirs=ld,
              libraries=libraries,
              extra_link_args=extraLinkArgs,
              sources=["utils.c"],
              define_macros=defines
              )
ext_modules=[shm]
#extracompargs=None
#if "--debug" in sys.argv:
#    sys.argv.remove("--debug")
#    extracompargs=["-DDEBUG"]
#if "--timing" in sys.argv:
#    sys.argv.remove("--timing")
#    if extracompargs==None:
#        extracompargs=["-DDOTIMING"]
#    else:
#        extracompargs.append("-DDOTIMING")
#cont=0
#if os.path.exists("/opt/intel/mkl"):
#    versions=os.listdir("/opt/intel/mkl")
#    versions=map(lambda x:string.split(x,"."),versions)
#    versions.sort()
#    if len(versions)>0:
#        version=string.join(versions[-1],".")
#    
#        mklinclude=["/opt/intel/mkl/%s/include"%version]
#        mkllib=["/opt/intel/mkl/%s/lib/em64t"%version]
#        print "Using MKL /opt/intel/mkl/%s/lib/em64t"%version
#        cont=1
#if cont==0:
#    print "MKL library not found - not making mklmodule"
#else:
#    core=Extension('coremodule',
#                  include_dirs=idnumpy+mklinclude,
#                  library_dirs=ld+mkllib,
#                  libraries=["mkl_intel_lp64","mkl_intel_thread","mkl_core","guide","pthread"],
#                   extra_compile_args=extracompargs,#["-DMKL_ILP64"],
#                  extra_link_args=["-lmkl_intel_lp64","-lmkl_intel_thread","-lmkl_core","-lguide","-lpthread","-lm"],
#                  sources=["coremodule.c","circ.c"]
#                  )
#    ext_modules.append(core)
setup (ext_modules = ext_modules)
