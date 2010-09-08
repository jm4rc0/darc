#mklmodule is a module that lets one use Intel MKL SVD and GEMM routines.

from distutils.core import setup, Extension
import sys,os.path,os,string
idnumpy=[sys.prefix+'/lib/python%d.%d/site-packages/numpy/core/include'%(sys.version_info[0],sys.version_info[1]),sys.prefix+'/include']
if not os.path.exists(idnumpy[0]):
    idnumpy=[sys.prefix+'/lib64/python%d.%d/site-packages/numpy/core/include'%(sys.version_info[0],sys.version_info[1]),sys.prefix+'/include']
    #print "Using %s"%str(idnumpy)
ld=[sys.prefix+'/lib']

shm=Extension('utilsmodule',
              include_dirs=idnumpy,
              library_dirs=ld,
              libraries=["rt"],
              extra_link_args=["-lrt"],
              sources=["utils.c"],
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
