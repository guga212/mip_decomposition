from .milpsolvers import GlpkSolver
from .miqppsolver import CplexSolver
from .minlpsolvers import CouenneSolver

import os as __os_hidden__
import sys as __sys_hidden__
exec_dir = __sys_hidden__.exec_prefix+'/bin'
lib_dir = __sys_hidden__.exec_prefix+'/lib'
if "LD_LIBRARY_PATH" in __os_hidden__.environ:
    if __os_hidden__.environ["LD_LIBRARY_PATH"].find(exec_dir) == -1:
        __os_hidden__.environ["LD_LIBRARY_PATH"] = f'{__os_hidden__.environ["LD_LIBRARY_PATH"]}:{exec_dir}'
    if __os_hidden__.environ["LD_LIBRARY_PATH"].find(lib_dir) == -1:
        __os_hidden__.environ["LD_LIBRARY_PATH"] = f'{__os_hidden__.environ["LD_LIBRARY_PATH"]}:{lib_dir}'
else:
    __os_hidden__.environ["LD_LIBRARY_PATH"] = f'{exec_dir}:{lib_dir}'