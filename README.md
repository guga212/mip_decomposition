# mip_decomposition
Python based mip solving framework

Usage:
1) The framework uses specific Conda environment
2) Environment can be created in the directory
"environment/" using "environment/Makefile"
3) Framework internally uses several solvers which
are not available at the cond repository. Therefore, 
such solvers (CPLEX, BARON, Couenne) should be put
into the directory "/environment/solver" and added
to the generated environment using mentioned 
above Makefile.
