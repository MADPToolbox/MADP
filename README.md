[![Build Status](https://travis-ci.org/MADPToolbox/MADP.svg?branch=master)](https://travis-ci.org/MADPToolbox/MADP)

MADP
====

MultiAgentDecisionProcess (MADP) is a toolbox for scientific research
in decision-theoretic planning and learning in multiagent systems.  It
is designed to be rather general, but most effort has been put in
planning algorithms for discrete Dec-POMDPs.

Comments, bug reports, patches, etc, are welcome. A mailinglist is
available at madp-users@isr.ist.utl.pt.

Authors:
* Frans Oliehoek,
* Matthijs Spaan,
* João Messias,
* Philipp Robbel

Please refer to the file [AUTHORS](AUTHORS) for contact information and current 
affiliations. MADP includes other software, for details see [COPYING](COPYING).


### Required software (as Debian package names)
-----------------------------------------------

MADP includes all required software.

Optional software:

* Doxygen (doxygen) [for generating documentation]
* Graphviz (graphviz) [for dependency graphs in the generated documentation]
* lp-solve (liblpsolve55-dev and libufsparse-dev) [see AgentIDMG]
* Matlab [can also be used a linear programming solver in AgentIDMG]
* GMPlib (libgmp3-dev) [for using arbitrary-length integers as indices] 
* libxml2 (libxml2-dev) [for using the XML-based factored model parser]
* Cuda [for policy iteration with GPU policy evaluation]
* cplex [for the DP-LPC solver]

For enabling optional software, see [src/Makefile.custom](src/Makefile.custom) and
[src/include/configuration.h](src/include/configuration.h).

The lazy-bastards-copy-paste-line to set up most debian-based users:
```
sudo aptitude install automake autoconf libtool ccache gcc g++ \
libboost-dev libboost-serialization-dev libboost-test-dev \
libsuitesparse-dev libgsl0-dev \
libgmp3-dev libxml2-dev doxygen graphviz 
```

### Documentation and Installation Instructions
-----------------------------------------------

See `doc/MADPToolbox-0.XXX.pdf`, which also includes detailed 
installation instructions.

To (re)generate API documentation from source, run:
```
make htmldoc
```
Open `doc/html/index.html` in a webbrowser

To compile, execute the following
```
sh autogen.sh
./configure
make
```

Problem descriptions can be loaded without specifying a path if
`~/.madp/problems` is a symlink to the problems subdir in the MADP tree.
Similarly, results are saved in (subdirs of) `~/.madp/results`, so it will
be convenient to make a symlink to the desired results locations.
This can be accomplished as follows:
```
mkdir ~/.madp
cd ~/.madp
ln -s ~/<PATHTOMADP>/problems
ln -s ~/<PATHTOMADP>/results
```

### Acknowledgments
-------------------

Please see the [AUTHORS](AUTHORS) file.
