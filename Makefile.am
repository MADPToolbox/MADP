ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src 
EXTRA_DIST = src/include/versions.h \
 src/include/configuration.h \
 src/parser/dpomdp.spirit \
 problems \
 doc \
 src/boost \
 src/solvers/GMAA-ICE

#removes subversion dirs before packing
dist-hook:
	rm -rf `find $(distdir) -name .svn`


bak:	bak-recursive

bak-recursive:
	list='$(SUBDIRS)'; for subdir in $$list; do \
	  test "$$subdir" = . || (cd $$subdir && $(MAKE) $(AM_MAKEFLAGS) bak); \
	done

htmldoc:
	doxygen doc/doxygen.conf;

package:
	rm -fr madp-0.3.1
	cp madp-0.3.1.tar.gz madp_0.3.1.orig.tar.gz
	tar xf madp_0.3.1.orig.tar.gz
	cp -r debian madp-0.3.1/
	cd madp-0.3.1 && debuild -us -uc
