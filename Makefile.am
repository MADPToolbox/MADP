AUTOMAKE_OPTIONS = foreign
ACLOCAL_AMFLAGS = -I m4
SUBDIRS = src 
EXTRA_DIST = src/include/versions.h \
 src/include/configuration.h \
 src/parser/dpomdp.spirit \
 src/parser/pomdp.spirit \
 problems \
 doc \
 src/boost \
 src/solvers/GMAA-IC \
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

MADPVER="0.4.1"
package:
	rm -fr madp-$(MADPVER)
	cp madp-(MADPVER).tar.gz madp_(MADPVER).orig.tar.gz
	tar xf madp_(MADPVER).orig.tar.gz
	cp -r debian madp-(MADPVER)/
	cd madp-(MADPVER) && debuild -us -uc
