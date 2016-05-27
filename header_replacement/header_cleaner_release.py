#!/bin/python
#
# cleans up old style headers. Run from /src.
#
import subprocess

def do_replace(filename):
    print "-->doing replace header for '%s'" % filename 
    sedc = ['sed', "--in-place", "-f", "../header_replacement/header_cleaner_release.sed", filename ]
    print sedc
    try:
        #res = ''
        #res = subprocess.check_output(sedc,shell=True)
        res = subprocess.check_output(sedc)
    except subprocess.CalledProcessError:
        print ">>>REPLACE FAILED!?"
        print res
        return False
    print "replace done:"
    print res



def already_replaced(filename):
    try:
        cmd = ["""/bin/grep REPLACE_MADP_HEADER %s """ % filename ]
        res = subprocess.check_output(cmd,shell=True)
    except subprocess.CalledProcessError:
        # did not get output...
        return False
    #print "res = ", res
    return True


def replace_single_file(filename):
    s = "processing %s..." % filename 
    print s
    if already_replaced(filename):
        print "\talready has newstyle header... skipping!" 
    else:
        do_replace(filename)

   
def find_all_CPP_files():
    #fc = ['find', 'base/ planning/ support/ solvers/ parser/ include/ examples/ utils/ -regextype sed -regex "\(.*\.cpp\|.*\.h\)" ']
    fc = ['find', 'base/', 'planning/', '-regextype' , 'sed', '-regex',  '"\(.*\.cpp\|.*\.h\)"' ]
    fc = ['find', 'base/', 'planning/', 'support/', 'solvers/', 'parser/', 'include/', 'examples/', 'utils/',           '-regextype' , 'sed', '-regex',  '\(.*\.cpp\|.*\.h\)' ]
    out = subprocess.check_output(fc)
    #print out
    l = out.split()
    return l


def clean_all_CPP_files():
    fs = find_all_CPP_files()
    print fs
    for f in fs:
        replace_single_file(f)

import sys
if __name__ == '__main__':
    argcount = len(sys.argv)
    print 'Number of arguments:', argcount, 'arguments.'
    print 'Argument List:', str(sys.argv)

    if argcount == 1: #always 1 argument, the name of this script...
        print "received no arguments, assuming you want me to clean all *.h and *.cpp files..."
        clean_all_CPP_files()
    if argcount == 2: 
        fn = sys.argv[1] 
        print "renaming %s..." % fn
        rename_pdf_file(fn)

