#!/bin/bash
cd ../src && 
    echo "starting linking..." &&
    ln -s ../header_replacement/replace_headers.sh && 
    ln -s ../header_replacement/replace_header.sh && 
    ln -s ../header_replacement/replace_header.sed && 
    ln -s ../header_replacement/Template_HEADER && 
    ln -s ../header_replacement/Template_CONTRIBUTING_AUTHORS_START && 
    ln -s ../header_replacement/Template_CONTRIBUTING_AUTHORS_END &&
    echo "linking done, starting replacement..." &&
    sh ./replace_headers.sh &&
    echo "replacing done, starting cleanup..." &&
    rm replace_header.sed replace_header.sh replace_headers.sh Template_CONTRIBUTING_AUTHORS_END Template_CONTRIBUTING_AUTHORS_START Template_HEADER && 
    cd ../header_replacement

    

