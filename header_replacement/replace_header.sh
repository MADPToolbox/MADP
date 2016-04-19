#!/bin/bash
#if we want backups:
#sed -f replace_header.sed --in-place=".backup" $@
sed -f replace_header.sed --in-place $@
