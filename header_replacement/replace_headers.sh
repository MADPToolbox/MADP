find base/ planning/ support/ solvers/ parser/ include/ examples/ utils/ -regextype sed -regex "\(.*\.cpp\|.*\.h\)" -print0 | xargs -0 ./replace_header.sh
