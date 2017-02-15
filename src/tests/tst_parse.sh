#!/bin/bash

./tst_parse -t > logs/tst_parse.log
diff logs/tst_parse.log logs/tst_parse.reference.log
