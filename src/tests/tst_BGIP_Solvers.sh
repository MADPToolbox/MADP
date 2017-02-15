#!/bin/bash

./tst_BGIP_Solvers -t > logs/tst_BGIP_Solvers.log
diff logs/tst_BGIP_Solvers.log logs/tst_BGIP_Solvers.reference.log
