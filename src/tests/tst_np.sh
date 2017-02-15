#!/bin/bash

./tst_np -t > logs/tst_np.log
diff logs/tst_np.log logs/tst_np.reference.log
