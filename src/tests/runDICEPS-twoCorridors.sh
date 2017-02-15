problem="twoCorridors_2.toi-dpomdp"
find ../../problems -name "${problem}*" -exec ln -sf {} \;
../solvers/DICEPS -d -t -h 1 --CE-restarts=2 --iterations=20 --samples=500 --updateSamples=10 --toi ${problem}
rm ${problem}*
