# ssba2016-scale-estimation

# requires opencv3 and ceres solver
mkdir build
cd build
cmake ..
make -j4
./main


The monocular scale estimator works in theory, but in pratice the observability is rather low. 
