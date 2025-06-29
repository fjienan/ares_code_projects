colcon build --symlink-install --cmake-args="-DR2=ON -DCMAKE_BUILD_TYPE=Release" --parallel-workers $(nproc) 
source install/setup.bash
