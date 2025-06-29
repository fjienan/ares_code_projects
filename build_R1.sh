#!/bin/bash
colcon build --symlink-install --cmake-args="-DR1=ON -DCMAKE_BUILD_TYPE=Release" --parallel-workers $(nproc)
source install/setup.bash
