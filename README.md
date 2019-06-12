RobustPGO
======================================

## Dependencies

###[GTSAM](https://bitbucket.org/gtborg/gtsam)
(Note that a github version has also been released, but for the github version the flag `GTSAM_BUILD_WITH_MARCH_NATIVE` sometimes causes problems )

Clone GTSAM:   
```bash
cd
git clone git@bitbucket.org:gtborg/gtsam.git
```

Build
```bash
cd gtsam 
mkdir build
cd build
cmake .. -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON
sudo make install
```
**Note:** 
The reason why we need EXPMAP is for the correct calculation of Jacobians. 
Enabling this and the `#define SLOW_BUT_CORRECT_BETWEENFACTOR` in LaserLoopCLosure.h are both important. Otherwise the default are some identity approximations for rotations, which works for some cases but fails for cases with manual loop closures, or artifacts. Note that `sudo make check` will partially fail because some unittests assume the EXPMAP flags to be off. 

## Build 
```bash
git clone git@github.com:MIT-SPARK/RobustPGO.git
cd RobustPGO
mkdir build
cd build
cmake ..
sudo make install
```