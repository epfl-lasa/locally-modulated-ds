# locally-modulated-ds

Definition of Locally Modulated Dynamical Systems, including an implementation 
using Gaussian Process Regression.

Publication: 

    Incremental Motion Learning with Locally Modulated Dynamical Systems
    Kronander, Khansari and Billard
    Robotics and Autonomous Systems, 2015


## A Minimal example

```c++
#include "locally_modulated_ds/linear_velocity_fields.h
#include "locally_modulated_ds/gp_modulated_ds.h

int D = 3;
Mat A = -0.4*Mat::Identity(D,D)
Vec target(D);

LinearVelocityField straightline_field(target,A,0.3);
// to evaluate the system:
Vec currpos; // set its value from somewhere
Vec vel = straightline_field(currpos);

GaussianProcessModulatedDS gpmds_(straightline_field); // provide the original dynamics to the constructor
Eigen::Matrix<float, 3,1> training_pos, training_vel;

// set the hyperparams for GP
gpmds_.get_gpr()->SetHyperParams(0.1, 1.0, 0.0001);

// fill with training data, usually get from demonstrations
// then add them to gpmds
gpmds_.AddData(training_pos, training_vel) // add a single training point,

int nTrain = 100;
Eigen::Matrix<float, 3,nTrain> training_pos_batch, training_vel_batch;
// fill with training data, usually get from demonstrations
// then add to gpmds
gpmds_.AddDataBatch(training_pos_batch, training_vel_batch) // add batch training point

// you can now query the system for velocities
auto output_vel = gpmds_->GetOutput(test_pos);

```

## Documentation
You can get some basic source code documentation by running doxygen.

```
sudo apt-get install doxygen
roscd locally_modulated_ds
doxygen Doxyfile
```


[![Build Status](https://magnum.travis-ci.com/epfl-lasa/locally-modulated-ds.svg?token=BqUQb763tsVV4QyzLgBy&branch=master)](https://magnum.travis-ci.com/epfl-lasa/locally-modulated-ds)



