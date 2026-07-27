#include <common/transformation.h>
#include <iostream>
#include <common/types.h>
#include <common/camera.h>
#include <common/frame.h>
#include <common/feature_wrapper.h>
#include <common/point.h>
#include <common/frame.h>
#include <common/param_helper.h>
#include <common/point.h>
#include <estimator/global.h>
#include <estimator/imu_handler.h>
#include <estimator/schur_vins.h>
#include <estimator/frame_handler_base.h>
#include <estimator/frame_handler_stereo.h>
int main(int argc, char **argv)
{
    FAR::Transformation T;
    FAR::FrameBundlePtr frame_bundle;
    std::cout << "T:\n" << T << std::endl;
    return 0;
}