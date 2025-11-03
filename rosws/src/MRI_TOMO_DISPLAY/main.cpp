// main.cpp

// Step1: include
#include "MRISliceExtractor.h"


int main(int argc, char* argv[])
{
    // Step2: initialize
    MRISlicer slicer("../IXI002-Guys-0828-T1.nii.gz");

    // in main
    while (true)
    {
        // step3: set param here
        MRISlicer::SliceParameters params;
        params.x = 11.11;  // pose
        params.y = 22.22;  // pose
        params.z = 33.33;  // pose
        params.yaw = 44.44;  // pose   rot(-180~180)
        params.pitch = -55.55;  // pose
        params.roll = 66.66;  // pose
        params.width_grid = params.height_grid = 240;  // sample grid, suggest 240
        params.scale = 1.0;  // set scale here

        slicer.setSliceParameters(params);

        // step4: run
        cv::Mat sliceImage = slicer.getSlice();

        if (sliceImage.empty())
        {
            std::cerr << "Failed to extract slice." << std::endl;
            return EXIT_FAILURE;
        }

        // test: show image
        cv::imshow("test", sliceImage);
        char key = static_cast<char>(cv::waitKey(30));
        if (key == 27) // Esc key
        {
            break;
        }
    }

    return EXIT_SUCCESS;
}