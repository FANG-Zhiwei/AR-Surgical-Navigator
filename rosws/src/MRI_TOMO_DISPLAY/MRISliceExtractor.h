//
// Created by WEQ on 27/11/2024.
//

#ifndef MRISLICEEXTRACTOR_H
#define MRISLICEEXTRACTOR_H

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <limits>

// OpenCV
#include <opencv2/opencv.hpp>

// ITK
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkAffineTransform.h>
#include <itkFlipImageFilter.h>


class MRISlicer {
public:

    typedef itk::Image<float, 3> ImageType;

    // init
    MRISlicer(const std::string& filename) {
        loadMRI(filename);
    }

    struct SliceParameters {
        double x;
        double y;
        double z;
        double yaw;   // z-axis rot，deg
        double pitch; // y-axis rot，deg
        double roll;  // x-axis rot，deg
        int width_grid;    // Grid width
        int height_grid;   // Grid height
        double scale; // MRI data scale
    };

    void setSliceParameters(const SliceParameters& params) {
        this->params = params;
    }

    cv::Mat getSlice() {
        if (!imageData) {
            std::cerr << "no MRI data.\n";
            return cv::Mat();
        }
        return get_MRI_slice(imageData, params);
    }


private:

    struct Point_grid {
        double x, y, z;
    };

    ImageType::Pointer imageData;
    SliceParameters params;

    inline double deg2rad(double degrees) const {
        return degrees * M_PI / 180.0;
    }

    std::vector<std::vector<double>> createRotationMatrix(double yaw, double pitch, double roll) const {
        double cy = cos(deg2rad(yaw));
        double sy = sin(deg2rad(yaw));
        double cp = cos(deg2rad(pitch));
        double sp = sin(deg2rad(pitch));
        double cr = cos(deg2rad(roll));
        double sr = sin(deg2rad(roll));

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        std::vector<std::vector<double>> R = {
                {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                {-sp,     cp * sr,               cp * cr}
        };

        return R;
    }

    void loadMRI(const std::string& filename) {
        typedef itk::ImageFileReader<ImageType> ReaderType;
        ReaderType::Pointer reader = ReaderType::New();
        reader->SetFileName(filename);
        try {
            reader->Update();
            imageData = reader->GetOutput();
            std::cout << "MRI data load." << filename << "\n";
        }
        catch(itk::ExceptionObject & error) {
            std::cerr << "MRI data loading failed." << error << std::endl;
            imageData = nullptr;
        }
    }

    /**
     * @brief input pose, get slice
     *
     * @param imageData itk volume
     * @param params position+rotation (deg)
     * @return cv::Mat 2D slice。
     */
    cv::Mat get_MRI_slice(ImageType::Pointer& imageData, const SliceParameters& params) const {

//        // debug
//        std::cout << params.x << " " << params.y << " " << params.z << " "
//                  << params.yaw << " " << params.pitch << " " << params.roll << "\n";

//        ImageType::SizeType size = imageData->GetLargestPossibleRegion().GetSize();

//        int width = static_cast<int>(size[0]);
//        int height = static_cast<int>(size[1]);
        int width = params.width_grid;
        int height = params.height_grid;
        double spacing = 1.0;

        std::vector<Point_grid> grid;
        std::vector<std::vector<double>> R = createRotationMatrix(params.yaw, params.pitch, params.roll);

        // set x, y at middle
        for (int i = -width/2; i <= (width/2 - 1); i++) {
            for (int j = -height/2; j <= (height/2 - 1); j++) {

                // generate a xz plane
                // 1. inverse z, as MRI data has inverse z
                // 2. set origin at center
                Point_grid point;
                point.x = i * spacing;
                point.y = 0;
                point.z = - j * spacing;

                // rotate + translate
                Point_grid rotatedPoint;
                rotatedPoint.x = R[0][0] * point.x + R[0][1] * point.y + R[0][2] * point.z + params.x / params.scale;
                rotatedPoint.y = R[1][0] * point.x + R[1][1] * point.y + R[1][2] * point.z + params.y / params.scale;
                rotatedPoint.z = R[2][0] * point.x + R[2][1] * point.y + R[2][2] * point.z - params.z / params.scale;

                grid.push_back(rotatedPoint);
            }
        }

//        // for debug
//        std::cout << "test -> grid size:" << grid.size();
//        int c = 0;
//        for (const auto& point : grid) {
//            std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")";
//            c++;
//            if (c == width){
//                std::cout << "\n \n";
//                c = 0;
//            }
//        }

        typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;
        InterpolatorType::Pointer interpolator = InterpolatorType::New();
        interpolator->SetInputImage(imageData);

        cv::Mat sampledImage = cv::Mat(height, width, CV_32FC1, cv::Scalar(0));
        float minValue = std::numeric_limits<float>::max();
        float maxValue = std::numeric_limits<float>::min();

        // sample
        ImageType::PointType point_itk;
        int idx = 0;
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j, ++idx) {
                point_itk[0] = grid[idx].x;  // x
                point_itk[1] = grid[idx].y;  // y
                point_itk[2] = grid[idx].z;  // z

                float value = 0;
                if (interpolator->IsInsideBuffer(point_itk)) {
                    value = interpolator->Evaluate(point_itk);
                }
                sampledImage.at<float>(j, i) = value;
                if (value > maxValue) maxValue = value;
                if (value < minValue) minValue = value;
            }
        }

        cv::Mat normalizedImage;
        float scale = 255.0f / (maxValue - minValue);
        float shift = -minValue * scale;
        sampledImage.convertTo(normalizedImage, CV_8UC1, scale, shift);

        // scale
        cv::resize(normalizedImage, normalizedImage, cv::Size(), params.scale, params.scale, cv::INTER_CUBIC);

        return normalizedImage;
    }
};

#endif // MRISLICEEXTRACTOR_H
