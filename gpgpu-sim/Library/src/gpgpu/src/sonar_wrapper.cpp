#include <gpgpu/sonar_test.cuh>

#include <iostream>

#include <stdio.h>
#include <complex>
#include <valarray>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;
typedef std::valarray<CArray> CArray2D;

int main() {

    int height = 50;
    int width = 10;
    cv::Mat depth_image = cv::Mat::ones(height, width, CV_32FC1);
    cv::Mat normal_image = cv::Mat::ones(height, width, CV_32FC3);
    cv::Mat rand_image = cv::Mat::ones(height, width, CV_32FC2);
    double _hFOV = 0.2;
    double _vFOV = 0.2;
    double _soundSpeed = 1500;
    double _maxDistance = 2.0;
    double _sourceLevel = 150.0;
    int _nBeams = height; int _nRays = width;
    int _raySkips = 1;
    double _vPixelSize = _vFOV / height;
    double _hPixelSize =_hFOV / width;
    double _beam_azimuthAngleWidth = _hPixelSize;
    double _beam_elevationAngleWidth = _vFOV/180*3.141592;
    double _ray_azimuthAngleWidth = _hPixelSize;
    double _ray_elevationAngleWidth = _vPixelSize*(_raySkips+1);
    double _sonarFreq = 900e3;
    double _bandwidth = 29.5e4;
    float max_T = _maxDistance*2.0/_soundSpeed;
    float _delta_f = 1.0/max_T;
    int _nFreq = ceil(_bandwidth/_delta_f);
    cv::Mat reflectivity_image = cv::Mat::ones(height, width, CV_32FC1);
    double _attenuation = 0.1;
    float *window;
    window = new float[_nFreq];
    float **beamCorrector;
    beamCorrector = new float*[_nBeams];
    for (int i = 0; i < _nBeams; i++)
        beamCorrector[i] = new float[_nBeams];
    float beamCorrectorSum = 10;
    bool debugFlag = false;

    CArray2D P_Beams = NpsGazeboSonar::sonar_calculation_wrapper
    (                                depth_image,
                                     normal_image,
                                     rand_image,
                                      _hPixelSize,
                                      _vPixelSize,
                                      _hFOV,
                                      _vFOV,
                                      _beam_azimuthAngleWidth,
                                      _beam_elevationAngleWidth,
                                      _ray_azimuthAngleWidth,
                                      _ray_elevationAngleWidth,
                                      _soundSpeed,
                                      _maxDistance,
                                      _sourceLevel,
                                      _nBeams,  _nRays,
                                      _raySkips,
                                      _sonarFreq,
                                      _bandwidth,
                                     _nFreq,
                                     reflectivity_image,
                                     _attenuation,
                                     window,
                                     beamCorrector,
                                     beamCorrectorSum,
                                     debugFlag


    );
    std::cout << "Hello World!";
    return 0;
}
