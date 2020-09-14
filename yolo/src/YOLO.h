#pragma once
#include <string>
#include <vector>
#include <mutex>
#include <opencv2/core/core_c.h>

struct Detection_t
{
    float x;
    float y;
    float w;
    float h;
    int class_id;
    float confidence;
};

class network;

class YOLO
{
public:
    YOLO();
    ~YOLO();

    void SetConfigurationPath(const std::string& path) {mConfigPath = path;}
    void SetWeightsPath(const std::string& path) {mWeightsPath = path;}
    void SetNamesPath(const std::string& path) {mNamesPath = path;}
    void SetDetectionThreshold(const float& set) {mDetectionThreshold = set;}
    void SetNMSThreshold(const float& set) {mNonMaxSuppressThreshold = set;}
    void DrawDetections(const bool& set) {mDrawDetections = set;}
    IplImage* GetDetectedImage(){return detected_ipl;}

    //This function is blocking and will take considerable time (relatively speaking)
    bool Initialize();

    std::vector<Detection_t> Detect(IplImage* img);
    bool IsRunning();

    void Lock(){mMutex.lock();}
    void Unlock(){mMutex.unlock();}

    std::string mLastError;

private:
    std::string mConfigPath;
    std::string mWeightsPath;
    std::string mNamesPath;
    float mDetectionThreshold;
    float mNonMaxSuppressThreshold;

    std::mutex mMutex;
    bool mIsRunning;
    bool mDrawDetections;
    IplImage* detected_ipl;
    
    network* mNetwork;
};