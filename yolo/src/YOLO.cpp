#include "YOLO.h"
#define OPENCV
#include "../darknet/darknet.h"
#define NUM_CLASSES 7
image **alphabet;
char *coco_names[] = {"Construction Barrel", "One Way Left", "One Way Right", "Road Closed", "Stop Sign", "Pedestrian", "No Turns", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow"};

YOLO::YOLO()
{
    mConfigPath = "";
    mWeightsPath = "";
    mNamesPath = "";
    mNetwork = NULL;
    mIsRunning = false;
    detected_ipl = NULL;
}
YOLO::~YOLO()
{
    
}
bool YOLO::Initialize()
{
    //can't initialize if we don't have the files.
    if( mConfigPath.size() == 0 || mWeightsPath.size() == 0)
        return false;

    //create non-const versions of our paths that we can pass into darknet
    char* mutable_config_path = new char[mConfigPath.size()+1];
    memcpy(mutable_config_path, mConfigPath.c_str(), mConfigPath.size());
    memset(&mutable_config_path[mConfigPath.size()], 0, 1);

    char* mutable_weights_path = new char[mWeightsPath.size()+1];
    memcpy(mutable_weights_path, mWeightsPath.c_str(), mWeightsPath.size());
    memset(&mutable_weights_path[mWeightsPath.size()], 0, 1);

    delete mNetwork;
    mNetwork = load_network(mutable_config_path, mutable_weights_path, 0);

    //I'm not entirely sure what this means. Usually batch size is a hyper param 
    //for training, but I see this in the yolo.c example sooo...
    set_batch_network(mNetwork, 1);

    delete[] mutable_config_path;
    delete[] mutable_weights_path;

    alphabet = load_alphabet();

    return mNetwork != NULL;
}
image ipl_to_image(IplImage* src)
{
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    image im = make_image(w, h, c);
    unsigned char *data = (unsigned char *)src->imageData;
    int step = src->widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return im;
}
IplImage *image_to_ipl(image im)
{
    int x,y,c;
    IplImage *disp = cvCreateImage(cvSize(im.w,im.h), IPL_DEPTH_8U, im.c);
    int step = disp->widthStep;
    for(y = 0; y < im.h; ++y){
        for(x = 0; x < im.w; ++x){
            for(c= 0; c < im.c; ++c){
                float val = im.data[c*im.h*im.w + y*im.w + x];
                disp->imageData[y*step + x*im.c + c] = (unsigned char)(val*255);
            }
        }
    }
    return disp;
}
std::vector<Detection_t> YOLO::Detect(IplImage* ipl_img)
{
    std::vector<Detection_t> ret;
    if(!mNetwork)
        return ret;

    mMutex.lock();
    mIsRunning = true;
    mMutex.unlock();


    image im = ipl_to_image(ipl_img);

    image sized = letterbox_image(im, mNetwork->w, mNetwork->h);

    //rgbgr_image(sized);

    try{
        network_predict(mNetwork, sized.data);
    } catch (...)
    {
        printf("exception in detect\n");
    }

    int num_detections = 0;
    detection* dets = get_network_boxes(mNetwork, 1, 1, mDetectionThreshold, 0, NULL, 0, &num_detections);
    //Non max suppression suppress duplicate detections.
    layer output_layer = mNetwork->layers[mNetwork->n-1];
    int size = output_layer.side * output_layer.side * output_layer.n;
    do_nms_sort(dets, num_detections, NUM_CLASSES, mNonMaxSuppressThreshold);
    //printf("classes: %d\n", output_layer.classes);

    //printf("num detections:%i\n", num_detections);
    Detection_t temp;
    for(int i = 0; i < num_detections; i++)
    {
        detection& det = dets[i];
        float max_prob = 0;
        int classification = -1;
        for(int j = 0; j < det.classes; j++)
        {
            if(det.prob[j] > max_prob)
            {
                classification = j;
                max_prob = det.prob[j];
            }
        }
        if(max_prob < mDetectionThreshold)
            continue;
        temp.class_id = classification;
        temp.confidence = max_prob;
        temp.x = det.bbox.x;
        temp.y = det.bbox.y;
        temp.w = det.bbox.w;
        temp.h = det.bbox.h;
        //printf("probability: %f\t\tclass: %d\t\tx: %f\ty: %f\\tw: %f\th: %fn", max_prob, classification, det.bbox.x, det.bbox.y, det.bbox.w, det.bbox.h);
        ret.push_back(temp);
    }
    if(mDrawDetections)
    {
        draw_detections(sized, dets, num_detections, mDetectionThreshold, coco_names, alphabet, NUM_CLASSES);
        mMutex.lock();
        if(detected_ipl)
            cvReleaseImage(&detected_ipl);
        detected_ipl = image_to_ipl(sized);
        mMutex.unlock();
    } 
    free_detections(dets, num_detections);
    free_image(im);
    free_image(sized);
    mMutex.lock();
    mIsRunning = false;
    mMutex.unlock();
    return ret;
}
bool YOLO::IsRunning()
{
    bool ret;
    mMutex.lock();
    ret = mIsRunning;
    mMutex.unlock();
    return ret;
}
