#include "CameraIO.h"
#include <fstream>
#include <iomanip>

CameraIO::CameraIO(std::string imageFile, std::string cameraParamfile)
{
    assert(!imageFile.empty() && !cameraParamfile.empty());
    std::ifstream camParam_file(cameraParamfile.c_str());
    if(!camParam_file.good()) {
        std::cerr << "camParam_file file error!" << std::endl;
        exit(-1);
    }

    char camParamBuffer[1024];
    memset(camParamBuffer, 0, 1024);
    camParam_file.read(camParamBuffer, 1024);
    assert(camParamBuffer[0] != 0);

    /// T_BS
    char *p = camParamBuffer, *pend = camParamBuffer;
    p = strstr(camParamBuffer, "T_BS");
    if(p == NULL) {
        std::cerr << "on T_BS in camera param !\n";
        exit(-1);
    }
    p = strstr(p, "data: ["); p += strlen("data: [");

    Eigen::Matrix<double, 4, 4>     se3d;

    for(int i = 0; i < 4; i++){
        for (int j = 0; j < 4; ++j) {
            while( *p=='\0' | *p=='\n' | (*p != '.' && *p != '-' && *p!= 'e' && ( *p > '9' | *p < '0')) ) p++;
            if(i==3 && j==3)
                pend = strstr(p,"]");
            else
                pend = strstr(p,",");
            pend++;
            char value[pend-p+1]; memset(value,0,pend-p+1); memcpy(value,p,pend-p);
            se3d(i,j) = atof(value);
            p=pend;
        }
    }

    pose_t T_BS(se3d);
    ///rate
    p = strstr(camParamBuffer, "rate_hz: ");
    p += strlen("rate_hz: "); pend = p;
    while (*pend >= '0' && *pend <= '9') pend++;

    char ratevalue[pend-p+1]; memset(ratevalue,0,pend-p+1); memcpy(ratevalue,p,pend-p);
    int rate = atoi(ratevalue);

    ///resolution
    /// X
    p = strstr(camParamBuffer, "resolution: [");
    p += strlen("resolution: ["); pend = p;
    while (*pend >= '0' && *pend <= '9') pend++;
    char valueX[pend-p+1]; memset(valueX,0,pend-p+1);   memcpy(valueX,p,pend-p);
    double width = atof(valueX);
    /// Y
    pend++;
    while (*pend < '0' && *pend > '9') pend++;
    p=pend;
    pend = strstr(pend,"]");
    char valueY[pend-p+1];  memset(valueY,0,pend-p+1); memcpy(valueY,p,pend-p);
    double height = atof(valueY);

    ///camera_model
    p = strstr(camParamBuffer, "camera_model: ");
    p += strlen("camera_model: ");
    pend = p; pend = strstr(pend,"\n");
    char camera_model[pend-p+1]; memset(camera_model,0,pend-p+1); memcpy(camera_model,p,pend-p);
    std::string Camera_mode(camera_model);

    ///distortion_model
    p = strstr(camParamBuffer, "distortion_model: ");
    p += strlen("distortion_model: ");
    pend = p; pend = strstr(pend,"\n");
    char distortion_model[pend-p+1];  memset(distortion_model,0,pend-p+1); memcpy(distortion_model,p,pend-p);
    std::string Distortion_mode(distortion_model);

    ///intrinsics
    p = strstr(camParamBuffer, "intrinsics: [");
    p += strlen("intrinsics: ["); pend=p;

    double intrinsics[4];
    for (int i = 0; i < 4; ++i) {
        while( *p != '.' && *p != '-' && *p!= 'e' && ( *p > '9' | *p < '0') ) p++;
        if(i==3)
            pend = strstr(p , "]");
        else
            pend = strstr(p , ",");

        char valueIntri0[pend-p+1];
        memset(valueIntri0,0,pend-p+1);
        memcpy(valueIntri0,p,pend-p);
        p = pend;

        intrinsics[i] = atof(valueIntri0);
    }

    ///distortion_coefficients
    p = strstr(camParamBuffer, "distortion_coefficients: [");
    p += strlen("distortion_coefficients: ["); pend=p;
    double distortion_coefficients[4];
    for (int i = 0; i < 4; ++i) {
        while( *p != '.' && *p != '-' && *p!= 'e' && ( *p > '9' | *p < '0') ) p++;
        if(i==3)
            pend = strstr(p , "]");
        else
            pend = strstr(p , ",");

        char valueDistor0[pend-p+1];
        memset(valueDistor0,0,pend-p+1);
        memcpy(valueDistor0,p,pend-p);
        p = pend;

        distortion_coefficients[i] = atof(valueDistor0);
    }


    /// construct VIOCamera
    camParam = std::make_shared<VIOCamera>(width,height,intrinsics[0],intrinsics[1],intrinsics[2],intrinsics[3],
                                           distortion_coefficients[0],distortion_coefficients[1],distortion_coefficients[2],
                                           distortion_coefficients[3],0,T_BS,rate,Camera_mode,Distortion_mode);

}

