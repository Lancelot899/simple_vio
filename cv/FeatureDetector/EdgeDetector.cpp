//
// Created by lancelot on 1/5/17.
//

#include "EdgeDetector.h"
#include "DataStructure/cv/Feature.h"

#define MIN_GRAD_HIST_CUT 0.5
namespace feature_detection {

using namespace std;
using namespace Eigen;

const static Vector2f directions[16] = {
    Vector2f(0,    1.0000),
    Vector2f(0.3827,    0.9239),
    Vector2f(0.1951,    0.9808),
    Vector2f(0.9239,    0.3827),
    Vector2f(0.7071,    0.7071),
    Vector2f(0.3827,   -0.9239),
    Vector2f(0.8315,    0.5556),
    Vector2f(0.8315,   -0.5556),
    Vector2f(0.5556,   -0.8315),
    Vector2f(0.9808,    0.1951),
    Vector2f(0.9239,   -0.3827),
    Vector2f(0.7071,   -0.7071),
    Vector2f(0.5556,    0.8315),
    Vector2f(0.9808,   -0.1951),
    Vector2f(1.0000,    0.0000),
    Vector2f(0.1951,   -0.9808)
};

int computeHistQuantil(int* hist, float below)
{
    int thresholdNum = hist[0]*below + 0.5f;
    for (int var = 0; var < 60; ++var) {
        thresholdNum -= hist[var];
        if(thresholdNum<0)  return var;
    }
    return 60;
}

EdgeDetector::EdgeDetector(
        const int img_width,
        const int img_height,
        const int cell_size,
        const int n_pyr_levels) :
    AbstractDetector(img_width, img_height, cell_size, n_pyr_levels),currentFrame(0)
{
    randomPattern = new unsigned char[img_width * img_height];
    std::srand(314152926);
    for (int i = 0; i < img_width*img_height; ++i) {
        randomPattern[i] = rand() & 0xFF;
    }

    gradHist = new int[100*(1+img_width/32)*(1+img_height/32)];
    threshold = new float[(img_width/32)*(img_height/32)+100];
    thresholdSmoothed = new float[(img_width/32)*(img_height/32)+100];
    edge.clear();
}

void EdgeDetector::detect(cvframePtr_t frame,
                          const AbstractDetector::ImgPyr_t &img_pyr,
                          const double detection_threshold,
                          AbstractDetector::features_t &fts)
{
    if(currentFrame != frame) makeHists(frame);

    float dw1 = 0.75, dw2 = dw1*dw1;

    int level = 0;
    float pixelTH0 = 0.0f, pixelTH1 = 0.0f, pixelTH2 = 0.0f;
    float thresholdFactor = 1.0f;
    for (int u = 0; u < frame->getWidth(level) ; ++u) {
        for (int v = 0; v < frame->getHeight(level); ++v) {
            if(frame->checkCellOccupy(u>>5,v>>5,level))
            {
                v += frame->getHeight(0)>>5;
                u += frame->getWidth(0)>>5;
                continue;
            }

            if(!level){
                pixelTH0 = thresholdSmoothed[(u>>5) + (v>>5) * thresholdStepU];
                pixelTH1 = pixelTH0*dw1;
                pixelTH2 = pixelTH1*dw2;
            }

            if (frame->getGradNorm(u,v,0) > pixelTH0*thresholdFactor)
            {
                cvFrame::grad_t  out;
                if(frame->getGrad(u,v,out,0))
                    fts.push_back(std::shared_ptr<Feature>(new Feature(frame,Eigen::Vector2d(u,v),out,0)));
            }
            else if(frame->getGradNorm(int(u*0.5+0.25f),int(v*0.5+0.25f),1)>pixelTH1*thresholdFactor) {
                int utmp = int(u*0.5+0.25f);
                int vtmp = int(v*0.5+0.25f);

                cvFrame::grad_t  out;
                if(frame->getGrad(utmp,vtmp,out,1))
                fts.push_back(std::shared_ptr<Feature>(new Feature(frame,Eigen::Vector2d(utmp,vtmp),out,1)));
            }
            else if (frame->getGradNorm(int(u*0.25+0.125f),int(v*0.25+0.125f),2)>pixelTH1*thresholdFactor) {
                int utmp1 = int(u*0.25+0.125f);
                int vtmp1 = int(v*0.25+0.125f);

                cvFrame::grad_t  out1;
                if(frame->getGrad(utmp1,vtmp1,out1,2))
                fts.push_back(std::shared_ptr<Feature>(new Feature(frame,Eigen::Vector2d(utmp1,vtmp1),out1,2)));
            }


        }
    }


}

void EdgeDetector::makeHists(cvframePtr_t frame)
{
    currentFrame = frame;
    int h = frame->getHeight(0);
    int w = frame->getWidth(0);

    //size of each cell : 32*32
    int w32 = w>>5;
    int h32 = h>>5;
    thresholdStepU = w32;
    thresholdStepV = h32;

    memset(threshold,100,sizeof(float)*w32*h32+100);
    memset(thresholdSmoothed,100,sizeof(float)*w32*h32+100);

    for (int x = 0; x < h32; ++x)
        for (int y = 0; y < w32; ++y)
        {
            int* hist0 = gradHist;
            memset(hist0,0,sizeof(int)*50); //devide into 49 parts

            //for each cell
            for (int i = 0; i < 32; ++i) for (int j = 0; j < 32; ++j)
            {
                int it = i+32*x ;
                int jt = j+32*y ;
                if(it>w-2 || jt>h-2 || it<1 || jt<1) continue;
                int g = sqrt(frame->getGradNorm(it,jt,0));
                if(g>48) g = 48;
                hist0[g+1]++;
                hist0[0]++;
            }

            threshold[x+y*w32] = computeHistQuantil(hist0,MIN_GRAD_HIST_CUT) + 3;
        }

    // SSD
    for(int y=0;y<h32;y++)
        for(int x=0;x<w32;x++)
        {
            float sum=0,num=0;
            if(x>0)
            {
                if(y>0) 	{num++; 	sum+=threshold[x-1+(y-1)*w32];}
                if(y<h32-1) {num++; 	sum+=threshold[x-1+(y+1)*w32];}
                num++; sum+=threshold[x-1+(y)*w32];
            }

            if(x<w32-1)
            {
                if(y>0) 	{num++; 	sum+=threshold[x+1+(y-1)*w32];}
                if(y<h32-1) {num++; 	sum+=threshold[x+1+(y+1)*w32];}
                num++; sum+=threshold[x+1+(y)*w32];
            }

            if(y>0) 	{num++; 	sum+=threshold[x+(y-1)*w32];}
            if(y<h32-1) {num++; 	sum+=threshold[x+(y+1)*w32];}
            num++; sum+=threshold[x+y*w32];

            thresholdSmoothed[x+y*w32] = (sum/num) * (sum/num);
        }
}

int EdgeDetector::sample(cvframePtr_t frame, float *rate)
{

}


class Pt {
public:
    float grad;
    Vector2d xy;

    Pt() {
        xy[0] = 0;
        xy[1] = 0;
        grad = 0.0;
    }

    Pt(float grad_, Vector2d xy_) : grad(grad_), xy(xy_) {}

    bool operator<(const Pt &m) const { return grad > m.grad; }
};

class Edgelete {
public:
    float grad;
    Vector2d xy;
    Vector2d dir;

    Edgelete() {
        xy[0] = 0;
        xy[1] = 0;
        grad = 0.0;
    }

    Edgelete(float grad_, Vector2d xy_, Vector2d dir_) : grad(grad_), xy(xy_), dir(dir_) {}

    bool operator<(const Edgelete &m) const { return grad > m.grad; }
};
}
