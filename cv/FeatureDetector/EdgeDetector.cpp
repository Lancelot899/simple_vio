//
// Created by lancelot on 1/5/17.
//
#include <alloca.h>

#include "EdgeDetector.h"
#include "DataStructure/cv/Feature.h"

#define MIN_GRAD_HIST_CUT 0.5
namespace feature_detection {

using namespace std;
using namespace Eigen;

const static Vector2d directions[16] = {
    Vector2d(0.9808,    0.1951),
    Vector2d(0.9239,   -0.3827),
    Vector2d(0.7071,   -0.7071),
    Vector2d(0.5556,    0.8315),
    Vector2d(0.9808,   -0.1951),
    Vector2d(1.0000,    0.0000),
    Vector2d(0.1951,   -0.9808),
    Vector2d(0,         1.0000),
    Vector2d(0.3827,    0.9239),
    Vector2d(0.1951,    0.9808),
    Vector2d(0.9239,    0.3827),
    Vector2d(0.7071,    0.7071),
    Vector2d(0.3827,   -0.9239),
    Vector2d(0.8315,    0.5556),
    Vector2d(0.8315,   -0.5556),
    Vector2d(0.5556,   -0.8315)

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
    std::allocator<char> alloc;
    randomPattern = (unsigned char*)alloc.allocate(img_width*img_height);
    std::srand(314152926);
    for (int i = 0; i < img_width*img_height; ++i) {
        randomPattern[i] = rand() & 0xFF;
    }

    gradHist = (int*)alloc.allocate(100*(1+img_width/32)*(1+img_height/32)*sizeof(int));
    threshold = (float*)alloc.allocate( ((img_width/32)*(img_height/32)+100)*sizeof(float) );
    thresholdSmoothed = (double*)alloc.allocate( ((img_width/32)*(img_height/32)+100)*sizeof(double) );
    edge.clear();
}

void EdgeDetector::makeHists(cvframePtr_t frame)
{
    currentFrame = frame;
    int h = frame->getHeight(0);
    int w = frame->getWidth(0);

    ///< size of cell grid : 32*32
    int w32 = w>>5;
    int h32 = h>>5;
    thresholdStepU = w32;
    thresholdStepV = h32;

    memset(threshold,100,sizeof(float)*w32*h32+100);
    memset(thresholdSmoothed,100,sizeof(float)*w32*h32+100);

    for (int x = 0; x < w32; ++x) for (int y = 0; y < h32; ++y)
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
            int isInf = isinf(thresholdSmoothed[x+y*w32]);
//            if(isInf==1 || isInf==-1) {
//                exit(-1)  ;
//            }
        }
}
void EdgeDetector::detect(cvframePtr_t frame,
                          const ImgPyr_t &img_pyr,
                          const double detection_threshold,
                          features_t &fts)
{
    if(currentFrame != frame) makeHists(frame);

    float thresholdFactor = 1.0f;
    float dw1 = 0.75f, dw2 = dw1*dw1;

    fts.clear();
    int w  = frame->getWidth(0);
    int h  = frame->getHeight(0);

    int n3=0, n2=0, n4=0;
    int pot = 5;
    int bestU0 = -1, bestU1 = -1,  bestU2 = -1, bestV0 = -1, bestV1 = -1,  bestV2 = -1;

    //    for (int cellV = 0; cellV < 32; ++cellV) {
    //        for (int cellU = 0; cellU < 32; ++cellU) {
    //            if(frame->checkCellOccupy(cellU,cellV,0))     continue;

    for(int y4=0;y4<h;y4+=(4*pot)) for(int x4=0;x4<w;x4+=(4*pot))
    {
        int my3 = std::min((4*pot), h-y4);
        int mx3 = std::min((4*pot), w-x4);
        int bestIdx4=-1; float bestVal4=0;
        Eigen::Vector2d dir4 = directions[randomPattern[n2] & 0xF];
        for(int y3=0;y3<my3;y3+=(2*pot)) for(int x3=0;x3<mx3;x3+=(2*pot))
        {
            int x34 = x3+x4;
            int y34 = y3+y4;
            int my2 = std::min((2*pot), h-y34);
            int mx2 = std::min((2*pot), w-x34);
            int bestIdx3=-1; float bestVal3=0;
            Eigen::Vector2d dir3 = directions[randomPattern[n2] & 0xF];
            for(int y2=0;y2<my2;y2+=pot) for(int x2=0;x2<mx2;x2+=pot)
            {
                int x234 = x2+x34;
                int y234 = y2+y34;
                int my1 = std::min(pot, h-y234);
                int mx1 = std::min(pot, w-x234);
                int bestIdx2=-1; float bestVal2=0;
                Eigen::Vector2d dir2 = directions[randomPattern[n2] & 0xF];
                for(int y1=0;y1<my1;y1+=1) for(int x1=0;x1<mx1;x1+=1)
                {
                    int idx = x1+x234 + w*(y1+y234);
                    int xf = x1+x234;
                    int yf = y1+y234;

                    if(xf<4 || xf>=w-5 || yf<4 || yf>h-4) continue;

                    double pixelTH0 = thresholdSmoothed[(xf>>5) + (yf>>5) * thresholdStepU];
                    double pixelTH1 = pixelTH0*dw1;
                    double pixelTH2 = pixelTH1*dw2;

                    float ag0 = frame->getGradNorm(xf,yf,0);

                    if(ag0 > pixelTH0*thresholdFactor)
                    {
                        cvFrame::grad_t  ag0d;    frame->getGrad(xf,yf,ag0d,0);
                        float dirNorm = fabs((float)(ag0d.dot(dir2)));

                        if(dirNorm > bestVal2)
                        { bestVal2 = dirNorm; bestIdx2 = idx, bestU0 = xf, bestV0 = yf; bestIdx3 = -2; bestIdx4 = -2;}
                    }
                    if(bestIdx3==-2) continue;

                    float ag1 = frame->getGradNorm((int)(xf*0.5f+0.25f),(int)(yf*0.5f+0.25f),1);
                    if(ag1 > pixelTH1*thresholdFactor)
                    {
                        cvFrame::grad_t  ag0d;    frame->getGrad((int)(xf*0.5f+0.25f),(int)(yf*0.5f+0.25f),ag0d,1);
                        float dirNorm = fabs((float)(ag0d.dot(dir3)));

                        if(dirNorm > bestVal3)
                        { bestVal3 = dirNorm; bestIdx3 = idx, bestU1 = (int)(xf*0.5f+0.25f), bestV1 = (int)(yf*0.5f+0.25f); bestIdx4 = -2;}
                    }
                    if(bestIdx4==-2) continue;

                    float ag2 = frame->getGradNorm((int)(xf*0.25f+0.125f), (int)(yf*0.25f+0.125f),2);
                    if(ag2 > pixelTH2*thresholdFactor)
                    {
                        cvFrame::grad_t  ag0d;    frame->getGrad((int)(xf*0.25f+0.125f),(int)(yf*0.25f+0.125f),ag0d,2);
                        float dirNorm = fabs((float)(ag0d.dot(dir4)));

                        if(dirNorm > bestVal4)
                        { bestVal4 = dirNorm; bestIdx4 = idx, bestU2 = (int)(xf*0.25f+0.125),bestV2 = (int)(yf*0.25f+0.125f); }
                    }
                }

                if(bestIdx2>0)
                {
                    cvFrame::grad_t  out;
                    if(frame->getGrad(bestU0,bestV0,out,0))
                        fts.push_back(std::shared_ptr<Feature>(new Feature(frame,Eigen::Vector2d(bestU0,bestV0),out,0)));
                    bestVal3 = 1e10;
                    n2++;
                }
            }

            if(bestIdx3>0)
            {
                cvFrame::grad_t  out;
                if(frame->getGrad(bestU1,bestV1,out,1))
                    fts.push_back(std::shared_ptr<Feature>(new Feature(frame,Eigen::Vector2d(bestU1,bestV1),out,1)));
                bestVal4 = 1e10;
                n3++;
            }
        }

        if(bestIdx4>0)
        {
            cvFrame::grad_t  out;
            if(frame->getGrad(bestU2,bestV2,out,2))
                fts.push_back(std::shared_ptr<Feature>(new Feature(frame,Eigen::Vector2d(bestU2,bestV2),out,2)));
            n4++;
        }
    }

    //        }
    //    }
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
