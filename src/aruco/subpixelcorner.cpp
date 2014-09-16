#include <aruco/subpixelcorner.h>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

namespace aruco{

SubPixelCorner::SubPixelCorner()
{
    _winSize=15;
    _apertureSize=3;
    _term.maxCount=10;
    _term.epsilon=0.1;
    _term.type=CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
     enable=true;
}

void SubPixelCorner::checkTerm()
{
    switch( _term.type)
    {
    case CV_TERMCRIT_ITER:
        _term.epsilon = 0.f;
        _term.maxCount;
        break;
    case CV_TERMCRIT_EPS:
        _term.maxCount=_term.COUNT;
        break;
    case CV_TERMCRIT_ITER | CV_TERMCRIT_EPS:
        break;
    default:
        _term.maxCount=_term.COUNT;
        _term.epsilon=0.1;
        _term.type=CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
    break;
    }

    eps = std::max( _term.epsilon ,0.0);
    eps=eps*eps;

    _max_iters= std::max( _term.maxCount, 1 );
    int max1=TermCriteria::MAX_ITER;
    _max_iters= std::min(_max_iters,max1);

}

double SubPixelCorner::pointDist(cv::Point2f estimate_corner,cv::Point2f curr_corner)
{
    double dist=((curr_corner.x-estimate_corner.x)*(curr_corner.x-estimate_corner.x))+
                        ((curr_corner.y-estimate_corner.y)*(curr_corner.y-estimate_corner.y));
    return dist;
}


void SubPixelCorner::generateMask()
{

     double coeff = 1. / (_winSize*_winSize);
     float * maskX=(float *)calloc(1,(_winSize*sizeof(float)));
     float * maskY=(float *)calloc(1,(_winSize*sizeof(float)));
     mask.create (_winSize,_winSize,CV_32FC(1));
    /* calculate mask */
     int k=0;
    for( int i = -_winSize/2, k = 0; i <= _winSize/2; i++, k++ )
    {
        maskX[k] = (float)exp( -i * i * coeff );

    }

        maskY = maskX;

    for( int i = 0; i < _winSize; i++ )
    {
        float * mask_ptr=mask.ptr <float>(i);
        for( int j = 0; j < _winSize; j++ )
        {
            mask_ptr[j] = maskX[j] * maskY[i];
        }
    }

}

void SubPixelCorner::RefineCorner(cv::Mat image,std::vector <cv::Point2f> &corners)
{

    if(enable==false)
        return;
    checkTerm();

    generateMask ();
    //loop over all the corner points
    for(int k=0;k<corners.size ();k++)
    {
        cv::Point2f curr_corner;
        //initial estimate
        cv::Point2f estimate_corner=corners[k];

// cerr << 'SSS" << corners[k].x <<":" << corners[k].y << endl;

        if(estimate_corner.x<0 || estimate_corner.y<0 || estimate_corner.y >image.rows || estimate_corner.y > image.cols)
            continue;
        int iter=0;
        double dist=TermCriteria::EPS;
        //loop till termination criteria is met
        do
       {
        iter=iter+1;
        curr_corner=estimate_corner;

        /*
Point cx;
cx.x=floor(curr_corner.x);
cx.y=floor(curr_corner.y);
double dx=curr_corner.x-cx.x;
double dy=curr_corner.y-cx.y;
float vIx[2];
float vIy[2];

vIx[0] = dx;
vIx[1] = 1 - dx;
vIy[0] = dy;
vIy[1] = 1 - dy;

int x1=std::max((int)(cx.x-_winSize-_apertureSize/2),0);
int y1=std::max((int)(cx.y-_winSize-_apertureSize/2),0);

xmin = x1<0?0:x1;
xmax = x1+_winSize<image.cols?x1+_winSize:image.cols-1;
ymin = y1<0?0:y1;
ymax = y1+_winSize<image.rows?y1+_winSize:image.rows-1;

Rect roi=Rect(xmin,ymin,xmax-xmin,ymax-ymin);
*/

        Mat local;
        cv::getRectSubPix (image,Size(_winSize+2*(_apertureSize/2),_winSize+2*(_apertureSize/2)),curr_corner,local);



        cv::Mat Dx,Dy;
        //extracing image ROI about the corner point
        //Mat local=image(roi);
        //computing the gradients over the neighborhood about corner point
        cv::Sobel (local,Dx,CV_32FC(1),1,0,_apertureSize,1,0);
        cv::Sobel (local,Dy,CV_32FC(1),0,1,_apertureSize,1,0);

        //parameters requried for estimations
        double A=0,B=0,C=0,D=0,E=0,F=0;
        int lx=0,ly=0;
        for(int i=_apertureSize/2;i<=_winSize;i++)
        {

            float *dx_ptr=Dx.ptr <float>(i);
            float *dy_ptr=Dy.ptr <float>(i);
            ly=i-_winSize/2-_apertureSize/2;

            float * mask_ptr=mask.ptr <float>(ly+_winSize/2);

            for(int j=_apertureSize/2;j<=_winSize;j++)
            {

                lx=j-_winSize/2-_apertureSize/2;
                //cerr << lx+_winSize/2 << ":" ;
                double val=mask_ptr[lx+_winSize/2];
                double dxx=dx_ptr[j]*dx_ptr[j]*val;
                double dyy=dy_ptr[j]*dy_ptr[j]*val;
                double dxy=dx_ptr[j]*dy_ptr[j]*val;

                A=A+dxx;
                B=B+dxy;
                E=E+dyy;
                C=C+dxx*lx+dxy*ly;
                F=F+dxy*lx+dyy*ly;

            }
        }

        //computing denominator
        double det=(A*E-B*B);
        if(fabs( det ) > DBL_EPSILON*DBL_EPSILON)
        {
         det=1.0/det;
        //translating back to original corner and adding new estimates
        estimate_corner.x=curr_corner.x+((C*E)-(B*F))*det;
        estimate_corner.y=curr_corner.y+((A*F)-(C*D))*det;
        }
        else
        {
            estimate_corner.x=curr_corner.x;
            estimate_corner.y=curr_corner.y;
        }

        dist=pointDist(estimate_corner,curr_corner);


        }while(iter<_max_iters && dist>eps);

        //double dist=pointDist(corners[k],estimate_corner);
        if(fabs(corners[k].x-estimate_corner.x) > _winSize || fabs(corners[k].y-estimate_corner.y)>_winSize)
        {
            estimate_corner.x=corners[k].x;
            estimate_corner.y=corners[k].y;
        }
        corners[k].x=estimate_corner.x;
        corners[k].y=estimate_corner.y;
        //cerr << "EEE" << corners[k].x <<":" << corners[k].y << endl;

    }

}

}
