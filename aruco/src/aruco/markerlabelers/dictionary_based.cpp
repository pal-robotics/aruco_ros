/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

#include "dictionary_based.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <bitset>
#include <cmath>
#include <opencv2/highgui.hpp>//TOREMOVE

namespace aruco
{
    void DictionaryBased::setParams(const Dictionary& dic, float max_correction_rate)
    {

        _nsubdivisions=sqrt(dic.nbits())+2;
        nbits_dict.clear();
        dicttypename=dic.getName();
        if (dic.getType()==Dictionary::ALL_DICTS){
            for(auto &dt:Dictionary::getDicTypes())
                if (dt!="ALL_DICTS" && dt!="CUSTOM")
                    vdic.push_back( Dictionary ::loadPredefined(dt));
        }
        else         vdic.push_back(  dic);

        //
        for(auto &dic:vdic)
            nbits_dict[dic.nbits()].push_back(&dic);

        _max_correction_rate = std::max(0.f, std::min(1.0f, max_correction_rate));
    }

    std::string DictionaryBased::getName() const
    {
        return dicttypename;
    }

    void DictionaryBased::toMat(uint64_t code, int nbits_sq, cv::Mat& out)
    {
        out.create(nbits_sq, nbits_sq, CV_8UC1);
        std::bitset<64> bs(code);
        int curbit = 0;
        for (int r = 0; r < nbits_sq; r++)
        {
            uchar* pr = out.ptr<uchar>(r);
            for (int c = 0; c < nbits_sq; c++)
                pr[c] = bs[curbit];
        }
    }

    int hamm_distance(uint64_t a, uint64_t b)
    {
        return static_cast<int>(std::bitset<64>(a ^ b).count());
    }

    bool DictionaryBased::detect(const cv::Mat& in, int& marker_id, int& nRotations, std::string &additionalInfo)
    {
        assert(in.rows == in.cols);

        cv::Mat grey;
        if (in.type() == CV_8UC1) grey = in;
        else cv::cvtColor(in, grey, CV_BGR2GRAY);
        // threshold image
        cv::threshold(grey, grey, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        std::map<uint32_t,std::vector<uint64_t> > nbits_ids;
        //for each
        for(auto &bitsids:nbits_dict){
            int nbits=bitsids.first;
            std::vector<uint64_t> ids;
            getInnerCode(grey,nbits,ids);
            if (ids.size()>0){
                if (ids[0]!=0){
                    nbits_ids[nbits]=ids;
                }
            }
        }
        //how many  are there?

        if ( nbits_ids.size()==0)return false;
        //check if any dictionary recognizes it
        for(auto nbits:nbits_ids){
            const auto &ids=nbits.second;
            //check in every dictionary
            for(auto &dic:nbits_dict[nbits.first]){
                //try a perfecf match
                for(int rot=0;rot<4;rot++)
                    if ( dic->is( ids[rot])){
                        //  std::cout<<"MATCH:"<<dic->getName()<<" "<<ids[rot]<<std::endl;
                        nRotations = rot;  // how many rotations are and its id
                        marker_id = dic->at (ids[rot]);
                        additionalInfo=dic->getName();
                        return true;
                    }

                //try with some error/correction if allowed
                if (_max_correction_rate > 0)
                {  // find distance to map elements
                    int _maxCorrectionAllowed = static_cast<int>( static_cast<float>(dic->tau()) * _max_correction_rate);
                    for (auto ci : dic->getMapCode())
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            if (hamm_distance(ci.first, ids[i]) < _maxCorrectionAllowed)
                            {
                                marker_id = ci.second;
                                nRotations = i;
                                additionalInfo=dic->getName();
                                return true;
                            }
                        }
                    }
                }

            }


        }


        return false;
    }


    bool DictionaryBased::getInnerCode(const cv::Mat& thres_img, int total_nbits, std::vector<uint64_t>& ids)
    {
        int bits_noborder = static_cast<int>(std::sqrt(total_nbits));
        int bits_withborder = bits_noborder + 2;
        // Markers  are divided in (bits_a+2)x(bits_a+2) regions, of which the inner bits_axbits_a belongs to marker
        // info
        // the external border shoould be entirely black
        cv::Mat nonZeros(bits_withborder,bits_withborder,CV_32SC1);
        cv::Mat nValues(bits_withborder,bits_withborder,CV_32SC1);
        nonZeros.setTo(cv::Scalar::all(0));
        nValues.setTo(cv::Scalar::all(0));
        for (int y = 0; y <  thres_img.rows; y++)
        {
            const uchar *ptr=thres_img.ptr<uchar>(y);
            int my=   float(bits_withborder)*float(y)/ float(thres_img.rows);
            for (int x = 0; x < thres_img.cols; x++)
            {
                int mx=   float(bits_withborder)*float(x)/ float(thres_img.cols);
                if( ptr[x]>125)
                    nonZeros.at<int>(my,mx)++;
                nValues.at<int>(my,mx)++;
            }
        }
        cv::Mat binaryCode(bits_withborder,bits_withborder,CV_8UC1);
        //now, make the theshold
        for(int y=0;y<bits_withborder;y++)
            for(int x=0;x<bits_withborder;x++){
                 if(nonZeros.at<int>(y,x)>nValues.at<int>(y,x)/2)
                    binaryCode.at<uchar>(y,x)=1;
                else
                    binaryCode.at<uchar>(y,x)=0;
            }

        //check if border is completely black
        for (int y = 0; y < bits_withborder; y++)
       {
           int inc = bits_withborder - 1;
           if (y == 0 || y == bits_withborder - 1)
               inc = 1;  // for first and last row, check the whole border
           for (int x = 0; x < bits_withborder; x += inc)
             if (binaryCode.at<uchar>(y,x)!=0 ) return false;
        }

        //take the inner code

        cv::Mat _bits(bits_noborder,bits_noborder,CV_8UC1);
        for(int y=0;y<bits_noborder;y++)
            for(int x=0;x<bits_noborder;x++)
                _bits.at<uchar>(y,x)=binaryCode.at<uchar>(y+1,x+1);

        // now, get the 64bits ids

        int nr = 0;
        do
        {
            ids.push_back(touulong(_bits));
            _bits = rotate(_bits);
            nr++;
        } while (nr < 4);
        return true;
    }


//    bool DictionaryBased::getInnerCode(const cv::Mat& thres_img, int total_nbits, std::vector<uint64_t>& ids)
//    {
//        auto toInt=[](float v){return int(v+0.5);};
//        int bits_a = static_cast<int>(std::sqrt(total_nbits));
//        int bits_a2 = bits_a + 2;
//        // Markers  are divided in (bits_a+2)x(bits_a+2) regions, of which the inner bits_axbits_a belongs to marker
//        // info
//        // the external border shoould be entirely black

//          float swidth = float(thres_img.rows) / float(bits_a2);
//        for (int y = 0; y < bits_a2; y++)
//        {
//            int inc = bits_a2 - 1;
//            if (y == 0 || y == bits_a2 - 1)
//                inc = 1;  // for first and last row, check the whole border
//            for (int x = 0; x < bits_a2; x += inc)
//            {
//                cv::Mat square = thres_img(cv::Rect( toInt(float(x) * swidth ), toInt(float(y) * swidth), swidth, swidth));
//                if (cv::countNonZero(square) > (swidth * swidth) / 2)
//                    return false;  // can not be a marker because the border element is not black!
//            }
//        }

//        // now,
//        // get information(for each inner square, determine if it is  black or white)

//        // now,
//        cv::Mat _bits = cv::Mat::zeros(bits_a, bits_a, CV_8UC1);
//        // get information(for each inner square, determine if it is  black or white)

//        for (int y = 0; y < bits_a; y++)
//        {
//            for (int x = 0; x < bits_a; x++)
//            {
//                int Xstart = toInt(float(x + 1) * swidth);
//                int Ystart = toInt(float(y + 1) * swidth);
//                cv::Mat square = thres_img(cv::Rect(Xstart, Ystart, swidth, swidth));
//                int nZ = cv::countNonZero(square);
//                if (nZ > (swidth * swidth) / 2)
//                    _bits.at<uchar>(y, x) = 1;
//            }
//        }
//        // now, get the 64bits ids

//        int nr = 0;
//        do
//        {
//            ids.push_back(touulong(_bits));
//            _bits = rotate(_bits);
//            nr++;
//        } while (nr < 4);
//        return true;
//    }

    // convert matrix of (0,1)s in a 64 bit value
    uint64_t DictionaryBased::touulong(const cv::Mat& code)
    {
        std::bitset<64> bits;
        int bidx = 0;
        for (int y = code.rows - 1; y >= 0; y--)
            for (int x = code.cols - 1; x >= 0; x--)
                bits[bidx++] = code.at<uchar>(y, x);
        return bits.to_ullong();
    }
    cv::Mat DictionaryBased::rotate(const cv::Mat& in)
    {
        cv::Mat out;
        in.copyTo(out);
        for (int i = 0; i < in.rows; i++)
        {
            for (int j = 0; j < in.cols; j++)
            {
                out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
            }
        }
        return out;
    }
}
