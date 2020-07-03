#include <opencv2/opencv.hpp>
 
// ORB-SLAM的系统接口
#include "System.h"
#include <string>
#include <chrono>   // for time stamp
#include <iostream>

#include <memory>
#include <sstream>
#include <string>

#include "libirimager/direct_binding.h"
 
using namespace std;
 
// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "./irsetting/mono_ir.yaml";
string vocFile = "./irsetting/ORBvoc.txt";
 
int main(int argc, char **argv) {
 
    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);

    cout<<"hello"<<endl;
 
    
    char config_path[] = "./irsetting/17071004.xml";
    
    if(::evo_irimager_usb_init(config_path, 0, 0) != 0) return -1;
    int err;
    int p_w;
    int p_h;
    if((err = ::evo_irimager_get_palette_image_size(&p_w, &p_h)) != 0)
    {
        std::cerr << "error on evo_irimager_get_palette_image_size: " << err << std::endl;
        exit(-1);
    }

    std::cout << "palette_image_size::::" << p_w << " x " << p_h <<std::endl;

    int t_w;
    int t_h;

    cout<<t_w<<endl;

    if((err = ::evo_irimager_get_thermal_image_size(&t_w, &t_h)) != 0)
    {
        std::cerr << "error on evo_irimager_get_palette_image_size: " << err << std::endl;
        exit(-1);
    }
    std::cout << "thermal_image_size::::" << t_w << " x " << t_h <<std::endl;
    std::vector<unsigned char> palette_image(p_w * p_h * 3);
    std::vector<unsigned short> thermal_data(t_w * t_h);

    while(1)
    {
    auto now = chrono::system_clock::now();
    // 记录系统时间
    auto start = chrono::system_clock::now();

    if((err = ::evo_irimager_get_thermal_palette_image(t_w, t_h, &thermal_data[0], p_w, p_h, &palette_image[0]))==0)
    {
        cout<< err<<endl;
        //--Code for displaying image -----------------
        cv::Mat thermal(t_h, t_w, CV_16UC1);

        memcpy(thermal.data, thermal_data.data() , thermal_data.size() * sizeof(unsigned short));
        
        cv::Mat thermal2(t_h, t_w, CV_8U);

        for(int i = 0; i < thermal2.rows; i++)
            for(int j = 0; j < thermal2.cols; j++)
            {
                double t = ((double)thermal.at<ushort>(i, j) - 1000.0) / 10.0;
                thermal2.at<uchar>(i, j) = int((t - 10) / (100 - 10) * 255.0);
            }

        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        // test
        if(thermal2.data) std::cout<<"OKKKKKK"<<std::endl;
        SLAM.TrackMonocular(thermal2, double(timestamp.count())/1000.0);

        }
        else std::cerr << "failed evo_irimager_get_thermal_palette_image: " << err << std::endl;
        
    };//cvGetWindowHandle("palette image daemon"));

    ::evo_irimager_terminate();

 
    return 0;
}