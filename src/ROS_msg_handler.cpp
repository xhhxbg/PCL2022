#include "ROSmsg.h"

#include <sstream>
#include <fstream>
#include <string>

int main() {
    int file_num = 4;
    std::ifstream file;
    for (int group = 0; group < 4; group++)
    {
        file.open("/home/yyf/Documents/ros_srtp/pcd/" + std::to_string(group) + "/data.txt");
        for (int index = 0; index < file_num; index++)
        {
            char buf[1014];
            file.getline(buf, sizeof(buf));//读取行
            std::stringstream word(buf);
            ROSmsg pc1(group, index);
            pc1.load_pcd();
            pc1.load_mat(word);
            pc1.show_origin();
            pc1.show_aligned();
            pcl::io::savePCDFile ("../data/cloud" + std::to_string(group) + "_" + std::to_string(index) + ".pcd", *(pc1.cloud_aligned));
        }
        file.close();
    }

}