#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include <vector>

using namespace std;

const std::string CameraNames[8]={"B_FISHEYE_C",
                            "F_FISHEYE_C",
                            "F_STEREO_L",
                            "F_STEREO_R",
                            "M_FISHEYE_L",
                            "M_FISHEYE_R",
                            "M_WINGMIRROR_L",
                            "M_WINGMIRROR_R"};

class CameraModel{
private:
    const string modelAttributes[7]={"image_resolution_px",
                        "reprojection_error",
                        "model",
                        "principal_point_px",
                        "focal_length_px",
                        "distortion_coeffs",
                        "xi"};
    uint16_t image_resolution_px[2];
    double reprojection_error;
    bool model; // 'true' - mei, 'false' - opencv_pinhole
    uint16_t principal_point_px[2];
    double focal_length_px[2];
    double distortion_coeffs[5];
    double xi;
public:
    CameraModel(string path){
        string myText;
        ifstream MyReadFile(path);
        while (getline (MyReadFile, myText)) {
            if(myText.find(modelAttributes[0]) != string::npos){
                myText = myText.erase(0, myText.find("[")+1);
                image_resolution_px[0] = stoi(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                image_resolution_px[1] = stoi(myText.substr(0, myText.find("]")));
            }
            else if(myText.find(modelAttributes[1]) != string::npos){
                myText = myText.erase(0, myText.find(": ")+2);
                reprojection_error = stod(myText);
            }
            else if(myText.find(modelAttributes[2]) != string::npos){
                model = myText.find("mei") != string::npos;
            }
            else if(myText.find(modelAttributes[3]) != string::npos){
                myText = myText.erase(0, myText.find("[")+1);
                principal_point_px[0] = stod(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                principal_point_px[1] = stod(myText.substr(0, myText.find("]")));
            }
            else if(myText.find(modelAttributes[4]) != string::npos){
                myText = myText.erase(0, myText.find("[")+1);
                focal_length_px[0] = stod(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                focal_length_px[1] = stod(myText.substr(0, myText.find("]")));
            }
            else if(myText.find(modelAttributes[5]) != string::npos){
                myText = myText.erase(0, myText.find("[")+1);
                distortion_coeffs[0] = stod(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                distortion_coeffs[1] = stod(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                distortion_coeffs[2] = stod(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                distortion_coeffs[3] = stod(myText.substr(0, myText.find(", ")));
                myText = myText.erase(0, myText.find(" ")+1);
                distortion_coeffs[4] = stod(myText.substr(0, myText.find("]")));
            }
            else if(myText.find(modelAttributes[6]) != string::npos){
                myText = myText.erase(0, myText.find(": ")+2);
                reprojection_error = stod(myText);
            }
        }
    }

    void GetResolution(uint16_t& x, uint16_t&y){
        x = image_resolution_px[0];
        y = image_resolution_px[1];
    }
};
/**
@params pts_view : points in view coords (should only contain points in frustum, so that only needed
points are in it)
@params camera_model : intrinsic calibration of the camera
...
@params out_pts_visible : points visible after filter (in view, output)
@params out_idx : kept point indexes (output)
@return none
**/
void filter(const std::vector<glm::vec3>& pts_view, CameraModel& camera_model, std::vector<glm::vec3>& out_pts_visible, std::vector<int32_t>& out_idx)
{
    cout << "this is the body of filter method" << endl;
    //TODO: apply occlusion algorithm
}

int main()
{
    // TODO create CameraModell class
    // TODO: read points
    // TODO: read camera model
    string path = "D:/joci/projects/AImotive/Obstacle detection/Description/research_scientist_obstacle_data/calibration/";
    string calibPath = path + CameraNames[0] + ".yaml";
    CameraModel test(calibPath);
    uint16_t x,y = 0;
    test.GetResolution(x,y);

    cout << x << endl;
    return 0;
}
