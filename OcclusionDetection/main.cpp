#include <iostream>
#include <fstream>
#include <glm/glm.hpp>
#include <vector>
#include <bits/stdc++.h>

using namespace std;

const std::string CameraNames[8]={"B_FISHEYE_C",
                            "F_FISHEYE_C",
                            "F_STEREO_L",
                            "F_STEREO_R",
                            "M_FISHEYE_L",
                            "M_FISHEYE_R",
                            "M_WINGMIRROR_L",
                            "M_WINGMIRROR_R"};

struct MyPoint{
        int idx;
        float z_value;
};




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
        MyReadFile.close();
    }

    void GetResolution(uint16_t& x, uint16_t&y){
        x = image_resolution_px[0];
        y = image_resolution_px[1];
    }
};

void LoadPoints(string pointPath, vector<glm::vec3>& points_out)
{
    cout << "Points are loading..." << endl;
    bool startParse = false;
    string myText;
    ifstream MyReadFile(pointPath);
    float x, y, z;
    while (getline (MyReadFile, myText)) {
        if(startParse){
            x = stof(myText.substr(0, myText.find(" ")));
            myText = myText.erase(0, myText.find(" ")+1);
            y = stof(myText.substr(0, myText.find(" ")));
            myText = myText.erase(0, myText.find(" ")+1);
            z = stof(myText.substr(0, myText.find(" ")));
            points_out.push_back(glm::vec3(x,y,z));
        }
        if(myText == "end_header"){startParse = true;}
    }
    MyReadFile.close();
}

/** \brief
* \param pts_view : points in view coords (should only contain points in frustum, so that only needed points are in it)
* \param camera_model : intrinsic calibration of the camera
* \param out_pts_visible : points visible after filter (in view, output)
* \param out_idx : kept point indexes (output)
* \return none
*/
void filter(const std::vector<glm::vec3>& pts_view, CameraModel& camera_model, std::vector<glm::vec3>& out_pts_visible, std::vector<int32_t>& out_idx)
{
    std::map<std::pair<int, int>, MyPoint> mapOfPoints;
    int resolution_x = 500;
    int resolution_y = 500;
    bool itr;
    float inverse_z = 0.0f;
    int idx_x = 0;
    int idx_y = 0;
    int i = 0;
    int x_max = 0;
    cout << "Computation started..." << endl;
    for(glm::vec3 item : pts_view){
        inverse_z = 1/item.z;
        idx_x = int(item.x*inverse_z*resolution_x);
        idx_y = int(item.y*inverse_z*resolution_y);
        if(mapOfPoints.find(std::make_pair(idx_x, idx_y)) != mapOfPoints.end()){
            if(mapOfPoints[std::make_pair(idx_x, idx_y)].z_value > item.z){
                mapOfPoints[std::make_pair(idx_x, idx_y)].idx = i;
                mapOfPoints[std::make_pair(idx_x, idx_y)].z_value = item.z;
            }
        }else{
            mapOfPoints[std::make_pair(idx_x, idx_y)].idx = i;
            mapOfPoints[std::make_pair(idx_x, idx_y)].z_value = item.z;
        }
        i++;
    }
    for (auto const& x : mapOfPoints){
        out_idx.push_back(x.second.idx);
        out_pts_visible.push_back(pts_view[x.second.idx]);
    }
    cout << "Computation finished!" << endl;
}

int main()
{
    // general path definition
    string path = "D:/joci/projects/AImotive/Obstacle detection/Description/research_scientist_obstacle_data/";

    //camera model read
    string calibPath = path + "calibration/" + CameraNames[2] + ".yaml";
    CameraModel cameraModel(calibPath);
    uint16_t x,y = 0;
    cameraModel.GetResolution(x,y);
    cout << "Cameramodel loaded with resolution of: " << x << "x" << y << endl;

    //read points
    string pointPath = path + "viewpoints/" + CameraNames[2] + "/00006526_viewpts.ply";
    vector<glm::vec3> points;
    LoadPoints(pointPath, points);
    cout << "The number of points loaded: " << points.size() << endl;

    //apply occlusion algorithm
    std::vector<glm::vec3> visiblePoints;
    std::vector<int32_t> visiblePoints_idx;
    filter(points, cameraModel, visiblePoints, visiblePoints_idx);
    cout << "The number of visiblePoints: " << visiblePoints.size() << endl;
    cout << "The number of visiblePoints_idx: " << visiblePoints_idx.size() << endl;

    //output for test file
    ofstream testFile("./test.txt");

    for(glm::vec3 item : visiblePoints){
        testFile << item.x << " " << item.y << " " << item.z << endl;
    }
    testFile.close();

    return 0;
}
