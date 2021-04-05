#include <iostream>
#include <glm/glm.hpp>
#include <vector>

using namespace std;

/**
@params pts_view : points in view coords (should only contain points in frustum, so that only needed
points are in it)
@params camera_model : intrinsic calibration of the camera
...
@params out_pts_visible : points visible after filter (in view, output)
@params out_idx : kept point indexes (output)
@return none
**/
void filter(const std::vector<glm::vec3>& pts_view, CameraModell& camera_model, std::vector<glm::vec3>& out_pts_visible, std::vector<int32_t>& out_idx)
{
    cout << "this is the body of filter method" << endl;
    //TODO: apply occlusion algorithm
}

int main()
{
    // TODO create CameraModell class
    // TODO: read points
    // TODO: read camera model


    cout << "Hello world!" << endl;
    return 0;
}
