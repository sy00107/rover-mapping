// Offline ORB final camera poses -> RTAB RGB-D map. No RTAB odometry or extra loop correction.
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
using namespace rtabmap;
int main(int argc,char**argv) {
 if(argc!=9) {std::cerr<<"input.tsv output fx fy cx cy depth_scale_m count\n";return 2;}
 ULogger::setLevel(ULogger::kError);
 std::string output=argv[2];
 if(std::ifstream(output+"/rtabmap.db").good()) return 2;
 ParametersMap p{{"Rtabmap/DetectionRate","0"},{"Rtabmap/LoopThr","1"},
  {"RGBD/ProximityBySpace","false"},{"RGBD/ProximityByTime","false"},
  {"RGBD/NeighborLinkRefining","false"},{"Optimizer/Iterations","0"},
  {"RGBD/LinearUpdate","0"},{"RGBD/AngularUpdate","0"},{"Mem/RehearsalSimilarity","1"},
  {"Grid/FromDepth","false"},{"Kp/DetectorStrategy","6"},{"Vis/FeatureType","6"},
  {"Vis/MinDepth","0.25"},{"Vis/MaxDepth","2.0"}};
 Rtabmap slam;slam.init(p,output+"/rtabmap.db");
 CameraModel model(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]),Transform::getIdentity(),0.,cv::Size(640,480));
 const double scale=std::stod(argv[7]);if(!(scale>0 && scale<.1)) return 2;
 std::ifstream input(argv[1]);if(!input) return 2;
 std::map<int,double> stamps;std::string line;int n=0;
 std::ofstream imported(output+"/orb_imported.tum");
 while(std::getline(input,line)) {
  std::istringstream ss(line);std::string stamp,left,depth;double x,y,z,qx,qy,qz,qw;
  std::getline(ss,stamp,'\t');std::getline(ss,left,'\t');std::getline(ss,depth,'\t');
  if(!(ss>>x>>y>>z>>qx>>qy>>qz>>qw)) return 3;
  cv::Mat image=cv::imread(left,0),raw=cv::imread(depth,cv::IMREAD_UNCHANGED),metric;
  if(image.size()!=cv::Size(640,480)||raw.size()!=image.size()||raw.type()!=CV_16UC1) return 3;
  raw.convertTo(metric,CV_32FC1,scale);metric.setTo(0,metric<.25);metric.setTo(0,metric>2.);
  double t=std::stoll(stamp)/1e9;++n;Transform pose(x,y,z,qx,qy,qz,qw);
  SensorData data(image,metric,std::vector<CameraModel>{model},n,t);
  // ORB export does not provide covariance. Nominal only; graph correction is disabled.
  if(!slam.process(data,pose,cv::Mat::eye(6,6,CV_64FC1)*1e-4)) return 4;
  stamps[n]=t;auto q=pose.getQuaterniond();
  imported<<std::setprecision(17)<<t<<' '<<pose.x()<<' '<<pose.y()<<' '<<pose.z()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
  if(n%100==0)std::cout<<n<<" ORB keyframes stored"<<std::endl;
 }
 if(n!=std::stoi(argv[8]))return 5;
 std::map<int,Transform> poses;std::multimap<int,Link> links;slam.getGraph(poses,links,true,true);
 std::map<int,double> used;for(auto &v:poses)used[v.first]=stamps.at(v.first);
 if(!graph::exportPoses(output+"/rtab_optimized.tum",10,poses,links,used))return 6;
 slam.close();
 std::ofstream report(output+"/runner_summary.json");
 report<<"{\"input_keyframes\":"<<n<<",\"graph_nodes\":"<<poses.size()<<",\"pose_source\":\"ORB offline final camera poses\",\"rtab_odometry\":false,\"rtab_extra_loop_correction\":false,\"optimizer_iterations\":0,\"covariance\":\"nominal, not measured\"}\n";
}
