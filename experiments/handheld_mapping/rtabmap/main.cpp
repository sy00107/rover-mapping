#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <memory>
using namespace rtabmap;
int main(int argc,char**argv) {
 if(argc!=9 && argc!=11) {std::cerr<<"pairs output fx fy cx cy baseline max_frames(0=all) [rgbd depth_scale_m]\n";return 2;}
 ULogger::setLevel(ULogger::kError);
 const bool rgbd=argc==11 && std::string(argv[9])=="rgbd";
 if(argc==11 && !rgbd) return 2;
 const double depthScale=rgbd?std::stod(argv[10]):0.;
 if(rgbd && !(depthScale>0 && depthScale<.1)) return 2;
 std::string output=argv[2];
 if(std::ifstream(output+"/rtabmap.db").good()) {std::cerr<<"Refusing existing database\n";return 2;}
 ParametersMap p{{"Rtabmap/DetectionRate","0"},{"Mem/STMSize","30"},{"Kp/DetectorStrategy","6"},{"Vis/FeatureType","6"},{"Grid/FromDepth","false"}};
 if(rgbd) {p["Vis/MaxDepth"]="2.0";p["Vis/MinDepth"]="0.25";}
 std::unique_ptr<Odometry> odom(Odometry::create(p));
 Rtabmap slam; slam.init(p,output+"/rtabmap.db");
 StereoCameraModel model(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]),std::stod(argv[7]),Transform::getIdentity(),cv::Size(640,480));
 CameraModel depthModel(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]),std::stod(argv[6]),Transform::getIdentity(),0.,cv::Size(640,480));
 std::ofstream raw(output+"/rtab_odometry_raw.tum");
 std::ifstream input(argv[1]); if(!input) return 2;
 std::ofstream timings(output+"/timings.csv"); timings<<"stamp,tracking_ok,odom_ms,slam_ms,inliers,matches,covariance_linear,covariance_angular\n";
 std::string line; int n=0,failed=0; int limit=std::stoi(argv[8]);
 while(std::getline(input,line) && (!limit || n<limit)) {
  std::istringstream ss(line);std::string stamp,left,right;std::getline(ss,stamp,'\t');std::getline(ss,left,'\t');std::getline(ss,right);
  double t=std::stoll(stamp)/1e9; ++n;
  cv::Mat l=cv::imread(left,0),r=cv::imread(right,rgbd?cv::IMREAD_UNCHANGED:cv::IMREAD_GRAYSCALE);
  if(l.empty()||r.empty()||l.size()!=cv::Size(640,480)||r.size()!=l.size()) return 3;
  SensorData data;
  if(rgbd) {
   if(r.type()!=CV_16UC1) return 3;
   cv::Mat metric; r.convertTo(metric,CV_32FC1,depthScale);
   metric.setTo(0,metric<.25);metric.setTo(0,metric>2.);
   data=SensorData(l,metric,std::vector<CameraModel>{depthModel},n,t);
  } else data=SensorData(l,r,model,n,t);
  OdometryInfo info;
  auto a=std::chrono::steady_clock::now(); Transform pose=odom->process(data,&info); auto b=std::chrono::steady_clock::now();
  if(!pose.isNull()){slam.process(data,pose,info.reg.covariance.empty() ? cv::Mat::eye(6,6,CV_64FC1) : info.reg.covariance);
   auto q=pose.getQuaterniond();raw<<std::setprecision(17)<<t<<' '<<pose.x()<<' '<<pose.y()<<' '<<pose.z()<<' '<<q.x()<<' '<<q.y()<<' '<<q.z()<<' '<<q.w()<<'\n';
  } else ++failed;
  auto c=std::chrono::steady_clock::now();
  timings<<std::setprecision(17)<<t<<','<<!pose.isNull()<<','<<std::chrono::duration<double,std::milli>(b-a).count()<<','<<std::chrono::duration<double,std::milli>(c-b).count()<<','<<info.reg.inliers<<','<<info.reg.matches<<','<<(info.reg.covariance.empty()?-1:info.reg.covariance.at<double>(0,0))<<','<<(info.reg.covariance.empty()?-1:info.reg.covariance.at<double>(3,3))<<'\n';
  if(n%100==0) std::cout<<n<<" frames; lost="<<failed<<std::endl;
 }
 std::map<int,Transform> poses;std::multimap<int,Link> links;
 // RTAB-Map generates node IDs independently of input frame IDs, particularly
 // after tracking failures. Export the timestamp stored by each graph node.
 std::map<int,Signature> signatures;
 slam.getGraph(poses,links,true,true,&signatures,false,false,false,false,false,false);
 std::map<int,double> used;
 for(const auto &v:poses) if(v.first>0) {
  auto signature=signatures.find(v.first);
  if(signature==signatures.end() || signature->second.getStamp()<=0) {
   std::cerr<<"Missing timestamp for graph node "<<v.first<<std::endl;
   slam.close();return 4;
  }
  used[v.first]=signature->second.getStamp();
 }
 if(!graph::exportPoses(output+"/rtab_optimized.tum",10,poses,links,used)) return 4;
 slam.close();
 std::ofstream summary(output+"/runner_summary.json");summary<<"{\"frames\":"<<n<<",\"lost\":"<<failed<<",\"nodes\":"<<poses.size()<<",\"features\":\"GFTT+BRIEF\",\"input\":\""<<(rgbd?"left IR + measured depth; optical frame; no IMU":"rectified stereo IR; optical frame; no IMU")<<"\"}\n";
}
