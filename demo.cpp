#include "ldlidar_driver.h" 
#include <cmath>          
#include <vector>    
#include <iostream>       
#include <numeric>     

struct ProcessedPoint {
    float x;         
    float y;           
    int intensity;     
    uint64_t stamp;    
};

struct ObjectCluster {
    std::vector<ProcessedPoint> points; 
    float center_x = 0.0f;           
    float center_y = 0.0f;            
    float width    = 0.0f;   
};

struct TrackedObject {
    int      id;                  
    float    x;                   
    float    y;                   
    float    prev_x;              
    float    prev_y;              
    float    width;               
    int      stationary_frames;   
    int      lost_frames;         
    int      age;                 
    bool     is_abandoned;       
    bool     matched_this_frame; 
};

uint64_t GetSystemTimeStamp(void) {
    auto tp  = std::chrono::time_point_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now());         
    auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(
                   tp.time_since_epoch());                    
    return static_cast<uint64_t>(dur.count());           
}

static void ComputeClusterCenter(ObjectCluster& cluster) {
    const size_t n = cluster.points.size();                    
    if (n == 0) return;                                       

    float sum_x = std::accumulate(cluster.points.begin(), cluster.points.end(), 0.0f,
                    [](float acc, const ProcessedPoint& p){ return acc + p.x; }); 
    float sum_y = std::accumulate(cluster.points.begin(), cluster.points.end(), 0.0f,
                    [](float acc, const ProcessedPoint& p){ return acc + p.y; }); 

    cluster.center_x = sum_x / static_cast<float>(n); 
    cluster.center_y = sum_y / static_cast<float>(n); 

    cluster.width = std::hypot(cluster.points.front().x - cluster.points.back().x,
                               cluster.points.front().y - cluster.points.back().y);
}

constexpr int    MIN_INTENSITY      = 15;    
constexpr float  MIN_DISTANCE_MM    = 200.f; 
constexpr float  MAX_DISTANCE_MM    = 5000.f;
constexpr float  FOV_MIN_DEG        = 90.f; 
constexpr float  FOV_MAX_DEG        = 270.f; 
constexpr float  CLUSTER_THRESH_MM  = 50.f;  
constexpr size_t MIN_CLUSTER_PTS    = 3;     
constexpr float  MIN_OBJ_WIDTH_MM   = 100.f; 
constexpr float  MAX_OBJ_WIDTH_MM   = 800.f; 
constexpr float  MAX_TRACKING_MM    = 400.f; 
constexpr float  STATIONARY_MM      = 30.f;  
constexpr int    MAX_LOST_FRAMES    = 10;    
constexpr int    MIN_AGE_FOR_DETECT = 2;     
constexpr int    LOOP_SLEEP_US      = 100000;

int main(int argc, char** argv) {
    if (argc < 4) exit(EXIT_FAILURE); 

    const std::string product_name(argv[1]);       
    const std::string communication_mode(argv[2]); 

    std::string      port_name, server_ip, server_port;
    uint32_t         serial_baudrate = 230400;          
    ldlidar::LDType  type_name;                          

    if      (communication_mode == "serialcom")            port_name   = argv[3];                     
    else if (communication_mode == "networkcom_tcpclient") { server_ip = argv[3]; server_port = argv[4]; } 
    else exit(EXIT_FAILURE);                                                                           

    ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver(); 
    node->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 
    node->EnableFilterAlgorithnmProcess(true);                            

    if      (product_name == "LD06") type_name = ldlidar::LDType::LD_06; 
    else if (product_name == "LD19") type_name = ldlidar::LDType::LD_19; 
    else exit(EXIT_FAILURE);                                               

    if (communication_mode == "serialcom") {
        if (!node->Start(type_name, port_name, serial_baudrate, ldlidar::COMM_SERIAL_MODE))
            exit(EXIT_FAILURE); 
    } else {
        if (!node->Start(type_name, server_ip.c_str(), server_port.c_str(), ldlidar::COMM_TCP_CLIENT_MODE))
            exit(EXIT_FAILURE); 
    }

    if (!node->WaitLidarCommConnect(3500)) { node->Stop(); exit(EXIT_FAILURE); } 

    ldlidar::Points2D      laser_scan_points;  
    std::vector<TrackedObject> tracked_objects; 
    int next_object_id = 1;                     

    std::cout << "디지털 트윈 감시 노드 가동 시작 (객체 분리 및 투기 탐지 모드)..." << std::endl;

    while (ldlidar::LDLidarDriver::IsOk()) { 

        switch (node->GetLaserScanData(laser_scan_points, 1500)) {

          case ldlidar::LidarStatus::NORMAL: { 

            std::vector<ProcessedPoint> filtered_points;
            filtered_points.reserve(laser_scan_points.size()); 

            for (const auto& pt : laser_scan_points) {
                if (pt.intensity < MIN_INTENSITY)                          continue; 
                if (pt.distance  < MIN_DISTANCE_MM ||
                    pt.distance  > MAX_DISTANCE_MM)                        continue;
                if (pt.angle     < FOV_MIN_DEG ||
                    pt.angle     > FOV_MAX_DEG)                            continue; 

                const float rad = pt.angle * (static_cast<float>(M_PI) / 180.0f); 
                filtered_points.push_back({ pt.distance * std::cos(rad),        
                                            pt.distance * std::sin(rad),         
                                            pt.intensity, pt.stamp });            
            }
            if (filtered_points.empty()) break; 

            std::vector<ObjectCluster> clusters;
            ObjectCluster current_cluster;

            for (const auto& fp : filtered_points) { 
                if (current_cluster.points.empty()) {                              
                    current_cluster.points.push_back(fp);                        
                    continue;
                }
                const float d = std::hypot(fp.x - current_cluster.points.back().x,
                                           fp.y - current_cluster.points.back().y); 
                if (d <= CLUSTER_THRESH_MM) {
                    current_cluster.points.push_back(fp);                         
                } else {
                    ComputeClusterCenter(current_cluster);                         
                    clusters.push_back(std::move(current_cluster));                
                    current_cluster.points.clear();                                
                    current_cluster.points.push_back(fp);                          
                }
            }
            if (!current_cluster.points.empty()) {
                ComputeClusterCenter(current_cluster);                          
                clusters.push_back(std::move(current_cluster));               
            }

            for (auto& t : tracked_objects) t.matched_this_frame = false;         
            std::vector<bool> cluster_claimed(clusters.size(), false);            

            for (auto& tracked : tracked_objects) {
                float min_dist      = MAX_TRACKING_MM; 
                int   best_idx      = -1;              

                for (size_t j = 0; j < clusters.size(); ++j) {
                    if (cluster_claimed[j])                     continue; 
                    if (clusters[j].points.size() < MIN_CLUSTER_PTS) continue;

                    const float d = std::hypot(tracked.x - clusters[j].center_x,
                                               tracked.y - clusters[j].center_y); 
                    if (d < min_dist) { min_dist = d; best_idx = j; }           
                }

                if (best_idx == -1) continue; 

                cluster_claimed[best_idx]    = true;  
                tracked.matched_this_frame   = true;  
                tracked.lost_frames          = 0;     
                tracked.age++;                        

                if (min_dist < STATIONARY_MM) {
                    tracked.stationary_frames++;     
                } else {
                    tracked.stationary_frames = 0;    
                    tracked.is_abandoned      = false; 
                }

                tracked.prev_x = tracked.x;
                tracked.prev_y = tracked.y;
                tracked.x      = clusters[best_idx].center_x; 
                tracked.y      = clusters[best_idx].center_y; 
                tracked.width  = clusters[best_idx].width;    
            }

            for (size_t j = 0; j < clusters.size(); ++j) {
                if (cluster_claimed[j])                          continue; 
                if (clusters[j].points.size() < MIN_CLUSTER_PTS) continue; 
                if (clusters[j].width < MIN_OBJ_WIDTH_MM ||
                    clusters[j].width > MAX_OBJ_WIDTH_MM)        continue; 

                bool is_separation = false; 
                int  person_id     = -1;   

                for (auto& tracked : tracked_objects) {
                    if (!tracked.matched_this_frame)    continue; 
                    if (tracked.stationary_frames != 0) continue; 
                    if (tracked.is_abandoned)           continue; 
                    if (tracked.age < MIN_AGE_FOR_DETECT) continue; 

                    const float d = std::hypot(clusters[j].center_x - tracked.prev_x,
                                               clusters[j].center_y - tracked.prev_y); 
                    if (d < MAX_TRACKING_MM) {
                        is_separation = true;    
                        person_id     = tracked.id; 
                        break;                 
                    }
                }
                
                const uint64_t event_time = GetSystemTimeStamp();

                TrackedObject new_obj {
                    next_object_id++,                          
                    clusters[j].center_x, clusters[j].center_y, 
                    clusters[j].center_x, clusters[j].center_y, 
                    clusters[j].width,                         
                    0, 0, 1,                                    
                    is_separation, true                          
                };
                tracked_objects.push_back(new_obj);

                if (is_separation) {
                    std::cout << "\n[ALERT] 객체 분리 및 투기 현장 감지!\n"
                              << " -> 이탈하는 사람 ID : " << person_id   << "\n"
                              << " -> 남겨진 투기물 ID : " << new_obj.id << "\n"
                              << " -> 발생 시각 (ns)   : " << event_time << std::endl;
                } else {
                    std::cout << "[INFO] 신규 진입 감지 | ID: " << new_obj.id << std::endl; 
                }
            }

            for (auto it = tracked_objects.begin(); it != tracked_objects.end(); ) {
                if (!it->matched_this_frame) {
                    it->lost_frames++;                                        
                    if (it->lost_frames > MAX_LOST_FRAMES) {
                        std::cout << "[INFO] 객체 소실 | ID: " << it->id << std::endl; 
                        it = tracked_objects.erase(it);                
                        continue;                                            
                    }
                }
                ++it; 
            }
            break; 
          }

          case ldlidar::LidarStatus::DATA_TIME_OUT:
            node->Stop(); 
            break;

          default: break;
        }

        usleep(LOOP_SLEEP_US);
    }

    node->Stop();  
    delete node;   
    return 0;      
}