#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h> //기존에서 추가된거 
#include <message_filters/time_synchronizer.h> //기존에서 추가된거 
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h> // 화살표 관련인듯
#include <visualization_msgs/MarkerArray.h> //화살표 관련인듯


class LidarManipulatorSync {
public:
    LidarManipulatorSync() : tfListener(tfBuffer) {
        // 퍼블리셔 설정 (보정된 포인트 클라우드 퍼블리시)
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("accumulated_pointcloud", 1);

        // message_filters를 이용해 두 토픽을 동기화
        scan_sub.subscribe(nh, "scan", 10);
        joint_state_sub.subscribe(nh, "/joint_states", 10);
        sync.reset(new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::JointState>(scan_sub, joint_state_sub, 10));
        sync->registerCallback(boost::bind(&LidarManipulatorSync::syncedCallback, this, _1, _2));

        // 누적 포인트 클라우드 초기화
        accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        accumulated_cloud->width = 0;
        accumulated_cloud->height = 1;

        // DH 파라미터 및 초기 joint_positions 등 필요한 초기화 (여기서는 간략하게 처리)
     dh_parameters <<    0.0,    0.0,    0.0,     // 1.5078 rad = 90도
                            0.0,  1.507,    0.0,     //a, alpha, d 순서 !!!
                           0.13,    0.0,    0.0,     //잘못 입력되었던 DH 파라미터 수정 (우선 그리퍼값 부여)
                          0.126,    0.0,    0.0,
                          0.124, -1.507,    0.0; 

    
        // DH 파라미터 초기화 (a, alpha, d)
        // 예: dh_parameters << 0.0, 0.0, 0.0,  0.0, 1.507, 0.0, ...;
        // 실제 값에 맞게 채워넣어야 함.
        joint_positions.resize(5);
        joint_positions.setZero();
    }

    // 동기화된 콜백: Lidar와 JointState 데이터를 동시에 처리
    void syncedCallback(const sensor_msgs::LaserScan::ConstPtr& scan,
                        const sensor_msgs::JointState::ConstPtr& joint_state) {
        // 1. joint_state 데이터를 이용해 현재 조인트 각도를 업데이트
        updateJointPositions(joint_state);

        // 2. tf를 사용하여 Lidar 좌표계를 Manipulator 좌표계로 변환하기 위한 변환을 조회
        geometry_msgs::TransformStamped tfTransform;
        try {
            // 예: lidar 프레임에서 manipulator_base 프레임으로의 변환 (프레임 이름은 실제 설정에 따라 조정)
            tfTransform = tfBuffer.lookupTransform("manipulator_base", scan->header.frame_id,
                                                     scan->header.stamp, ros::Duration(0.1));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("TF lookup failed: %s", ex.what());
            return;
        }

        // 3. Lidar 스캔 데이터를 포인트 클라우드로 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (std::isfinite(range)) {
                float angle = scan->angle_min + i * scan->angle_increment;
                pcl::PointXYZ pt;
                pt.x = range * cos(angle);
                pt.y = range * sin(angle);
                pt.z = 0;
                cloud_lidar->points.push_back(pt);
            }
        }

        // 4. (선택 사항) 조인트 데이터를 기반으로 kinematics 계산 수행
        // 예: calculateTransforms() 호출하여 end_effector_T 계산
        calculateTransforms();

        // 5. tf 변환과 kinematics 결과를 결합하여 보정 변환 행렬을 구성
        // 여기서는 tf에서 얻은 변환을 사용하여 Lidar 포인트들을 Manipulator 좌표계로 변환
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_manipulator(new pcl::PointCloud<pcl::PointXYZ>);
        // tfTransform을 Eigen::Matrix4f 형식으로 변환
        Eigen::Matrix4f tf_matrix = transformMsgToEigen(tfTransform.transform);

        // 예를 들어, kinematics 결과 end_effector_T와 tf_matrix를 곱하여 최종 보정 행렬을 얻을 수 있음.
        // 최종 변환: 최종변환 = end_effector_T * tf_matrix;
        // (여기서는 단순히 tf_matrix만 적용하는 예시로 진행)
        pcl::transformPointCloud(*cloud_lidar, *cloud_manipulator, tf_matrix);

        // 6. 누적 포인트 클라우드에 합산
        *accumulated_cloud += *cloud_manipulator;

        // 7. ROS 메시지로 변환 후 퍼블리시
        sensor_msgs::PointCloud2 pc2_msg;
        pcl::toROSMsg(*accumulated_cloud, pc2_msg);
        pc2_msg.header.frame_id = "manipulator_base";
        pc2_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(pc2_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher cloud_pub;

    // message_filters 구독자와 동기화 객체
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub;
    message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub;
    boost::shared_ptr< message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::JointState> > sync;

    // tf 관련
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // 누적 포인트 클라우드
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;

    // DH 파라미터 및 조인트 상태 (예시로 간단하게)
    Eigen::MatrixXf dh_parameters; // (5 x 3)
    Eigen::VectorXf joint_positions;
    Eigen::Matrix4f end_effector_T; // kinematics를 통한 보정 결과 (calculateTransforms에서 계산)

    // 예시: JointState를 업데이트하는 함수
    void updateJointPositions(const sensor_msgs::JointState::ConstPtr& joint_state) {
        // 관절 이름에 따라 적절히 업데이트 (예시는 기존 코드와 유사)
        for (size_t i = 0; i < joint_state->position.size(); ++i) {
            if (joint_state->name[i] == "joint1")
                joint_positions(0) = joint_state->position[i];
            else if (joint_state->name[i] == "joint2")
                joint_positions(1) = joint_state->position[i]; // 보정값 추가 가능
            else if (joint_state->name[i] == "joint3")
                joint_positions(2) = joint_state->position[i];
            else if (joint_state->name[i] == "joint4")
                joint_positions(3) = joint_state->position[i];
        }
        // 5번째 관절(예시)
        if(joint_positions.size() >= 5)
            joint_positions(4) = 0;
    }

    // 예시: DH 파라미터와 joint_positions를 이용한 kinematics 계산
  void calculateTransforms() {     
        ROS_INFO("calculateTransforms() method called");  // 디버깅용 출력문      
        Eigen::Matrix4f T01, T02, T03, T04, T12, T23, T34, T45;
        T01 = T02 = T03 = T04 = T12 = T23 = T34 = T45 = Eigen::Matrix4f::Identity(); // 각 관절의 변환 행렬 초기화

        for (int i = 0; i < 5; ++i) {
                       
            float a = dh_parameters(i, 0);
            float alpha = dh_parameters(i, 1);
            float d = dh_parameters(i, 2);
            float theta = joint_positions(i);
          
            Eigen::Matrix4f T;
            T <<             cos(theta),              -sin(theta),           0,               a,
                sin(theta) * cos(alpha),  cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
                sin(theta) * sin(alpha),  cos(theta) * sin(alpha),  cos(alpha),  cos(alpha) * d,
                                      0,                        0,           0,              1;

            if (i == 0) {
                T01 = T;
            } else if (i == 1) {
                T12 = T;
            } else if (i == 2) {
                T23 = T;
            } else if (i == 3) {
                T34 = T;
            } else if (i == 4) {
                T45 = T;
            }            
        }
        
        T02 = T01 * T12;             // Joint 2까지의 변환 행렬
        T03 = T01 * T12 * T23;       // Joint 3까지의 변환 행렬
        T04 = T01 * T12 * T23 * T34; // Joint 4가지의 변환 행렬
        end_effector_T = T01 * T12 * T23 * T34 * T45; // 모든 관절의 변환 행렬을 곱하여 최종 엔드 이펙터의 변환 행렬을 얻음
        end_effector_T(2, 3) = end_effector_T(2, 3) + (0.077 + 0.09); //DH parameter로 구현되지 못한 z축 0.077 보정 + 라이다 높이 보정 (0.09)

        ROS_INFO_STREAM("T01 transformation matrix:\n" << T01); // 디버깅용 출력문 T01
        ROS_INFO_STREAM("T02 transformation matrix:\n" << T02); // 디버깅용 출력문 T02
        ROS_INFO_STREAM("T03 transformation matrix:\n" << T03); // 디버깅용 출력문 T03
        ROS_INFO_STREAM("End effector transformation matrix:\n" << end_effector_T); // 디버깅용 출력문 end_effector_T
        
    }
    
      void visualizeTransforms() {
        visualization_msgs::MarkerArray marker_array;
       
        publishTransformMarker(T01, "T01", 0.0, 1.0, 0.0, marker_array);//초록색      
        publishTransformMarker(T02, "T02", 0.0, 0.0, 1.0, marker_array);//파란색       
        publishTransformMarker(T03, "T03", 1.0, 0.0, 1.0, marker_array);//보라색        
        publishTransformMarker(end_effector_T, "end_effector_T", 1.0, 1.0, 1.0, marker_array);//흰색

        visualization_pub.publish(marker_array);
    }

        void publishTransformMarker(const Eigen::Matrix4f& transform, const std::string& marker_ns, float r, float g, float b, visualization_msgs::MarkerArray& marker_array) {
        Eigen::Vector3f position = transform.block<3, 1>(0, 3); // 병진성분t 추출 
        Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0); // 회전성분R 추출
        Eigen::Quaternionf quaternion(rotation); // 회전성분을 쿼터니언으로 변환하기 위한 행렬 선언

        // Normalize the quaternion to ensure valid values
        quaternion.normalize(); // 정규화

        tf2::Quaternion tf_quaternion;
        tf_quaternion.setValue(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()); // 쿼터니언 변환

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = marker_ns;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = position(0);
        marker.pose.position.y = position(1); //Translation값 적용
        marker.pose.position.z = position(2);
        tf2::convert(tf_quaternion, marker.pose.orientation);  // Rotation값 적용 (쿼터니언)
        marker.scale.x = 1.0;  
        marker.scale.y = 0.1;   //화살표 크기 세팅
        marker.scale.z = 0.1;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_array.markers.push_back(marker);
    }

    // geometry_msgs::Transform 을 Eigen::Matrix4f 로 변환하는 헬퍼 함수
    Eigen::Matrix4f transformMsgToEigen(const geometry_msgs::Transform& transform_msg) {
        Eigen::Matrix4f eigen_transform = Eigen::Matrix4f::Identity();

        // 회전 (쿼터니언 -> 회전 행렬)
        Eigen::Quaternionf q(transform_msg.rotation.w,
                               transform_msg.rotation.x,
                               transform_msg.rotation.y,
                               transform_msg.rotation.z);
        Eigen::Matrix3f R = q.normalized().toRotationMatrix();

        // Eigen 행렬에 대입
        eigen_transform.block<3,3>(0,0) = R;
        eigen_transform(0,3) = transform_msg.translation.x;
        eigen_transform(1,3) = transform_msg.translation.y;
        eigen_transform(2,3) = transform_msg.translation.z;

        return eigen_transform;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_manipulator_sync_node");
    LidarManipulatorSync node;
      ros::Rate loop_rate(10); // 속도 조절
    while (ros::ok()) {
        manipulator.visualizeTransforms();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
