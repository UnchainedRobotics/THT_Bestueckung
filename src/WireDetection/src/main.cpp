// Code for camera control partly taken from Photoneo examples and PCL tutorials from https://pointclouds.org/ and PCL documentation

#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#define PHOXI_PCL_SUPPORT
#define PHOXI_OPENCV_SUPPORT

#include "pcl_main_operations.hpp"
#include "additional_functions.hpp"
#include <vector>
#include <string>
#include <zmq.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>


using PointT = pcl::PointXYZRGB;
pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
float leaf_size = 0.0001f;


void convertToOpenCV(const pho::api::PFrame &Frame, cv::Mat &pointCloudMat, cv::Mat &graymat);

void leave_only_wire(cv::Mat &image_3d, cv::Mat &grayimage, cv::Mat &processed_image, PointT min_p, PointT max_p,
                     float threshold);

bool compare(PointT p1, PointT p2) {
    return p1.y < p2.y;
}


int main(int argc, char *argv[]) {
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind("tcp://127.0.0.1:5556");
    pho::api::PPhoXi PhoXiDevice = create_PphoXi();
    PhoXiDevice->StopAcquisition();
    zmq::message_t request;


    while (1) {
        // defining variable nedded in the loop
        PointT min_point_AABB;
        PointT max_point_AABB;
        PointT min_point_OBB;
        PointT max_point_OBB;
        PointT position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;

        CloudT::Ptr wire_1(new CloudT);
        CloudT::Ptr wire_2(new CloudT);

        float y_max_1 = -1000;
        float y_max_2 = -1000;

        //  Wait for next request from client
        socket.recv(&request);
        std::string task = std::string(static_cast<char *>(request.data()), request.size());
        // Print out received message
        std::cout << "Received task from client: " + task << std::endl;
        if (task == "quit") {
            break;
        }
        //Send reply
        std::string answer = std::string(task);
        std::vector <u_char> answer_u;
        zmq::message_t reply(task.length());
        memcpy(reply.data(), task.data(), task.length());
        socket.send(reply, ZMQ_NOBLOCK);

        socket.recv(&request);
        std::string threshold_str = std::string(static_cast<char *>(request.data()), request.size());
        std::cout << "Received threshold from client: " + threshold_str << "\n";

        //take image, create and cut cloud
        pho::api::PFrame Frame = startSoftwareTriggerExample(PhoXiDevice);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud <pcl::PointXYZRGB>);
        std::cout << "Frame " << Frame << "\n";
        pho::api::PhoXiTimeout timeout;
        pcl::PointCloud<PointT>::Ptr temp_clood_ptr(
                new pcl::PointCloud<PointT>());
        pcl::PointCloud <PointT> MyPCLCloud, MyPCLCloudTemp;
        Frame->ConvertTo(MyPCLCloud);
        std::cout << "Number of points in PCL Cloud : " << MyPCLCloud.points.size()<< "\n";

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud <pcl::PointXYZRGB>);
        CloudT::Ptr cloud_filtered_2(new pcl::PointCloud <PointT>);
        cloud_filtered_2 = passThrough(-0.027, 0.015, MyPCLCloud.makeShared());  //this values a set so in order to cut from the image gripper and table

        std::cout << "Saving cloud_filtered.ply, cloud_filtered.pcd : " << MyPCLCloud.points.size()<< "\n";
        pcl::io::savePLYFile("../../images/cloud_filtered.ply", *cloud_filtered_2, true);
        pcl::io::savePCDFile("../../images/cloud_filtered.pcd", *cloud_filtered_2, true);
        std::cout << "done " << MyPCLCloud.points.size()<< "\n";


        // Clustering
        cloud_filtered_2 = voxelGrid(leaf_size, cloud_filtered_2);
        std::vector <pcl::PointIndices> cluster_indices = euclidian_clustering(cloud_filtered_2);

        // Finding two lowest cluster. Those should be the wires
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
             it != cluster_indices.end(); ++it) {
            CloudT::Ptr cloud_cluster(new CloudT);
            CloudT::Ptr boundingBox(new CloudT);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                cloud_cluster->points.push_back(cloud_filtered_2->points[*pit]);
            }
            int r = rand() % 255;
            int g = rand() % 255;
            int b = rand() % 255;
            for (PointT &p:*cloud_cluster) {
                p.r = r;
                p.g = g;
                p.b = b;
            }

            auto point_with_max_y_temporary = max_element(cloud_cluster->points.begin(), cloud_cluster->points.end(),
                                                         compare);

            if (y_max_1 < point_with_max_y_temporary->y) {
                y_max_2 = y_max_1;
                y_max_1 = point_with_max_y_temporary->y;
                wire_2 = wire_1;
                wire_1 = cloud_cluster;
            }
            if (y_max_2 < point_with_max_y_temporary->y and cloud_cluster->size() != wire_1->size()) {
                y_max_2 = point_with_max_y_temporary->y;
                wire_2 = cloud_cluster;
            }
        }
        *temp_clood_ptr += *cloud_filtered_2;
        *temp_clood_ptr += *wire_1;
        *temp_clood_ptr += *wire_2;
        std::cout << "y_max_1: " + std::to_string(y_max_1) << "\n";
        std::cout << "y_max_2: " + std::to_string(y_max_2) << "\n";


        // Opencv part
        // creating 3d and 2d images only with single wires
        cv::Mat image_3d, grayimage, wire_1_gray, wire_2_gray;
        convertToOpenCV(Frame, image_3d, grayimage);
        float threshold = atoi(threshold_str.c_str());
        std::cout << "Threshold: " + std::to_string(threshold) << "\n";

        // wire 1
        area_bounding_box(wire_1, min_point_AABB, max_point_AABB,
                          min_point_OBB, max_point_OBB, position_OBB,
                          rotational_matrix_OBB);

        pcl::CropBox <PointT> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z, 1.0));
        boxFilter.setMax(Eigen::Vector4f(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z, 1.0));
        boxFilter.setInputCloud(MyPCLCloud.makeShared());
        boxFilter.filter(*wire_1);
        grayimage.copyTo(wire_1_gray);
        cv::Mat wire_1_mat;
        leave_only_wire(image_3d, wire_1_gray, wire_1_mat, min_point_AABB, max_point_AABB, threshold);

        CloudT::Ptr wire_1_filtered(new CloudT);
        for (PointT &p:wire_1->points) {
            if (p.r >= threshold) {
                wire_1_filtered->push_back(p);
            }
        }
        std::cout << "Saving image_3d.exr, wire_1.exr, wire_1.jpg \n";
        wire_1 = wire_1_filtered;
        //pcl::io::savePLYFile("../../wire_1.ply", *wire_1, true);
        //pcl::io::savePCDFile("../../wire_1.pcd", *wire_1, true);
        cv::imwrite("../../images/image_3d.exr", image_3d);
        cv::imwrite("../../images/wire_1.exr", wire_1_mat);
        cv::imwrite("../../images/wire_1.jpg", wire_1_gray);
        std::cout << "done \n";
        if (task == "tcp_calibration") {
            std::cout << " breaking the loop in order to provide only one wire\n";
            //Send reply
            answer = std::string(threshold_str);
            reply = zmq::message_t(threshold_str.length());
            memcpy(reply.data(), threshold_str.data(), threshold_str.length());
            socket.send(reply, ZMQ_NOBLOCK);
            continue;
        }

        // wire 2
        area_bounding_box(wire_2, min_point_AABB, max_point_AABB,
                          min_point_OBB, max_point_OBB, position_OBB,
                          rotational_matrix_OBB);

        boxFilter.setMin(Eigen::Vector4f(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z, 1.0));
        boxFilter.setMax(Eigen::Vector4f(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z, 1.0));
        boxFilter.setInputCloud(MyPCLCloud.makeShared());
        boxFilter.filter(*wire_2);
        grayimage.copyTo(wire_2_gray);
        cv::Mat wire_2_mat;
        leave_only_wire(image_3d, wire_2_gray, wire_2_mat, min_point_AABB, max_point_AABB, threshold);
        CloudT::Ptr wire_2_filtered(new CloudT);

        for (PointT &p:wire_2->points) {
            if (p.r >= threshold) {

                wire_2_filtered->push_back(p);
            }
        }
        wire_2 = wire_2_filtered;
        std::cout << "Saving wires.exr, wires.jpg, wire_2.exr, wire_2.jpg \n";
        cv::imwrite("../../images/wire_2.exr", wire_2_mat);
        cv::imwrite("../../images/wire_2.jpg", wire_2_gray);
        cv::imwrite("../../images/wires.exr", wire_1_mat + wire_2_mat);
        cv::imwrite("../../images/wires.jpg", wire_1_gray + wire_2_gray);
        std::cout << "done \n";

        //Send reply
        answer = std::string(threshold_str);
        reply = zmq::message_t(threshold_str.length());
        memcpy(reply.data(), threshold_str.data(), threshold_str.length());
        socket.send(reply, ZMQ_NOBLOCK);
    }
    // Disconnect PhoXi device
    PhoXiDevice->Disconnect();
    return 0;
}

void convertToOpenCV(const pho::api::PFrame &Frame, cv::Mat &pointCloudMat, cv::Mat &graymat) {
    std::cout << "Frame " << Frame->Info.FrameIndex << "\n";
    if (!Frame->PointCloud.Empty()) {
        if (Frame->PointCloud.ConvertTo(pointCloudMat)) {
            cv::Point3f MiddlePoint = pointCloudMat.at<cv::Point3f>(
                    pointCloudMat.rows / 2, pointCloudMat.cols / 2);
            std::cout << "Middle point: " << MiddlePoint.x << "; "
                      << MiddlePoint.y << "; " << MiddlePoint.z << std::endl;


            for (int x_ = 0; x_ < pointCloudMat.cols; x_++) {
                for (int y_ = 0; y_ < pointCloudMat.rows; y_++) {
                    if (!pointCloudMat.empty()) {

                        if (
                                pointCloudMat.at<cv::Vec3f>(y_, x_)[1] < -0.04 ||
                                pointCloudMat.at<cv::Vec3f>(y_, x_)[1] > 0.015
                                ) {

                            pointCloudMat.at<cv::Vec3f>(y_, x_)[0] = 0;
                            pointCloudMat.at<cv::Vec3f>(y_, x_)[1] = 0;
                            pointCloudMat.at<cv::Vec3f>(y_, x_)[2] = 0;
                            pointCloudMat.at<float>(y_, x_) = 0;
                        } else {
                            continue;
                        }
                    }
                }
            }
        }

        if (Frame->Texture.ConvertTo(graymat)) {
            //double minVal;
            //double maxVal;
            //cv::Point minLoc;
            //cv::Point maxLoc;
           // cv::minMaxLoc(graymat, &minVal, &maxVal, &minLoc, &maxLoc);
            std::cout<<"Saving graymat.jpg \n";
            cv::imwrite("../../images/graymat.jpg", graymat);
            std::cout<<"done \n";
        }

    }
}

void leave_only_wire(cv::Mat &image_3d, cv::Mat &grayimage, cv::Mat &processed_image, PointT min_p, PointT max_p,
                     float threshold) {
    image_3d.copyTo(processed_image);
    for (int x_ = 0; x_ < processed_image.cols; x_++) {
        for (int y_ = 0; y_ < processed_image.rows; y_++) {
            if (!processed_image.empty()) {

                if (
                        image_3d.at<cv::Vec3f>(y_, x_)[0] < min_p.x ||
                        image_3d.at<cv::Vec3f>(y_, x_)[0] > max_p.x ||
                        image_3d.at<cv::Vec3f>(y_, x_)[1] < min_p.y ||
                        image_3d.at<cv::Vec3f>(y_, x_)[1] > max_p.y ||
                        image_3d.at<cv::Vec3f>(y_, x_)[2] < min_p.z ||
                        image_3d.at<cv::Vec3f>(y_, x_)[2] > max_p.z ||
                        grayimage.at<float>(y_, x_) < threshold) {

                    processed_image.at<cv::Vec3f>(y_, x_)[0] = 0;
                    processed_image.at<cv::Vec3f>(y_, x_)[1] = 0;
                    processed_image.at<cv::Vec3f>(y_, x_)[2] = 0;
                    grayimage.at<float>(y_, x_) = 0;
                } else {
                    continue;
                }
            }

        }
    }
    for (int x_ = 0; x_ < processed_image.cols; x_++) {
        for (int y_ = 0; y_ < processed_image.rows; y_++) {
            if (image_3d.at<cv::Vec3f>(y_, x_)[0] == 0 &&
                image_3d.at<cv::Vec3f>(y_, x_)[1] == 0 &&
                image_3d.at<cv::Vec3f>(y_, x_)[2] == 0) {
                grayimage.at<float>(y_, x_) = 0;
            }
        }
    }
}




