 #include <vision/point_cloud_manager.h>

using namespace XBot::HyperGraph::Utils;

PointCloudManager::PointCloudManager(std::string topic_name, ros::NodeHandle nh):
_nh(nh),
_nhpr("~"),
_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
_cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZRGB>),
_cloud_without_outliers(new pcl::PointCloud<pcl::PointXYZRGB>),
_cloud_planar_segmented(new pcl::PointCloud<pcl::PointXYZRGB>),
_isCallbackDone(false)
{
    // Subscribe to point cloud topic
    _pc_sub = _nh.subscribe(topic_name, 1, &PointCloudManager::callback, this);

    // Advertise object topic
    _obj_pub = _nh.advertise<teb_test::ObjectMessageString>("optimizer/obstacles", 10, true);
    _pc_voxel_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("voxel", 10, true);
    _pc_planar_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("planar", 10, true);
    _pc_outlier_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("outlier", 10, true);
    _ma_pub = _nh.advertise<visualization_msgs::MarkerArray>("markers", 10, true);

    // extract camera tf w.r.t. world frame (at the moment it handles static camera; easy upgrade moving
    // the lookupTransform in the callback)
    tf::TransformListener listener;
    try
    {
        listener.waitForTransform("world", "zedm_left_camera_frame", ros::Time::now(), ros::Duration(10.0));
        listener.lookupTransform("world", "zedm_left_camera_frame", ros::Time::now(), _transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    tf::transformTFToEigen(_transform, _w_T_cam);
}

void PointCloudManager::callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc (new pcl::PointCloud<pcl::PointXYZRGB>);
    pc = msg;
    pcl::transformPointCloud(*pc, *_point_cloud, _w_T_cam.matrix());

    _point_cloud->header.frame_id = "world";

    // Filter cloud from NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_point_cloud, *_point_cloud, indices);

    if(!_isCallbackDone)
        _isCallbackDone = true;
}


void PointCloudManager::clusterExtraction()
{
    if (_isCallbackDone)
    {
        // planar segmentation
        auto tic_segmentation = std::chrono::high_resolution_clock::now();
        planarSegmentation(_point_cloud, _cloud_planar_segmented);
        auto toc_segmentation = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration_segmentation = toc_segmentation - tic_segmentation;

        // voxel filtering
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(_cloud_planar_segmented);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud (_cloud_planar_segmented);
        vg.setLeafSize (0.01, 0.01, 0.01);
        auto tic_voxel = std::chrono::high_resolution_clock::now();
        vg.filter (*_cloud_voxel_filtered);
        auto toc_voxel = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration_voxel = toc_voxel - tic_voxel;

        // outliers statistical removal
        auto tic_outliers = std::chrono::high_resolution_clock::now();
        outlierRemoval(_cloud_voxel_filtered, _cloud_without_outliers);
        auto toc_outliers = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> duration_outliers = toc_outliers - tic_outliers;

        std::cout << "Requested time for perception: " << std::endl;
        std::cout << "voxel filtering: " << duration_voxel.count() << std::endl;
        std::cout << "planar segmentation: " << duration_segmentation.count() << std::endl;
        std::cout << "outliers removal: " << duration_outliers.count() << std::endl;

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.05);
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree);
        ec.setInputCloud(_cloud_without_outliers);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            // Extract i-th cluster from the scene_cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                cloud_cluster->push_back ((*_cloud_planar_segmented)[*pit]);
                cloud_cluster->push_back ((*_cloud_without_outliers)[*pit]);
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            cloud_cluster->header.frame_id = "world";

            double x_mean = 0, y_mean = 0 , z_mean = 0;
            for (auto point : cloud_cluster->points)
            {
                x_mean += point.x;
                y_mean += point.y;
                z_mean += point.z;
            }

            x_mean /= cloud_cluster->size();
            y_mean /= cloud_cluster->size();
            z_mean /= cloud_cluster->size();

            // This will broadcast a frame in the middle point of each cluster
            tf::Transform t;
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            t.setOrigin(tf::Vector3(x_mean, y_mean, z_mean));
            t.setRotation(q);
            _transforms.push_back(t);

            // Create a publisher
            ros::Publisher pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("object_"+std::to_string(j), 1, true);

            // push_back
            _cluster_cloud.push_back(cloud_cluster);
            _cc_pub.push_back(pub);

            j++;
        }
    generateObjectMsg();
    }
}

void PointCloudManager::outlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (input);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.8);
    sor.filter (*output);
}

void PointCloudManager::planarSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
    // perform ransac planar filtration to remove table top
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_before (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
      // Optional
      seg1.setOptimizeCoefficients (true);
      // Mandatory
      seg1.setModelType (pcl::SACMODEL_PLANE);
      seg1.setMethodType (pcl::SAC_RANSAC);
      seg1.setDistanceThreshold (0.01);

      seg1.setInputCloud (input);
      seg1.segment (* inliers, * coefficients);


      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;

      extract.setInputCloud (input);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (* output_before);

      removePointsBelowPlane(output_before, output, coefficients);
}

void PointCloudManager::removePointsBelowPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, pcl::ModelCoefficients::Ptr coefficients)
{
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    // TODO: this can be replaced by a std::for_each to speed up the code
    for (int i = 0; i < input->size(); i++)
    {
        double value = coefficients->values[0] * input->points[i].x + coefficients->values[1] * input->points[i].y + coefficients->values[2] * input->points[i].z + coefficients->values[3];
        if (value < 0)
            indices->indices.push_back(i);
    }

    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*output);
}

void PointCloudManager::generateObjectMsg()
{
    for (int i = 0 ; i < _transforms.size(); i++)
    {
        teb_test::ObjectMessage msg;
        msg.header.frame_id = "world";
        msg.header.seq = i;
        msg.header.stamp = ros::Time::now();

        msg.name.data = "object_" + std::to_string(i);

        msg.pose.position.x = _transforms[i].getOrigin()[0];
        msg.pose.position.y = _transforms[i].getOrigin()[1];
        msg.pose.position.z = _transforms[i].getOrigin()[2];

        // TODO: fix orientation!
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 1;

        // extract size of the bounding boxes
        std::vector<double> x_vec, y_vec, z_vec;
        for (auto point : _cluster_cloud[i]->points)
        {
            x_vec.push_back(point.x);
            y_vec.push_back(point.y);
            z_vec.push_back(point.z);
        }

        auto x_min = *std::min_element(x_vec.begin(), x_vec.end());
        auto y_min = *std::min_element(y_vec.begin(), y_vec.end());
        auto z_min = *std::min_element(z_vec.begin(), z_vec.end());

        auto x_max = *std::max_element(x_vec.begin(), x_vec.end());
        auto y_max = *std::max_element(y_vec.begin(), y_vec.end());
        auto z_max = *std::max_element(z_vec.begin(), z_vec.end());

        msg.size.x = x_max - x_min;
        msg.size.y = y_max - y_min;
        msg.size.z = z_max - z_min;

        _objects.objects.push_back(msg);
    }
}

void PointCloudManager::publishObjectMarkers()
{
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;

    for (auto object : _objects.objects)
    {
        m.header = object.header;
        m.id = object.header.seq;
        m.pose = object.pose;

        m.action = visualization_msgs::Marker::ADD;
        m.type = visualization_msgs::Marker::CUBE;

        m.scale = object.size;

        m.color.r = 1;  m.color.g = 0;  m.color.b = 0;  m.color.a = 0.3;

        ma.markers.push_back(m);
    }
    _ma_pub.publish(ma);
}

void PointCloudManager::run()
{
    _pc_pub.publish(_point_cloud);
    _transforms.clear();
    _cc_pub.clear();
    _cluster_cloud.clear();
    _objects.objects.clear();

    clusterExtraction();
    for (int i = 0; i < _cc_pub.size(); i++)
    {
        _cc_pub[i].publish(_cluster_cloud[i]);
        _obj_pub.publish(_objects);
        publishObjectMarkers();
        _broadcaster.sendTransform(tf::StampedTransform(_transforms[i], ros::Time::now(), "world", "object_" + std::to_string(i)));
    }

    _pc_voxel_pub.publish(_cloud_voxel_filtered);
    _pc_planar_pub.publish(_cloud_planar_segmented);
    _pc_outlier_pub.publish(_cloud_without_outliers);
}
