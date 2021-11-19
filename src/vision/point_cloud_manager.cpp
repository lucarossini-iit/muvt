 #include <vision/point_cloud_manager.h>

using namespace XBot::HyperGraph::Utils;

PointCloudManager::PointCloudManager(std::string topic_name, ros::NodeHandle nh):
_nh(nh),
_nhpr("~"),
_point_cloud(new pcl::PointCloud<pcl::PointXYZ>),
_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),
_cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>),
_isCallbackDone(false)
{
    // Subscribe to point cloud topic
    _pc_sub = _nh.subscribe(topic_name, 1, &PointCloudManager::callback, this);

    // Advertise object topic
    _obj_pub = _nh.advertise<teb_test::ObjectMessageString>("objects", 10, true);
    _pc_filt_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("voxel", 10, true);
    _pc_filt2_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("outlier", 10, true);
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

void PointCloudManager::callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    pc = msg;
    pcl::transformPointCloud(*pc, *_point_cloud, _w_T_cam.matrix());

    std::cout << _point_cloud->points[500].x << " " << _point_cloud->points[500].y << " " << _point_cloud->points[500].z << std::endl;

//    _point_cloud->header.frame_id = "world";

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
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(_point_cloud);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (_point_cloud);
        vg.setLeafSize (0.02f, 0.02f, 0.02f);
        vg.filter (*_cloud_filtered);

        // outliers removal
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
        outlierRemoval(_cloud_filtered, _cloud_filtered2);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.05);
        ec.setMinClusterSize (50);
        ec.setMaxClusterSize (500);
        ec.setSearchMethod (tree);
        ec.setInputCloud (_cloud_filtered2);
        auto tic = std::chrono::high_resolution_clock::now();
        ec.extract (cluster_indices);
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc - tic;

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            // Extract i-th cluster from the scene_cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->push_back ((*_cloud_filtered2)[*pit]); //*
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            cloud_cluster->header.frame_id = "zedm_left_camera_frame";

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
            ros::Publisher pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("object_"+std::to_string(j), 1, true);

            // push_back
            _cluster_cloud.push_back(cloud_cluster);
            _cc_pub.push_back(pub);

            j++;
        }
    generateObjectMsg();
    }
}

void PointCloudManager::outlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input);
    sor.setMeanK (25);
    sor.setStddevMulThresh (1.);
    sor.filter (*output);
}

void PointCloudManager::generateObjectMsg()
{
    for (int i = 0 ; i < _transforms.size(); i++)
    {
        teb_test::ObjectMessage msg;
        msg.header.frame_id = "zedm_left_camera_frame";
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
        _broadcaster.sendTransform(tf::StampedTransform(_transforms[i], ros::Time::now(), "zedm_left_camera_frame", "object_" + std::to_string(i)));
    }

    _pc_filt_pub.publish(_cloud_filtered);
    _pc_filt2_pub.publish(_cloud_filtered2);
}
