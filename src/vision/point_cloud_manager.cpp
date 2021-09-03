 #include <vision/point_cloud_manager.h>

using namespace XBot::HyperGraph::Utils;

PointCloudManager::PointCloudManager(std::string topic_name, ros::NodeHandle nh):
_nh(nh),
_nhpr("~"),
_point_cloud(new pcl::PointCloud<pcl::PointXYZ>),
_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>),
_isCallbackDone(false)
{
    // Get Parameters
    std::string extractorType;
    if (!_nhpr.getParam("extractorType", extractorType))
        ROS_ERROR("Missing mandatory parameter 'extractorType'!");

    if (extractorType == "3d_object")
        _extractor_type = ExtractorType::OBJECT;
    else if (extractorType == "euclidean_cluster")
        _extractor_type = ExtractorType::EUCLIDEANCLUSTER;

    // Subscribe to point cloud topic
    _pc_sub = _nh.subscribe(topic_name, 1, &PointCloudManager::callback, this);
}

PointCloudManager::PointCloudManager(Eigen::Vector3d pos, ros::NodeHandle nh):
_nh(nh),
_nhpr("~"),
_point_cloud(new pcl::PointCloud<pcl::PointXYZ>),
_scene_cloud(new pcl::PointCloud<pcl::PointXYZ>),
_isCallbackDone(false)
{
    // Advertise PointCloud topic
    _sc_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("scene_cloud", 1, true);
    _mc_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("model_cloud", 1, true);

    // Get Parameters
    std::string extractorType;
    if (!_nhpr.getParam("extractorType", extractorType))
        ROS_ERROR("Missing mandatory parameter 'extractorType'!");

    if (extractorType == "3d_object")
        _extractor_type = ExtractorType::OBJECT;
    else if (extractorType == "euclidean_cluster")
        _extractor_type = ExtractorType::EUCLIDEANCLUSTER;

    // Create PointCloud
    double size = 0.2;
    double res = 0.01;
    unsigned int index = 0;

    _point_cloud->resize(10000);
    _scene_cloud->resize(100000);

    // front face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2;
            _point_cloud->points[index].y = pos(1) - size/2 + i*res;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // back face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) + size/2;
            _point_cloud->points[index].y = pos(1) - size/2 + i*res;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // left face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) - size/2;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // right face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) + size/2;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // top face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) - size/2 + k*res;
            _point_cloud->points[index].z = pos(2) + size/2;
            index++;
        }
    }

    // bottom face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) - size/2 + k*res;
            _point_cloud->points[index].z = pos(2) - size/2;
            index++;
        }
    }

    _point_cloud->header.frame_id = "world";

    // add ground to the scene cloud
    *_scene_cloud = *_point_cloud;
    for (auto &point : _scene_cloud->points)
        point.x += 2.0;
    Eigen::Vector3d ground_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
    double ground_res = 0.01;
    double ground_size = 1.0;
    for (int k = 0; k <= ground_size/ground_res; k++)
    {
        for (int i = 0; i <= ground_size/ground_res; i++)
        {
            _scene_cloud->points[index].x = ground_pos(0) - ground_size/2 + i*ground_res;
            _scene_cloud->points[index].y = ground_pos(1) - ground_size/2 + k*ground_res;
            _scene_cloud->points[index].z = -1.0;
            index++;
        }
    }

    _isCallbackDone = true;
}

void PointCloudManager::callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    _scene_cloud = msg;
    _scene_cloud->header.frame_id = "world";

    // Filter cloud from NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_scene_cloud, *_scene_cloud, indices);

    if(!_isCallbackDone)
        _isCallbackDone = true;
}

void PointCloudManager::extractObject()
{
    if (_isCallbackDone)
    {
        std::cout << "starting extracting objects..." << std::endl;

        // extract normals from both the scene and model
        pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
        norm_est.setKSearch(10);
        norm_est.setInputCloud(_point_cloud);
        norm_est.compute(*model_normals);

        norm_est.setInputCloud(_scene_cloud);
        norm_est.compute(*scene_normals);

        // downsample the cloud in order to find a small number of keypoints used for 3D object recognition
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
        uniform_sampling.setInputCloud(_point_cloud);
        uniform_sampling.setRadiusSearch(0.05);
        uniform_sampling.filter(*model_keypoints);

        uniform_sampling.setInputCloud(_scene_cloud);
        uniform_sampling.filter(*scene_keypoints);

        // associate a 3D descriptor to the point cloud
        pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors (new pcl::PointCloud<pcl::SHOT352>);
        pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors (new pcl::PointCloud<pcl::SHOT352>);
        pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
        descr_est.setRadiusSearch(0.05);

        descr_est.setInputCloud(model_keypoints);
        descr_est.setInputNormals(model_normals);
        descr_est.setSearchSurface(_point_cloud);
        descr_est.compute(*model_descriptors);

        descr_est.setInputCloud(scene_keypoints);
        descr_est.setInputNormals(scene_normals);
        descr_est.setSearchSurface(_scene_cloud);
        descr_est.compute(*scene_descriptors);

        // determine point to point correspondences between model and scene descriptors
        pcl::KdTreeFLANN<pcl::SHOT352> match_search;
        match_search.setInputCloud(model_descriptors);
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences());

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
        {
            std::vector<int> neigh_indices (1);
            std::vector<float> neigh_sqr_dists (1);
            if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
            {
                continue;
            }
            int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
            if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            {
                pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back (corr);
            }
        }
        std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());

        pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (0.05);

        rf_est.setInputCloud (model_keypoints);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (_point_cloud);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (_scene_cloud);
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
        clusterer.setHoughBinSize (0.05);
        clusterer.setHoughThreshold (5.0);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (model_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;
        clusterer.recognize (rototranslations, clustered_corrs);

        std::cout << "RESULTS: " << std::endl;

        for (auto T : rototranslations)
        {
            auto rotation = T.block<3,3>(0,0);
            auto translation = T.block<3,1>(0,3);

            std::cout << "rotation: \n" << rotation << std::endl;
            std::cout << "translation: \n" << translation.transpose() << std::endl;
        }
    }
}

void PointCloudManager::clusterExtraction()
{
    if (_isCallbackDone)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(_scene_cloud);

        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (_scene_cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (10000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
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
                cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
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
//            tf::Transform t;
//            tf::Quaternion q;
//            q.setRPY(0, 0, 0);
//            t.setOrigin(tf::Vector3(x_mean, y_mean, z_mean));
//            t.setRotation(q);
//            _transform.push_back(t);

            // Create a publisher
            ros::Publisher pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("object_"+std::to_string(j), 1, true);

            // push_back
            _cluster_cloud.push_back(cloud_cluster);
            _cc_pub.push_back(pub);

            j++;
        }

        // This will create an oriented bounding box around each cluster
        computeBoundingBoxes(_cluster_cloud);
    }
}

void PointCloudManager::computeBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> inputCloud)
{
    for (auto cloud : inputCloud)
    {
        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                        ///    the signs are different and the box doesn't get correctly oriented in some cases.
        /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud);
        pca.project(*cloudSegmented, *cloudPCAprojection);
        std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
        std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
        // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
        */

        /// These eigenvectors are used to transform the point cloud to the origin point (0, 0, 0)
        /// such that the eigenvectors correspond to the axes of the space. The minimum point, maximum point,
        /// and the middle of the diagonal between these two points are calculated for the transformed cloud
        /// (also referred to as the projected cloud when using PCL's PCA interface).
        ///
        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        /// Compute the quaternion from the eigenvectors to re-orient the bounding box
        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
        const Eigen::Quaterniond bboxQuaterniond(bboxQuaternion);
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

        tf::Transform t;
        tf::Quaternion q;
        tf::quaternionEigenToTF(bboxQuaterniond, q);
        t.setOrigin(tf::Vector3(bboxTransform(0), bboxTransform(1), bboxTransform(2)));
        t.setRotation(q);
        _transform.push_back(t);
    }
}

void PointCloudManager::run()
{
    _sc_pub.publish(_scene_cloud);
    _mc_pub.publish(_point_cloud);
    _transform.clear();
    _cc_pub.clear();
    _cluster_cloud.clear();

    if (_extractor_type == ExtractorType::OBJECT)
    {
        extractObject();
//        _broadcaster.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "world", "object"));
    }
    else if (_extractor_type == ExtractorType::EUCLIDEANCLUSTER)
    {
        clusterExtraction();
        for (int i = 0; i < _cc_pub.size(); i++)
            _cc_pub[i].publish(_cluster_cloud[i]);
        for (int i = 0; i < _transform.size(); i++)
            _broadcaster.sendTransform(tf::StampedTransform(_transform[i], ros::Time::now(), "zedm_left_camera_frame", "object_" + std::to_string(i)));
    }
}
