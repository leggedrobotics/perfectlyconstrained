
#include "pointmatcher_ros/transform.h"

// ros
#include <ros/ros.h>
#include <ros/common.h>

// eigen_conversions
#include <eigen_conversions/eigen_msg.h>

// tf
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "pointmatcher_ros/PmTf.h"


// ugly test depending on roscpp because tf_conversions is not properly versionized
#if !ROS_VERSION_MINIMUM(1, 9, 30)
#define transformTFToEigen TransformTFToEigen
#define transformEigenToTF TransformEigenToTF
#endif // !ROS_VERSION_MINIMUM(1, 9, 30)

namespace pointmatcher_ros
{
template<typename T>
typename PointMatcher<T>::TransformationParameters transformListenerToEigenMatrix(const tf::TransformListener& listener,
                                                                                  const std::string& target, const std::string& source,
                                                                                  const ros::Time& stamp)
{

    tf::StampedTransform stampedTr;
    listener.waitForTransform(target, source, stamp, ros::Duration(0.1));
    listener.lookupTransform(target, source, stamp, stampedTr);

    Eigen::Affine3d eigenTr;
    tf::transformTFToEigen(stampedTr, eigenTr);
    return eigenTr.matrix().cast<T>();
}

template PointMatcher<float>::TransformationParameters transformListenerToEigenMatrix<float>(const tf::TransformListener& listener,
                                                                                             const std::string& target,
                                                                                             const std::string& source,
                                                                                             const ros::Time& stamp);
template PointMatcher<double>::TransformationParameters transformListenerToEigenMatrix<double>(const tf::TransformListener& listener,
                                                                                               const std::string& target,
                                                                                               const std::string& source,
                                                                                               const ros::Time& stamp);


//--------------------------------
template<typename T>
typename PointMatcher<T>::TransformationParameters odomMsgToEigenMatrix(const nav_msgs::Odometry& odom)
{
    Eigen::Affine3d eigenTr;
    tf::poseMsgToEigen(odom.pose.pose, eigenTr);
    return eigenTr.matrix().cast<T>();
}

template PointMatcher<float>::TransformationParameters odomMsgToEigenMatrix<float>(const nav_msgs::Odometry& odom);
template PointMatcher<double>::TransformationParameters odomMsgToEigenMatrix<double>(const nav_msgs::Odometry& odom);


//--------------------------------
// Eigen to Odom
template<typename T>
nav_msgs::Odometry eigenMatrixToOdomMsg(const typename PointMatcher<T>::TransformationParameters& inTr, const std::string& frame_id,
                                        const ros::Time& stamp)
{
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;

    // Fill pose
    const Eigen::Affine3d eigenTr(Eigen::Matrix4d(eigenMatrixToDim<double>(inTr.template cast<double>(), 4)));
    tf::poseEigenToMsg(eigenTr, odom.pose.pose);

    // Fill velocity, TODO: find proper computation from delta poses to twist
    // odom.child_frame_id = cloudMsgIn.header.frame_id;
    odom.twist.covariance[0 + 0 * 6] = -1;
    odom.twist.covariance[1 + 1 * 6] = -1;
    odom.twist.covariance[2 + 2 * 6] = -1;
    odom.twist.covariance[3 + 3 * 6] = -1;
    odom.twist.covariance[4 + 4 * 6] = -1;
    odom.twist.covariance[5 + 5 * 6] = -1;

    return odom;
}

template nav_msgs::Odometry eigenMatrixToOdomMsg<float>(const PointMatcher<float>::TransformationParameters& inTr,
                                                        const std::string& frame_id, const ros::Time& stamp);
template nav_msgs::Odometry eigenMatrixToOdomMsg<double>(const PointMatcher<double>::TransformationParameters& inTr,
                                                         const std::string& frame_id, const ros::Time& stamp);


//--------------------------------
// Pose to Eigen
template<typename T>
typename PointMatcher<T>::TransformationParameters poseMsgToEigenMatrix(const geometry_msgs::Pose& pose)
{
    Eigen::Affine3d eigenTr;
    tf::poseMsgToEigen(pose, eigenTr);
    return eigenTr.matrix().cast<T>();
}

template PointMatcher<float>::TransformationParameters poseMsgToEigenMatrix<float>(const geometry_msgs::Pose& pose);
template PointMatcher<double>::TransformationParameters poseMsgToEigenMatrix<double>(const geometry_msgs::Pose& pose);


//--------------------------------
// Eigen to Pose
template<typename T>
geometry_msgs::Pose eigenMatrixToPoseMsg(const typename PointMatcher<T>::TransformationParameters& inTr)
{
    geometry_msgs::Pose pose;

    // Fill pose
    const Eigen::Affine3d eigenTr(Eigen::Matrix4d(eigenMatrixToDim<double>(inTr.template cast<double>(), 4)));
    tf::poseEigenToMsg(eigenTr, pose);


    return pose;
}

template geometry_msgs::Pose eigenMatrixToPoseMsg<float>(const PointMatcher<float>::TransformationParameters& inTr);
template geometry_msgs::Pose eigenMatrixToPoseMsg<double>(const PointMatcher<double>::TransformationParameters& inTr);


//--------------------------------
template<typename T>
tf::Transform eigenMatrixToTransform(const typename PointMatcher<T>::TransformationParameters& inTr)
{
    tf::Transform tfTr;
    const Eigen::Affine3d eigenTr(Eigen::Matrix4d(eigenMatrixToDim<double>(inTr.template cast<double>(), 4)));
    tf::transformEigenToTF(eigenTr, tfTr);
    return tfTr;
}

template tf::Transform eigenMatrixToTransform<float>(const PointMatcher<float>::TransformationParameters& inTr);
template tf::Transform eigenMatrixToTransform<double>(const PointMatcher<double>::TransformationParameters& inTr);


//--------------------------------
template<typename T>
tf::StampedTransform eigenMatrixToStampedTransform(const typename PointMatcher<T>::TransformationParameters& inTr,
                                                   const std::string& target, const std::string& source, const ros::Time& stamp)
{
    return tf::StampedTransform(eigenMatrixToTransform<T>(inTr), stamp, target, source);
}

template tf::StampedTransform eigenMatrixToStampedTransform<float>(const PointMatcher<float>::TransformationParameters& inTr,
                                                                   const std::string& target, const std::string& source,
                                                                   const ros::Time& stamp);
template tf::StampedTransform eigenMatrixToStampedTransform<double>(const PointMatcher<double>::TransformationParameters& inTr,
                                                                    const std::string& target, const std::string& source,
                                                                    const ros::Time& stamp);


//--------------------------------
template<typename T>
typename PointMatcher<T>::TransformationParameters eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix,
                                                                    int dimp1)
{
    typedef typename PointMatcher<T>::TransformationParameters M;
    assert(matrix.rows() == matrix.cols());
    assert((matrix.rows() == 3) || (matrix.rows() == 4));
    assert((dimp1 == 3) || (dimp1 == 4));

    if (matrix.rows() == dimp1)
        return matrix;

    M out(M::Identity(dimp1, dimp1));
    out.topLeftCorner(2, 2) = matrix.topLeftCorner(2, 2);
    out.topRightCorner(2, 1) = matrix.topRightCorner(2, 1);
    return out;
}

bool transformCloudToFrame(const std::string& fixedFrame, const std::string& targetFrame, const ros::Time& targetStamp,
                           const tf2_ros::Buffer& tfBuffer, StampedPointCloud& pointCloud, const double waitTimeTfLookup)
{
    const auto& sourceFrame{ pointCloud.header_.frame_id };
    const auto& sourceStamp{ pointCloud.header_.stamp };

    if (sourceFrame.empty())
    {
        ROS_WARN("Point cloud could not be transformed to target frame due to missing source frame.");
        return false;
    }
    if (targetFrame.empty())
    {
        ROS_WARN("Point cloud could not be transformed to target frame due to missing target frame.");
        return false;
    }
    if (pointCloud.isEmpty())
    {
        ROS_WARN("Point cloud could not be transformed to target frame as it contains no data.");
        return false;
    }

    if (sourceFrame == targetFrame && sourceStamp == targetStamp)
    {
        ROS_DEBUG("Early return when transforming point cloud as it is already in the target frame at the expected time.");
        return true;
    }

    if (!tfBuffer.canTransform(targetFrame, targetStamp, sourceFrame, sourceStamp, fixedFrame, ros::Duration(waitTimeTfLookup)))
    {
        ROS_WARN_STREAM("Requested transform from frames \'" + sourceFrame + "' to '" + targetFrame + "', with target timestamp "
                        << targetStamp << " cannot be found.");
        return false;
    }

    geometry_msgs::TransformStamped tfSourceToTargetRos;
    try
    {
        // Lookup tf from source frame to target frame, using an intermediate fixed frame
        tfSourceToTargetRos =
            tfBuffer.lookupTransform(targetFrame, targetStamp, sourceFrame, sourceStamp, fixedFrame, ros::Duration(waitTimeTfLookup));
    }
    catch (const tf2::TransformException& exception)
    {
        ROS_DEBUG_STREAM("Caught an exception while looking up transformation: " << exception.what());
        return false;
    }

    // Transform point cloud from source frame to target frame.
    PmTf tfSourceToTarget = PmTf::FromRosTfMsg(tfSourceToTargetRos);
    pointCloud.transform(tfSourceToTarget);

    return true;
}


template PointMatcher<float>::TransformationParameters eigenMatrixToDim<float>(const PointMatcher<float>::TransformationParameters& matrix,
                                                                               int dimp1);
template PointMatcher<double>::TransformationParameters eigenMatrixToDim<double>(
    const PointMatcher<double>::TransformationParameters& matrix, int dimp1);
} // namespace pointmatcher_ros