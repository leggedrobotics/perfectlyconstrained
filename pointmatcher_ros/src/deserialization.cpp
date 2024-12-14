#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include "pointmatcher_ros/deserialization.h"

// std
#include <memory>
#include <vector>

// boost
#include <boost/algorithm/string.hpp>

// pointmatcher
#include <pointmatcher/IO.h>

namespace pointmatcher_ros
{
//! Transform a ROS PointCloud2 message into a libpointmatcher point cloud
template<typename T>
typename PointMatcher<T>::DataPoints rosMsgToPointMatcherCloud(const sensor_msgs::PointCloud2& rosMsg, const bool isDense)
{

    // FIXME: continue from here, need to decode time properly
    typedef PointMatcher<T> PM;
    typedef PointMatcherIO<T> PMIO;
    typedef typename PMIO::PMPropTypes PM_types;
    typedef typename PM::DataPoints DataPoints;
    typedef typename DataPoints::Label Label;
    typedef typename DataPoints::Labels Labels;
    typedef typename DataPoints::View View;
    typedef typename DataPoints::TimeView TimeView;

    if (rosMsg.fields.empty())
        return DataPoints();


    // fill labels
    // conversions of descriptor fields from pcl
    // see http://www.ros.org/wiki/pcl/Overview
    Labels featLabels;
    Labels descLabels;
    Labels timeLabels;
    std::vector<bool> isFeature;
    std::vector<PM_types> fieldTypes;
    for (auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it)
    {
        const std::string name(it->name);
        const size_t count(std::max<size_t>(it->count, 1));
        if (name == "x" || name == "y" || name == "z")
        {
            featLabels.push_back(Label(name, count));
            isFeature.push_back(true);
            fieldTypes.push_back(PM_types::FEATURE);
        }
        else if (name == "rgb" || name == "rgba")
        {
            descLabels.push_back(Label("color", (name == "rgba") ? 4 : 3));
            isFeature.push_back(false);
        }
        else if ((it + 1) != rosMsg.fields.end() && it->name == "normal_x" && (it + 1)->name == "normal_y")
        {
            if ((it + 2) != rosMsg.fields.end() && (it + 2)->name == "normal_z")
            {
                descLabels.push_back(Label("normals", 3));
                it += 2;
                isFeature.push_back(false);
                isFeature.push_back(false);
                fieldTypes.push_back(PM_types::DESCRIPTOR);
                fieldTypes.push_back(PM_types::DESCRIPTOR);
            }
            else
            {
                descLabels.push_back(Label("normals", 2));
                it += 1;
                isFeature.push_back(false);
                fieldTypes.push_back(PM_types::DESCRIPTOR);
            }
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::DESCRIPTOR);
        }
        else if ((it + 1) != rosMsg.fields.end() && boost::algorithm::ends_with(name, "_splitTime_high32")
                 && boost::algorithm::ends_with(((it + 1)->name), "_splitTime_low32"))
        {
            // time extraction
            // const std::string beginning = name.substr(0, name.size()-4);
            std::string startingName = name;
            boost::algorithm::erase_last(startingName, "_splitTime_high32");
            const std::string beginning = startingName;

            timeLabels.push_back(Label(beginning, 1));
            it += 1;
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::TIME);
            fieldTypes.push_back(PM_types::TIME);
        }
        // else if (name == "stamps")
        else if (name == "time")
        {
            // timeLabels.push_back(Label(name, count));
            // fieldTypes.push_back(PM_types::TIME);
            // TODO(Perception) Check if this is the best way to treat time point descriptors. Upstream libpointmatcher's design is
            // designed differently.
            descLabels.push_back(Label(name, count));
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::DESCRIPTOR);
        }
        else
        {
            descLabels.push_back(Label(name, count));
            isFeature.push_back(false);
            fieldTypes.push_back(PM_types::DESCRIPTOR);
        }
    }

    featLabels.push_back(Label("pad", 1));
    assert(isFeature.size() == rosMsg.fields.size());
    assert(fieldTypes.size() == rosMsg.fields.size());

    // create cloud
    const unsigned pointCount(rosMsg.width * rosMsg.height);
    DataPoints cloud(featLabels, descLabels, timeLabels, pointCount);
    cloud.getFeatureViewByName("pad").setConstant(1);

    // fill cloud
    // TODO: support big endian, pass through endian-swapping method just after the *reinterpret_cast
    typedef sensor_msgs::PointField PF;
    size_t fieldId = 0;
    for (auto it(rosMsg.fields.begin()); it != rosMsg.fields.end(); ++it, ++fieldId)
    {
        if (it->name == "rgb" || it->name == "rgba")
        {
            // special case for colors
            if (((it->datatype != PF::UINT32) && (it->datatype != PF::INT32) && (it->datatype != PF::FLOAT32)) || (it->count != 1))
                throw std::runtime_error(
                    (boost::format("Colors in a point cloud must be a single element of size 32 bits, found %1% elements of type %2%")
                     % it->count % unsigned(it->datatype))
                        .str());
            View view(cloud.getDescriptorViewByName("color"));
            int ptId(0);
            for (size_t y(0); y < rosMsg.height; ++y)
            {
                const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
                for (size_t x(0); x < rosMsg.width; ++x)
                {
                    const uint32_t rgba(*reinterpret_cast<const uint32_t*>(dataPtr + it->offset));
                    const T colorA(T((rgba >> 24) & 0xff) / 255.);
                    const T colorR(T((rgba >> 16) & 0xff) / 255.);
                    const T colorG(T((rgba >> 8) & 0xff) / 255.);
                    const T colorB(T((rgba >> 0) & 0xff) / 255.);
                    view(0, ptId) = colorR;
                    view(1, ptId) = colorG;
                    view(2, ptId) = colorB;
                    if (view.rows() > 3)
                        view(3, ptId) = colorA;
                    dataPtr += rosMsg.point_step;
                    ptId += 1;
                }
            }
        }
        else if (boost::algorithm::ends_with(it->name, "_splitTime_high32") || boost::algorithm::ends_with(it->name, "_splitTime_low32"))
        {
            std::string startingName = it->name;
            bool isHigh = false;
            if (boost::algorithm::ends_with(it->name, "_splitTime_high32"))
            {
                boost::algorithm::erase_last(startingName, "_splitTime_high32");
                isHigh = true;
            }
            if (boost::algorithm::ends_with(it->name, "_splitTime_low32"))
            {
                boost::algorithm::erase_last(startingName, "_splitTime_low32");
            }


            TimeView timeView(cloud.getTimeViewByName(startingName));
            // use view to read data

            int ptId(0);
            const size_t count(std::max<size_t>(it->count, 1));
            for (size_t y(0); y < rosMsg.height; ++y)
            {
                const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
                for (size_t x(0); x < rosMsg.width; ++x)
                {
                    const uint8_t* fPtr(dataPtr + it->offset);
                    for (unsigned dim(0); dim < count; ++dim)
                    {
                        if (isHigh)
                        {
                            const uint32_t high32 = *reinterpret_cast<const uint32_t*>(fPtr);
                            const uint32_t low32 = uint32_t(timeView(dim, ptId));
                            timeView(dim, ptId) = (((uint64_t)high32) << 32) | ((uint64_t)low32);
                        }
                        else
                        {
                            const uint32_t high32 = uint32_t(timeView(dim, ptId) >> 32);
                            const uint32_t low32 = *reinterpret_cast<const uint32_t*>(fPtr);
                            timeView(dim, ptId) = (((uint64_t)high32) << 32) | ((uint64_t)low32);
                        }
                        dataPtr += rosMsg.point_step;
                        ptId += 1;
                    }
                }
            }
        }
        // FIXME: this might never be used
        // else if (it->name == "stamps")
        //{
        //  ROS_INFO("!!!!!!!!!!!! GOT MSG WITH STAMPS FIELD !!!!!!!!!!!!!!!!");

        //  TimeView timeView(cloud.getTimeViewByName("stamps"));

        //  // use view to read data
        //  int ptId(0);
        //  const size_t count(std::max<size_t>(it->count, 1));
        //  for (size_t y(0); y < rosMsg.height; ++y)
        //  {
        //    const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
        //    for (size_t x(0); x < rosMsg.width; ++x)
        //    {
        //      const uint8_t* fPtr(dataPtr + it->offset);
        //      for (unsigned dim(0); dim < count; ++dim)
        //      {
        //        switch (it->datatype)
        //        {
        //          case PF::INT8:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const int8_t*>(fPtr));
        //            fPtr += 1;
        //            ROS_INFO("!!!!!!!!!!!! CASE INT8 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::UINT8:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const uint8_t*>(fPtr));
        //            fPtr += 1;
        //            ROS_INFO("!!!!!!!!!!!! CASE UINT8 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::INT16:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const int16_t*>(fPtr));
        //            fPtr += 2;
        //            ROS_INFO("!!!!!!!!!!!! CASE INT16 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::UINT16:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const uint16_t*>(fPtr));
        //            fPtr += 2;
        //            ROS_INFO("!!!!!!!!!!!! CASE UINT16 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::INT32:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const int32_t*>(fPtr));
        //            fPtr += 4;
        //            ROS_INFO("!!!!!!!!!!!! CASE INT32 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::UINT32:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const uint32_t*>(fPtr));
        //            fPtr += 4;
        //            ROS_INFO("!!!!!!!!!!!! CASE UINT32 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::FLOAT32:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const float*>(fPtr));
        //            fPtr += 4;
        //            //ROS_INFO("!!!!!!!!!!!! CASE FLOAT32 !!!!!!!!!!!!!!!!");
        //            break;
        //          case PF::FLOAT64:
        //            timeView(dim, ptId) = boost::uint64_t(*reinterpret_cast<const double*>(fPtr));
        //            fPtr += 8;
        //            ROS_INFO("!!!!!!!!!!!! CASE FLOAT64 !!!!!!!!!!!!!!!!");
        //            break;
        //          default: abort();
        //        }
        //      }
        //      dataPtr += rosMsg.point_step;
        //      ptId += 1;
        //    }
        //  }
        //}
        else
        {

            // get view for editing data
            View view((it->name == "normal_x")
                          ? cloud.getDescriptorRowViewByName("normals", 0)
                          : ((it->name == "normal_y")
                                 ? cloud.getDescriptorRowViewByName("normals", 1)
                                 : ((it->name == "normal_z") ? cloud.getDescriptorRowViewByName("normals", 2)
                                                             : ((isFeature[fieldId]) ? cloud.getFeatureViewByName(it->name)
                                                                                     : cloud.getDescriptorViewByName(it->name)))));

            // use view to read data
            int ptId(0);
            const size_t count(std::max<size_t>(it->count, 1));
            for (size_t y(0); y < rosMsg.height; ++y)
            {
                const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step * y);
                for (size_t x(0); x < rosMsg.width; ++x)
                {
                    const uint8_t* fPtr(dataPtr + it->offset);
                    for (unsigned dim(0); dim < count; ++dim)
                    {
                        switch (it->datatype)
                        {
                            case PF::INT8:
                                view(dim, ptId) = T(*reinterpret_cast<const int8_t*>(fPtr));
                                fPtr += 1;
                                break;
                            case PF::UINT8:
                                view(dim, ptId) = T(*reinterpret_cast<const uint8_t*>(fPtr));
                                fPtr += 1;
                                break;
                            case PF::INT16:
                                view(dim, ptId) = T(*reinterpret_cast<const int16_t*>(fPtr));
                                fPtr += 2;
                                break;
                            case PF::UINT16:
                                view(dim, ptId) = T(*reinterpret_cast<const uint16_t*>(fPtr));
                                fPtr += 2;
                                break;
                            case PF::INT32:
                                view(dim, ptId) = T(*reinterpret_cast<const int32_t*>(fPtr));
                                fPtr += 4;
                                break;
                            case PF::UINT32:
                                view(dim, ptId) = T(*reinterpret_cast<const uint32_t*>(fPtr));
                                fPtr += 4;
                                break;
                            case PF::FLOAT32:
                                view(dim, ptId) = T(*reinterpret_cast<const float*>(fPtr));
                                fPtr += 4;
                                break;
                            case PF::FLOAT64:
                                view(dim, ptId) = T(*reinterpret_cast<const double*>(fPtr));
                                fPtr += 8;
                                break;
                            default: abort();
                        }
                    }
                    dataPtr += rosMsg.point_step;
                    ptId += 1;
                }
            }
        }
    }


    if (isDense == false)
    {
        std::shared_ptr<typename PM::DataPointsFilter> filter(PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter"));
        return filter->filter(cloud);
    }

    return cloud;
}

template PointMatcher<float>::DataPoints rosMsgToPointMatcherCloud<float>(const sensor_msgs::PointCloud2& rosMsg, const bool isDense);
template PointMatcher<double>::DataPoints rosMsgToPointMatcherCloud<double>(const sensor_msgs::PointCloud2& rosMsg, const bool isDense);

} // namespace pointmatcher_ros