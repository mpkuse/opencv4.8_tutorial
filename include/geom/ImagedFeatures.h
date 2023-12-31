#pragma once

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/features2d.hpp>

#include <iostream>
#include <vector>

#include <helpers/MiscUtils.h>

namespace geom
{
    class FeatIdx
    {
    public:
        FeatIdx(const uint32_t seq_, const uint32_t idx_) : seq(seq_), idx(idx_) {}
        const std::string Str() const { return "{" + std::to_string(seq) + "," + std::to_string(idx) + "}"; }

    private:
        const uint32_t seq;
        const uint32_t idx;
    };

    class ImagedFeatures
    {

    public:
        ImagedFeatures(const std::string im_fname, uint32_t seq_, cv::Ptr<cv::Feature2D> feat_detector);

        void PrintInfo() const;
        uint32_t DetectedFeaturesCount() const;

        const cv::Mat Image() const { return im; }
        const cv::Mat KeyptsDescriptor() const { return desc; }
        const std::vector<cv::KeyPoint> Keypts() const { return kpts; }
        const std::vector<geom::FeatIdx> KeyptsIdx() const { return kpts_idx; }

        const uint32_t Seq() const { return seq; }
        const std::string FileName() const { return im_fname; }

    private:
        const std::string im_fname;
        uint32_t seq;
        cv::Mat im;
        std::vector<cv::KeyPoint> kpts;
        cv::Mat desc;
        std::vector<FeatIdx> kpts_idx;
    };

} // geom