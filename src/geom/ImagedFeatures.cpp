#include <geom/ImagedFeatures.h>

namespace geom
{
    ImagedFeatures::ImagedFeatures(const std::string im_fname_, uint32_t seq_, cv::Ptr<cv::Feature2D> feat_detector) : im_fname(im_fname_), seq(seq_)
    {
        if (feat_detector.empty())
        {
            printf("[ERROR] Uninitialized Feature Detector\n");
            return;
        }

        im = cv::imread(im_fname.c_str(), cv::IMREAD_GRAYSCALE);

        if (im.empty())
        {
            printf("[ERROR] Cannot open file=%s\n", im_fname.c_str());
            return;
        }

        feat_detector->detectAndCompute(im, cv::noArray(), kpts, desc);

        for (uint32_t i = 0; i < kpts.size(); i++)
        {
            kpts_idx.push_back(FeatIdx(seq, i));
        }
    }

    uint32_t ImagedFeatures::DetectedFeaturesCount() const
    {
        return kpts.size();
    }

    void ImagedFeatures::PrintInfo(const uint8_t verbosity) const
    {

        printf("Seq#%u ; ", Seq());
        printf("im.%s ; ", helpers::MiscUtils::cvmat_info(im).c_str());

        if (verbosity > 0)
        {
            printf("n_feats: %u ; desc_size: %u %u", DetectedFeaturesCount(), desc.rows, desc.cols);
            printf("fname: %s ; ", im_fname.c_str());
        }
        printf("\n");
    }
}