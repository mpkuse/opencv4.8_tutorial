#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/features2d.hpp>

#include <iostream>
#include <vector>

#include <helpers/MiscUtils.h>
#include <helpers/TermColor.h>
#include <helpers/ElapsedTime.h>
#include <helpers/DisjointSet.h>

#include <geom/ImagedFeatures.h>
#include <geom/PairMatcher.h>
#include <geom/TwoViewGeometry.h>

TEST(View3d, FeatureTrackCreation)
{
    // file list
    const std::string base = "../data/SFMedu/";
    std::vector<std::string> image_list;
    image_list.push_back(base + "/B21.jpg");
    image_list.push_back(base + "/B22.jpg");
    image_list.push_back(base + "/B23.jpg");
    image_list.push_back(base + "/B24.jpg");
    image_list.push_back(base + "/B25.jpg");

    // load all the images and extract feat
    cv::Ptr<cv::Feature2D> feat_detector = cv::SIFT::create();
    std::vector<geom::ImagedFeatures> frame_list;
    for (uint32_t i = 0u; i < image_list.size(); i++)
    {
        const auto frame = geom::ImagedFeatures(image_list.at(i), i, feat_detector);
        frame_list.push_back(frame);
        frame.PrintInfo();
    }

    // pairwise matching between each pair
    std::vector<geom::PairMatcher2> pair_list;
    for (size_t i = 0u; i < frame_list.size(); i++)
    {
        for (size_t j = i + 1; j < frame_list.size(); j++)
        {
            auto frame_pair = geom::PairMatcher2(frame_list.at(i), frame_list.at(j));
            frame_pair.PrintInfo();
            pair_list.push_back(frame_pair);
        }
    }

    // disjoint set
    printf("--disjointset\n");
#if 0
    datastruct::DisjointSet<geom::FeatIdx> ds;
    // datastruct::DisjointSet< int > ds;
#else
    datastruct::mp::DisjointSetForest<geom::FeatIdx> ds;
#endif

    // add every feature match to disjoint set
    for (size_t p = 0u; p < pair_list.size(); p++)
    {
        // size_t p = 0u;
        // {
        // cv::imshow( "viz", pair_list.at(p).VizImage() );
        // cv::waitKey(0);
        pair_list.at(p).PrintInfo();

        const auto M1Idx = pair_list.at(p).M1Idx();
        const auto M2Idx = pair_list.at(p).M2Idx();

        EXPECT_EQ(M1Idx.size(), M2Idx.size());

        for (size_t f = 0u; f < M1Idx.size(); f++)
        {
            // std::cout << "#" << f << " : " << M1Idx.at(f).Str() << "---" << M2Idx.at(f).Str() << std::endl;

#if 0
            ds.make_set(M1Idx.at(f));
            // printf("total disjoint sets: %u\n", ds.CountSets());
            ds.make_set(M2Idx.at(f));
            // printf("total disjoint sets: %u\n", ds.CountSets());

            ds.unionSets(M1Idx.at(f), M2Idx.at(f));
#else

            // int item_count = ds.element_count();
            int v1 = ds.exists(M1Idx.at(f));
            int v2 = ds.exists(M2Idx.at(f));
            // printf("[initial] v1=%d v2=%d\n", v1, v2);

            if (v1 < 0)
                v1 = ds.append_element(M1Idx.at(f));
            else
            {
                // std::cout << M1Idx.at(f).Str() << " m1 already exisits\n";
            }

            if (v2 < 0)
                v2 = ds.append_element(M2Idx.at(f));
            else
            {
                // std::cout << M2Idx.at(f).Str() << " m2 already exisits\n";
            }

            // printf("[after] v1=%d v2=%d m_setCount=%d\n", v1, v2,  ds.element_count() );

            // ds.add_element(item_count + 1, M1Idx.at(f));
            // ds.add_element(item_count + 2, M2Idx.at(f));
            ds.union_sets(v1, v2);
            // printf( "[after union_set] m_setCount=%d\n", ds.element_count()  );

#endif
        }
    }

#if 0
    printf("total disjoint sets: %u\n", ds.CountSets());
    auto all_roots = ds.GetAllRoots(); 
    for( size_t f=0 ; f<all_roots.size() ; f++ ) {
        // const auto items_in_set = ds.GetSetItems( all_roots.at(f) ); 
        // std::cout << "track#" << f << " (items=" << items_in_set.size() << "): ";
        // for( size_t i=0 ; i<items_in_set.size() ; i++ ) {
        //     std::cout << items_in_set.at(i) << ", "; 
        // }
        std::cout << std::endl;
    }
#else
    printf("total disjoint sets: %u\n", ds.set_count());
    const auto all_roots = ds.get_all_root_slow();
    printf("all_roots.size(): %lu\n", all_roots.size());

    printf("disjoint sets ie. feature tracks: \n");
    int stat[10] = {0};
    for (const int r : all_roots)
    {
        auto S = ds.get_all_items_in_set(r);
        printf("r=%5d n_items=%ld\t", r, S.size());
        stat[int(S.size())]++;
        for (const auto s : S)
        {
            std::cout << s.Str() << " ";
        }
        std::cout << std::endl;
    }

    for (uint32_t i = 0; i < 10; i++)
    {
        printf(" %u features observed %u times\n", stat[i], i);
    }
    printf("total disjoint sets: %u\n", ds.set_count());

#endif
}
