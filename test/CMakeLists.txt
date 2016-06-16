add_executable( test_svo_wrapper test_svo_wrapper.cpp )
target_link_libraries( test_svo_wrapper 
    libzedslam
)

add_executable( test_feature_detection test_feature_detection.cpp )
target_link_libraries( test_feature_detection 
    libzedslam
)

add_executable( test_feature_tracking test_feature_tracking.cpp )
target_link_libraries( test_feature_tracking 
    libzedslam
)

add_executable( test_semi_direct_vo test_semi_direct_vo.cpp )
target_link_libraries( test_semi_direct_vo 
    libzedslam
)

add_executable( test_semi_direct_vo_g2o test_semi_direct_vo_g2o.cpp )
target_link_libraries( test_semi_direct_vo_g2o
    libzedslam
)

add_executable( test_semi_direct_vo_g2o_video test_semi_direct_vo_g2o_video.cpp )
target_link_libraries( test_semi_direct_vo_g2o_video
    libzedslam
)

add_executable( test_direct_vo test_direct_vo.cpp )
target_link_libraries( test_direct_vo 
    libzedslam
)

add_executable( test_direct_vo_g2o_video test_direct_vo_g2o_video.cpp )
target_link_libraries( test_direct_vo_g2o_video
    libzedslam
)

add_executable( test_localmap test_localmap.cpp )
target_link_libraries( test_localmap
    libzedslam
)

