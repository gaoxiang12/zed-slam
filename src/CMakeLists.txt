set( SRC_LIST 
    core/Feature.cpp 
    core/Frame.cpp 
    core/Map.cpp 
    core/MapPoint.cpp
    core/Tracker.cpp 
    core/Featuredetection.cpp 
    core/Config.cpp
    
    io/Zed_svo_wrapper.cpp 
    
    utils/SLAMTools.cpp
    
    g2o_types/types_direct.cpp
)

set( G2O_LIBS 
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension 
)

add_library( libzedslam ${SRC_LIST})

target_link_libraries( libzedslam 
    ${ZED_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_nppi_LIBRARY}
    ${CUDA_npps_LIBRARY}
    ${fast_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/3rdparty/vikit_common/lib/libvikit_common.so
    ${G2O_LIBS}
    ${CHOLDMOD_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${Sophus_LIBRARIES}
)