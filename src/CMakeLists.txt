set(THIRD_PARTY_INCLUDE_DIRS
        ${CERES_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})
set(THIRD_PARTY_LIBS
        ${CERES_LIBRARIES}
        ${OpenCV_LIBS})

add_definitions("-DCALIB_DLL_EXPORT")
include_directories(${PROJECT_SOURCE_DIR}/include
    ${THIRD_PARTY_INCLUDE_DIRS})
link_directories(${PROJECT_SOURCE_DIR}/lib)

aux_source_directory(. LIBCALIB_SOURCE_LIST)
add_library(calib ${LIB_TYPE} ${LIBCALIB_SOURCE_LIST})

target_link_libraries(calib cbdetect ${THIRD_PARTY_LIBS})

