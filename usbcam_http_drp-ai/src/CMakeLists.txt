cmake_minimum_required (VERSION 3.8)
project(RZV_USB_SAMPLE)

add_executable (sample_app_usbcam_http 
"main" 
"ws_server" 
"include/lwsock.hpp"
"recognize_proc"
"image_converter" 
"camera/camera" 
"drp/drp_proc" 
"command/command_base" 
"command/bbox_t.h" 
"command/classify_t.h" 
"util/system_analyzer" 
"recognize/recognize_base" 
"recognize/irecognize_model.h" 
"recognize/common/box" 
"recognize/tinyyolov2/tinyyolov2_model" 
"recognize/yolov3/yolov3_model" 
"recognize/resnet/resnet_model" 
"recognize/hrnet/hrnet_model"
)
target_compile_options(sample_app_usbcam_http PUBLIC ${CXX} ${LDFLAGS} ${OpenMP_CXX_FLAGS} -lm -lrt -O3 -DNDEBUG)
target_compile_features(sample_app_usbcam_http PUBLIC cxx_std_17)
link_directories("/usr/lib64/")
include_directories("include")
include_directories("/usr/include")
include_directories(SYSTEM $ENV{OECORE_NATIVE_SYSROOT} /opt/poky/3.1.14/sysroots/aarcch64-poky-linux/usr/include)
TARGET_LINK_LIBRARIES(sample_app_usbcam_http pthread)
TARGET_LINK_LIBRARIES(sample_app_usbcam_http turbojpeg)
TARGET_LINK_LIBRARIES(sample_app_usbcam_http jpeg)

