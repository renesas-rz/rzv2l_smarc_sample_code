/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : yolov3_model.h
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

#pragma once
#ifndef YOLOV3MODEL_H
#define YOLOV3MODEL_H

/*****************************************
* Includes
******************************************/
#include <linux/drpai.h>
#include "../irecognize_model.h"
#include "../../includes.h"
#include "../common/box.h"
#include "../common/functions.h"
#include "../common/yolo_common.h"

class YoloV3Model :public IRecognizeModel
{
private:
    constexpr static  string_view LABEL_LIST = "coco-labels-2014_2017.txt";
    constexpr static  string_view MODEL_NAME = "yolov3_cam";
    constexpr static  int32_t  YOLOV3_NUM_BB = 3;
    constexpr static int32_t  YOLOV3_NUM_INF_OUT_LAYER = 3;
    constexpr static float  YOLOV3_TH_PROB = 0.5f;
    constexpr static float  YOLOV3_TH_NMS = 0.5f;
    constexpr static int32_t  YOLOV3_MODEL_IN_W = 416;
    constexpr static int32_t  YOLOV3_MODEL_IN_H = 416;
#ifdef MODEL_VGA
    constexpr static int32_t  YOLOV3_DRPAI_IN_WIDTH = 640;
    constexpr static int32_t  YOLOV3_DRPAI_IN_HEIGHT = 480;
    constexpr static string_view MODEL_DIR = "yolov3_cam";
#else
    constexpr static string_view MODEL_DIR = "yolov3_cam_fhd";
    constexpr static int32_t  YOLOV3_DRPAI_IN_WIDTH = 1920;
    constexpr static int32_t  YOLOV3_DRPAI_IN_HEIGHT = 1080;

#endif
public:
    YoloV3Model();
    virtual ~YoloV3Model() {}
    virtual int32_t inf_post_process(float* arg);
    virtual shared_ptr<PredictNotifyBase> get_command();
    virtual int32_t print_result();

private:
    void post_proc(float* floatarr, std::vector<detection>& det);


    /*****************************************
    * YOLOv3
    ******************************************/
    /* Class labels to be classified */
    /* Empty since labels will be loaded from label_list file */
    std::vector<std::string> label_file_map = {};

    /* Number of grids in the image. The length of this array MUST match with the NUM_INF_OUT_LAYER */
    vector<uint8_t> num_grids;
    /* Number of DRP-AI output */
    uint32_t num_inf_out;
    /* Anchor box information */
    vector<double> anchors;

    int32_t num_class;

    vector<detection> postproc_data;


};

#endif
