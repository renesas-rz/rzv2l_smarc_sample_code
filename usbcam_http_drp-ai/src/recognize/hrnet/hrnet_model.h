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
* File Name    : hrnet_model.h
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

#pragma once

#ifndef HRNET_MODEL_H
#define HRNET_MODEL_H

/*****************************************
* Includes
******************************************/
#include <linux/drpai.h>
#include "../irecognize_model.h"
#include "../../includes.h"
#include "../command/pose_detection.h"

/*****************************************
* Static Variables and Macro for HRNet
******************************************/

class HRnetModel : public IRecognizeModel
{
private:

#ifdef MODEL_VGA
    constexpr static string_view HRNET_MODEL_DIR = "hrnet_cam";
    constexpr static  int32_t HRNET_DRPAI_IN_WIDTH = (640);
    constexpr static  int32_t HRNET_DRPAI_IN_HEIGHT = (480);
#else
    constexpr static string_view HRNET_MODEL_DIR = "hrnet_cam_fhd";
    constexpr static  int32_t HRNET_DRPAI_IN_WIDTH = (1920);
    constexpr static  int32_t HRNET_DRPAI_IN_HEIGHT = (1080);

#endif

    /*HRNet Related*/
    constexpr static  int32_t HRNET_NUM_OUTPUT_W = (48);
    constexpr static  int32_t HRNET_NUM_OUTPUT_H = (64);
    constexpr static  int32_t HRNET_NUM_OUTPUT_C = (17);
    /* Number of DRP-AI output */
    constexpr static  int32_t NUM_INF_OUT = HRNET_NUM_OUTPUT_W * HRNET_NUM_OUTPUT_H * HRNET_NUM_OUTPUT_C;
    constexpr static  float HRNET_TH_KPT = (0.1f);

    /*DRP-AI Input image information*/
    constexpr static string_view MODEL_NAME = "hrnet_cam";
    constexpr static  int32_t HRNET_DRPAI_IN_CHANNEL = (2);
    constexpr static  int32_t HRNET_DRPAI_IN_SIZE = (HRNET_DRPAI_IN_WIDTH * HRNET_DRPAI_IN_HEIGHT * HRNET_DRPAI_IN_CHANNEL);
    /*Cropping Image Related*/
    constexpr static float HRNET_CROPPED_IMAGE_WIDTH = (HRNET_DRPAI_IN_WIDTH);
    constexpr static  float HRNET_CROPPED_IMAGE_HEIGHT = (HRNET_DRPAI_IN_HEIGHT);
    /*HRNet Post Processing & Drawing Related*/
    constexpr static  float HRNET_OUTPUT_LEFT = (276 * (HRNET_DRPAI_IN_WIDTH / 960.0f));
    constexpr static  float HRNET_OUTPUT_TOP = (0);
    constexpr static  float HRNET_OUTPUT_WIDTH = (405 * (HRNET_DRPAI_IN_WIDTH / 960.0f));
    constexpr static  float HRNET_OUTPUT_HEIGHT = (HRNET_DRPAI_IN_HEIGHT);
    constexpr static  float HRNET_OUTPUT_ADJ_X = (2);
    constexpr static  float HRNET_OUTPUT_ADJ_Y = (0);



public:
    HRnetModel();
    virtual int32_t inf_post_process(float* arg);
    virtual shared_ptr<PredictNotifyBase> get_command();
    virtual int32_t print_result();

    int32_t hrnet_offset(int32_t b, int32_t y, int32_t x);

private:
    int8_t sign(int32_t x);
    void coord_convert(vector<pos_t>& result, float preds[][3]);
    int8_t post_process(vector<pos_t>& result, float* floatarr);


private:

    vector<pos_t> postproc_result;

};

#endif //HRNET_MODEL_H
