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
* File Name    : resnet_model.h
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

#pragma once
#ifndef RESNET_MODEL_H
#define RESNET_MODEL_H

/*****************************************
* Includes
******************************************/
#include <linux/drpai.h>
#include "../irecognize_model.h"
#include "../../includes.h"


class ResnetModel : public IRecognizeModel
{

private:
    constexpr static   string_view label_list = "synset_words_imagenet.txt";
    constexpr static   string_view MODEL_NAME = "resnet50_cam";
    /*Number of class to be classified (Need to change to use the customized model.)*/
    constexpr static  int32_t RESNET_NUM_CLASS = 1000;
    constexpr static  int32_t RESNET_DRPAI_IN_CHANNEL = 2;
    constexpr static  int32_t RESNET_TOP_NUM = 5;
#ifdef MODEL_VGA
    constexpr static string_view MODEL_DIR = "resnet50_cam";
    constexpr static int32_t  RESNET_DRPAI_IN_WIDTH = 640;
    constexpr static int32_t  RESNET_DRPAI_IN_HEIGHT = 480;
#else
    constexpr static string_view MODEL_DIR = "resnet50_cam_fhd";
    constexpr static int32_t  RESNET_DRPAI_IN_WIDTH = 1920;
    constexpr static int32_t  RESNET_DRPAI_IN_HEIGHT = 1080;

#endif

public:
    ResnetModel();
    virtual int32_t inf_post_process(float* arg);
    virtual shared_ptr<PredictNotifyBase> get_command();
    virtual int32_t print_result();

private:
    int32_t post_process(std::map<float, int32_t>& result, float* floatarr);
    int8_t print_result(std::map<float, int32_t> result);
    std::map<int32_t, std::string> load_label_file(string_view  label_file_name);

private:
    std::map<int32_t, std::string> label_file_map;
    std::map<float, int32_t> postproc_result;
};
#endif // !RESNET_MODEL_H
