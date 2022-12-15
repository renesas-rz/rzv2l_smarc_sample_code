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
* File Name    : resnet_model.cpp
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "resnet_model.h"
#include "../command/classification.h"

ResnetModel::ResnetModel() : IRecognizeModel(RESNET_NUM_CLASS, MODEL_DIR.data(), MODEL_NAME.data(), RESNET_DRPAI_IN_WIDTH, RESNET_DRPAI_IN_HEIGHT, RESNET_DRPAI_IN_CHANNEL), label_file_map(load_label_file(label_list))
{
    if (label_file_map.empty())
    {
        fprintf(stderr, "[ERROR] Failed to load label file: %s\n", label_list.data());
    } 
}

/**
 * @brief inf_post_process
 * @details implementation post process
 * @param arg NN output
 * @return int32_t 
 */
int32_t ResnetModel::inf_post_process(float*arg)
{
    postproc_result.clear();
    return post_process(postproc_result, arg);
}

/**
 * @brief load_label_file
 * @details Load label list text file and return the label list that contains the label.
 * @param label_file_name filename of label list. must be in txt format
 * @return std::map<int32_t, std::string> list contains labels
 */
std::map<int32_t, std::string> ResnetModel::load_label_file(string_view  label_file_name)
{
    int32_t n = 0;
    std::map<int32_t, std::string> list = {};
    std::map<int32_t, std::string> empty = {};
    std::ifstream infile(label_file_name.data());

    if (!infile.is_open())
    {
        return list;
    }

    std::string line = "";
    while (std::getline(infile, line))
    {
        list[n++] = line;
        if (infile.fail())
        {
            return empty;
        }
    }

    return list;
}

/**
 * @brief get_command
 * @details implementation create command
 * @return shared_ptr<PredictNotifyBase> 
 */
shared_ptr<PredictNotifyBase> ResnetModel::get_command()
{
    Classification* ret = new Classification();
    int32_t cnt=0;
    for (reverse_iterator it = postproc_result.rbegin(); it != postproc_result.rend(); it++)
    {
        if (cnt == RESNET_TOP_NUM)break;
        cnt++;
        classify_t dat;
        dat.names = label_file_map[(*it).second];
        dat.pred = (*it).first * 100;
        ret->predict.push_back(dat);
    }

    return shared_ptr<PredictNotifyBase>(move(ret));
}

/**
 * @brief print_result
 * @details implementation print postprocess result
 * @return int32_t 
 */
int32_t ResnetModel::print_result()
{
   return print_result(postproc_result);
}

/**
 * @brief post_process
 * @details NN output to classes
 * @param result detected classes
 * @param floatarr DRP-AI result
 * @return int32_t 
 */
int32_t ResnetModel::post_process(std::map<float, int32_t>& result, float* floatarr)
{
    int32_t i = 0;
    if (label_file_map.empty())
    {
        fprintf(stderr, "[ERROR] Failed to load label file\n");
        return -1;
    }

    /* Post-processing */
    /* Score the model, and print scores for first 5 classes */
    for (i = 0; i < RESNET_NUM_CLASS; i++)
    {
        result[floatarr[i]] = i;
    }

    return 0;
}

/**
 * @brief print_result
 * @details implementation print postprocess result
 * @param result postprocessed result
 * @return int8_t 
 */
int8_t ResnetModel::print_result(std::map<float, int32_t> result)
{
   
    int32_t result_cnt = 0;
    for (reverse_iterator it = result.rbegin(); it != result.rend(); it++)
    {
        result_cnt++;
        if (result_cnt > 5) break;
        printf("  Top %d [%5.1f%%] : [%s]\n", result_cnt, (*it).first * 100, label_file_map[(*it).second].c_str());
    }

    return 0;
}
