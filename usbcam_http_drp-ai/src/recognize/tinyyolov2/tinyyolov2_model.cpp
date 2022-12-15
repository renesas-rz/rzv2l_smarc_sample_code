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
* File Name    : tinyyolov2_model.cpp
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "tinyyolov2_model.h"
#include "../command/object_detection.h"
#include "../util/string_formatter.h"
TinyYoloV2Model::TinyYoloV2Model() :IRecognizeModel(MODEL_DIR.data(), MODEL_NAME.data(), TINY_YOLOV2_DRPAI_IN_WIDTH, TINY_YOLOV2_DRPAI_IN_HEIGHT, 2)
{
    std::cout << "Yolo model" << std::endl;

    num_grids = { 13 };
    anchors =
    {
        1.08,   1.19,
        3.42,   4.41,
        6.63,   11.38,
        9.42,   5.11,
        16.62,  10.52
    };
    label_file_map = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor" };
    num_inf_out = (label_file_map.size() + 5) * TINY_YOLOV2_NUM_BB * num_grids[0] * num_grids[0];
    outBuffSize = num_inf_out;
}

/**
 * @brief inf_post_process
 * @details implementation post process
 * @param arg 
 * @return int32_t 
 */
int32_t TinyYoloV2Model::inf_post_process(float* arg)
{
    /*CPU Post-Processing for Tiny YOLOv2*/
    postproc_data.clear();
    post_proc(arg, postproc_data);
    return 0;
}

/**
 * @brief get_command
 * @details implementation create command
 * @return shared_ptr<PredictNotifyBase> 
 */
shared_ptr<PredictNotifyBase> TinyYoloV2Model::get_command()
{
    ObjectDetection* ret = new ObjectDetection();
    for(detection det : postproc_data)
    {   
        if (det.prob == 0)
        {
            continue;
        }
        else
        {
            bbox_t dat;
            dat.name = label_file_map[det.c].c_str();
            dat.X = (int32_t)(det.bbox.x - (det.bbox.w / 2));
            dat.Y = (int32_t)(det.bbox.y - (det.bbox.h / 2));
            dat.W = (int32_t)det.bbox.w;
            dat.H = (int32_t)det.bbox.h;
            dat.pred = det.prob * 100.0;

            ret->predict.push_back(dat);
        }
    }
    return shared_ptr<PredictNotifyBase>(move(ret));
}

/**
 * @brief print_result
 * @details implementation print postprocess result
 * @return int32_t 
 */
int32_t TinyYoloV2Model::print_result()
{
    /*Displays AI Inference results & Processing Time on console*/
    YoloCommon::print_boxes(postproc_data, label_file_map);
    return 0;
}

/**
 * @brief post_proc
 * @details NN output to bouding box
 * @param floatarr DRP-AI result
 * @param det detected boundig box list
 */
void TinyYoloV2Model::post_proc(float* floatarr, std::vector<detection>& det)
{
    uint32_t count = 0;
    /* Following variables are required for correct_region_boxes in Darknet implementation*/
    /* Note: This implementation refers to the "darknet detector test" */
    float new_w, new_h;
    float correct_w = 1.;
    float correct_h = 1.;
    if ((float)(TINY_YOLOV2_MODEL_IN_W / correct_w) < (float)(TINY_YOLOV2_MODEL_IN_H / correct_h))
    {
        new_w = (float)TINY_YOLOV2_MODEL_IN_W;
        new_h = correct_h * TINY_YOLOV2_MODEL_IN_W / correct_w;
    }
    else
    {
        new_w = correct_w * TINY_YOLOV2_MODEL_IN_H / correct_h;
        new_h = TINY_YOLOV2_MODEL_IN_H;
    }
    int32_t label_num = label_file_map.size();
    uint32_t n = 0;
    uint32_t b = 0;
    uint32_t y = 0;
    uint32_t x = 0;
    uint32_t offs = 0;
    int32_t i = 0;
    float tx = 0;
    float ty = 0;
    float tw = 0;
    float th = 0;
    double tc = 0;
    float center_x = 0;
    float center_y = 0;
    float box_w = 0;
    float box_h = 0;
    double objectness = 0;
    uint8_t num_grid = 0;
    uint8_t anchor_offset = 0;
    float classes[label_num];
    float max_pred = 0;
    int8_t pred_class = -1;
    float probability = 0;
    detection d;
    Box bb;
    /*Post Processing Start*/
    for (n = 0; n < TINY_YOLOV2_NUM_INF_OUT_LAYER; n++)
    {

        num_grid = num_grids[n];
        anchor_offset = 2 * TINY_YOLOV2_NUM_BB * (TINY_YOLOV2_NUM_INF_OUT_LAYER - (n + 1));

        for (b = 0; b < TINY_YOLOV2_NUM_BB; b++)
        {
            for (y = 0; y < num_grid; y++)
            {
                for (x = 0; x < num_grid; x++)
                {
                    offs = YoloCommon::yolo_offset(n, b, y, x, num_grids.data(), TINY_YOLOV2_NUM_BB, label_file_map.size());
                    tx = floatarr[offs];
                    ty = floatarr[YoloCommon::yolo_index(num_grid, offs, 1)];
                    tw = floatarr[YoloCommon::yolo_index(num_grid, offs, 2)];
                    th = floatarr[YoloCommon::yolo_index(num_grid, offs, 3)];
                    tc = floatarr[YoloCommon::yolo_index(num_grid, offs, 4)];
                    /* Compute the bounding box */
                    /*get_region_box*/
                    center_x = ((float)x + CommonFunc::sigmoid(tx)) / (float)num_grid;
                    center_y = ((float)y + CommonFunc::sigmoid(ty)) / (float)num_grid;
                    box_w = (float)exp(tw) * anchors[anchor_offset + 2 * b + 0] / (float)num_grid;
                    box_h = (float)exp(th) * anchors[anchor_offset + 2 * b + 1] / (float)num_grid;

                    /* Size Adjustment*/
                    /* correct_region_boxes */
                    center_x = (center_x - (TINY_YOLOV2_MODEL_IN_W - new_w) / 2. / TINY_YOLOV2_MODEL_IN_W) / ((float)new_w / TINY_YOLOV2_MODEL_IN_W);
                    center_y = (center_y - (TINY_YOLOV2_MODEL_IN_H - new_h) / 2. / TINY_YOLOV2_MODEL_IN_H) / ((float)new_h / TINY_YOLOV2_MODEL_IN_H);
                    box_w *= (float)(TINY_YOLOV2_MODEL_IN_W / new_w);
                    box_h *= (float)(TINY_YOLOV2_MODEL_IN_H / new_h);

                    center_x = round(center_x * TINY_YOLOV2_DRPAI_IN_WIDTH);
                    center_y = round(center_y * TINY_YOLOV2_DRPAI_IN_HEIGHT);
                    box_w = round(box_w * TINY_YOLOV2_DRPAI_IN_WIDTH);
                    box_h = round(box_h * TINY_YOLOV2_DRPAI_IN_HEIGHT);

                    objectness = CommonFunc::sigmoid(tc);

                    bb = { center_x, center_y, box_w, box_h };


                    /* Get the class prediction */
                    for (i = 0; i < label_num; i++)
                    {
                        classes[i] = floatarr[YoloCommon::yolo_index(num_grid, offs, 5 + i)];
                    }
                    CommonFunc::softmax(classes, label_num);
                    max_pred = 0;
                    pred_class = -1;
                    for (i = 0; i < label_num; i++)
                    {
                        if (classes[i] > max_pred)
                        {
                            pred_class = i;
                            max_pred = classes[i];
                        }
                    }

                    /* Store the result into the list if the probability is more than the threshold */
                    probability = max_pred * objectness;
                    if (probability > TINY_YOLOV2_TH_PROB)
                    {
                        d = { bb, pred_class, probability };
                        det.push_back(d);
                        count++;
                    }
                }
            }
        }
    }
    /* Non-Maximum Supression filter */
    filter_boxes_nms(det, count, TINY_YOLOV2_TH_NMS);
}

