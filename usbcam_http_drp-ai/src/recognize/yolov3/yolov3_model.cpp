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
* File Name    : yolov3_model.cpp
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "yolov3_model.h"
#include "../command/object_detection.h"

/**
 * @brief Construct a new Yolo V 3 Model:: Yolo V 3 Model object
 * 
 */
YoloV3Model::YoloV3Model() :IRecognizeModel(MODEL_DIR.data(), MODEL_NAME.data(), YOLOV3_DRPAI_IN_WIDTH, YOLOV3_DRPAI_IN_HEIGHT, 2)
{
    std::cout << "Yolo model" << std::endl;

    label_file_map = YoloCommon::load_label_file(LABEL_LIST.data());

    num_class = label_file_map.size();

    /* init num_grids*/
    num_grids = { 13,26,52 };
     /*init num_inf_out*/
    num_inf_out = (num_class + 5) * YOLOV3_NUM_BB * num_grids[0] * num_grids[0]
        + (num_class + 5) * YOLOV3_NUM_BB * num_grids[1] * num_grids[1]
        + (num_class + 5) * YOLOV3_NUM_BB * num_grids[2] * num_grids[2];
    /* init anchors*/
    anchors =
    {
        10, 13,
        16, 30,
        33, 23,
        30, 61,
        62, 45,
        59, 119,
        116, 90,
        156, 198,
        373, 326
    };

    outBuffSize = num_inf_out;
}

/**
 * @brief inf_post_process
 * @details  implementation post process
 * @param arg 
 * @return int32_t 
 */
int32_t YoloV3Model::inf_post_process(float* arg)
{
    postproc_data.clear();
    post_proc(arg, postproc_data);
    return 0;
}

/**
 * @brief implementation create command
 * 
 * @return shared_ptr<PredictNotifyBase> 
 */
shared_ptr<PredictNotifyBase> YoloV3Model::get_command()
{
    ObjectDetection* ret = new ObjectDetection();
    for (detection det : postproc_data)
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
int32_t YoloV3Model::print_result()
{
    YoloCommon::print_boxes(postproc_data, label_file_map);
    return 0;
}

/**
 * @brief post process
 * @details NN output to bouding box
 * @param floatarr DRP-AI result
 * @param det detected boundig box list
 */
void YoloV3Model::post_proc(float* floatarr, vector<detection>& det)
{
    /* Following variables are required for correct_yolo/region_boxes in Darknet implementation*/
  /* Note: This implementation refers to the "darknet detector test" */
    float new_w, new_h;
    float correct_w = 1.;
    float correct_h = 1.;
    if ((float)(YOLOV3_MODEL_IN_W / correct_w) < (float)(YOLOV3_MODEL_IN_H / correct_h))
    {
        new_w = (float)YOLOV3_MODEL_IN_W;
        new_h = correct_h * YOLOV3_MODEL_IN_W / correct_w;
    }
    else
    {
        new_w = correct_w * YOLOV3_MODEL_IN_H / correct_h;
        new_h = YOLOV3_MODEL_IN_H;
    }

    int32_t n = 0;
    int32_t b = 0;
    int32_t y = 0;
    int32_t x = 0;
    int32_t offs = 0;
    int32_t i = 0;
    float tx = 0;
    float ty = 0;
    float tw = 0;
    float th = 0;
    float tc = 0;
    float center_x = 0;
    float center_y = 0;
    float box_w = 0;
    float box_h = 0;
    float objectness = 0;
    uint8_t num_grid = 0;
    uint8_t anchor_offset = 0;
    float classes[num_class];
    float max_pred = 0;
    int32_t pred_class = -1;
    float probability = 0;
    detection d;

    for (n = 0; n < YOLOV3_NUM_INF_OUT_LAYER; n++)
    {
        num_grid = num_grids[n];
        anchor_offset = 2 * YOLOV3_NUM_BB * (YOLOV3_NUM_INF_OUT_LAYER - (n + 1));
        for (b = 0; b < YOLOV3_NUM_BB; b++)
        {
            for (y = 0; y < num_grid; y++)
            {
                for (x = 0; x < num_grid; x++)
                {
                    offs = YoloCommon::yolo_offset(n, b, y, x, num_grids.data(), YOLOV3_NUM_BB, label_file_map.size());
                    tx = floatarr[offs];
                    ty = floatarr[YoloCommon::yolo_index(num_grid, offs, 1)];
                    tw = floatarr[YoloCommon::yolo_index(num_grid, offs, 2)];
                    th = floatarr[YoloCommon::yolo_index(num_grid, offs, 3)];
                    tc = floatarr[YoloCommon::yolo_index(num_grid, offs, 4)];

                    /* Compute the bounding box */
                    /*get_yolo_box/get_region_box in paper implementation*/
                    center_x = ((float)x + CommonFunc::sigmoid(tx)) / (float)num_grid;
                    center_y = ((float)y + CommonFunc::sigmoid(ty)) / (float)num_grid;
                    box_w = (float)exp(tw) * anchors[anchor_offset + 2 * b + 0] / (float)YOLOV3_MODEL_IN_W;
                    box_h = (float)exp(th) * anchors[anchor_offset + 2 * b + 1] / (float)YOLOV3_MODEL_IN_W;
                    /* Adjustment for VGA size */
                    /* correct_yolo/region_boxes */
                    center_x = (center_x - (YOLOV3_MODEL_IN_W - new_w) / 2. / YOLOV3_MODEL_IN_W) / ((float)new_w / YOLOV3_MODEL_IN_W);
                    center_y = (center_y - (YOLOV3_MODEL_IN_H - new_h) / 2. / YOLOV3_MODEL_IN_H) / ((float)new_h / YOLOV3_MODEL_IN_H);
                    box_w *= (float)(YOLOV3_MODEL_IN_W / new_w);
                    box_h *= (float)(YOLOV3_MODEL_IN_H / new_h);

                    center_x = round(center_x * YOLOV3_DRPAI_IN_WIDTH);
                    center_y = round(center_y * YOLOV3_DRPAI_IN_HEIGHT);
                    box_w = round(box_w * YOLOV3_DRPAI_IN_WIDTH);
                    box_h = round(box_h * YOLOV3_DRPAI_IN_HEIGHT);
                    
                    objectness = CommonFunc::sigmoid(tc);

                    Box bb = { center_x, center_y, box_w, box_h };
                    /* Get the class prediction */
                    {
                        for (i = 0; i < num_class; i++)
                        {
                            classes[i] = CommonFunc::sigmoid(floatarr[YoloCommon::yolo_index(num_grid, offs, 5 + i)]);
                        }
                    }
                    max_pred = 0;
                    pred_class = -1;
                    for (i = 0; i < num_class; i++)
                    {
                        if (classes[i] > max_pred)
                        {
                            pred_class = i;
                            max_pred = classes[i];
                        }
                    }

                    /* Store the result into the list if the probability is more than the threshold */
                    probability = max_pred * objectness;
                    if (probability > YOLOV3_TH_PROB)
                    {
                        d = { bb, pred_class, probability };
                        det.push_back(d);
                    }
                }
            }
        }
    }
    /* Non-Maximum Supression filter */
    filter_boxes_nms(det, det.size(), YOLOV3_TH_NMS);

}
