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
* File Name    : hrnet_model.cpp
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "hrnet_model.h"
HRnetModel::HRnetModel() : IRecognizeModel(128 * 1024, HRNET_MODEL_DIR.data(), MODEL_NAME.data(), HRNET_DRPAI_IN_WIDTH, HRNET_DRPAI_IN_HEIGHT, HRNET_DRPAI_IN_CHANNEL)
{
    std::cout << "HRNet model" << std::endl;
}

int32_t HRnetModel::inf_post_process(float* arg)
{
    postproc_result.clear();
    post_process(postproc_result, arg);
    return 0;

}
int32_t HRnetModel::print_result()
{
    /*Displays AI Inference results & Processing Time on console*/
    int32_t id = 0;
    for (pos_t p : postproc_result)
    {
        printf("  ID %d: (%.2d, %.2d): %.2f%% \n", id, p.X, p.Y, p.preds);
        id++;
    }
    return 0;
}

shared_ptr<PredictNotifyBase> HRnetModel::get_command()
{     
    PoseDetection* ret = new PoseDetection();
    for (pos_t p : postproc_result)
    {
        ret->predict.push_back(p);
    }
    return shared_ptr<PredictNotifyBase>(move(ret));
}


/*****************************************
* Function Name : hrnet_offset
* Description   : Get the offset number to access the HRNet attributes
* Arguments     : b = Number to indicate which region [0~17]
*                 y = Number to indicate which region [0~64]
*                 x = Number to indicate which region [0~48]
* Return value  : offset to access the HRNet attributes.
*******************************************/
int32_t HRnetModel::hrnet_offset(int32_t b, int32_t y, int32_t x)
{
    return b * HRNET_NUM_OUTPUT_W * HRNET_NUM_OUTPUT_H + y * HRNET_NUM_OUTPUT_W + x;
}

/*****************************************
* Function Name : sign
* Description   : Get the sign of the input value
* Arguments     : x = input value
* Return value  : returns the sign, 1 if positive -1 if not
*******************************************/
int8_t HRnetModel::sign(int32_t x)
{
    return x > 0 ? 1 : -1;
}

/**
 * @brief coord_convert
 * @details HRNet coord convert to original image size
 * @param result reference to store result
 * @param preds postproce result
 */
void HRnetModel::coord_convert(vector<pos_t> &result, float preds[][3])
{
    /* Render skeleton on image and print their details */
    int32_t posx = 0;
    int32_t posy = 0;
    int8_t i = 0;
    result.clear();
    for (i = 0; i < HRNET_NUM_OUTPUT_C; i++)
    {
        /* 0.5 is round */
        posx = (int32_t)(preds[i][0] / HRNET_CROPPED_IMAGE_WIDTH * HRNET_OUTPUT_WIDTH + 0.5) + HRNET_OUTPUT_LEFT + HRNET_OUTPUT_ADJ_X;
        posy = (int32_t)(preds[i][1] / HRNET_CROPPED_IMAGE_HEIGHT * HRNET_OUTPUT_HEIGHT + 0.5) + HRNET_OUTPUT_TOP + HRNET_OUTPUT_ADJ_Y;

        pos_t p;
        p.X = posx;
        p.Y = posy;
        p.preds = preds[i][2] * 100;
        result.push_back(p);
    }
    return;
}

/**
 * @brief post_process
 * @details implementation post process
 * @param result reference to store result(X,Y,predict)
 * @param floatarr DRP-AI result
 * @return int8_t 
 */
int8_t HRnetModel::post_process(vector<pos_t> &result,float* floatarr)
{
    float     lowest_kpt_score = 0;

    float score = 0;
    int32_t b = 0;
    int32_t y = 0;
    int32_t x = 0;
    int32_t i = 0;
    int32_t offs = 0;

    float center[] = { HRNET_CROPPED_IMAGE_WIDTH / 2 - 1, HRNET_CROPPED_IMAGE_HEIGHT / 2 - 1 };
    int8_t ind_x = -1;
    int8_t ind_y = -1;
    float max_val = -1;
    float scale_x, scale_y, coords_x, coords_y;
    float hrnet_preds[HRNET_NUM_OUTPUT_C][3];

    for (b = 0; b < HRNET_NUM_OUTPUT_C; b++)
    {
        float scale[] = { HRNET_CROPPED_IMAGE_WIDTH / 200.0, HRNET_CROPPED_IMAGE_HEIGHT / 200.0 };
        ind_x = -1;
        ind_y = -1;
        max_val = -1;
        for (y = 0; y < HRNET_NUM_OUTPUT_H; y++)
        {
            for (x = 0; x < HRNET_NUM_OUTPUT_W; x++)
            {
                offs = hrnet_offset(b, y, x);
                if (floatarr[offs] > max_val)
                {
                    /*Update the maximum value and indices*/
                    max_val = floatarr[offs];
                    ind_x = x;
                    ind_y = y;
                }
            }
        }
        if (0 > max_val)
        {
            ind_x = -1;
            ind_y = -1;
            lowest_kpt_score = 0;
            return -1 ;
        }
        hrnet_preds[b][0] = float(ind_x);
        hrnet_preds[b][1] = float(ind_y);
        hrnet_preds[b][2] = max_val;
        offs = hrnet_offset(b, ind_y, ind_x);
        if (ind_y > 1 && ind_y < HRNET_NUM_OUTPUT_H - 1)
        {
            if (ind_x > 1 && ind_x < HRNET_NUM_OUTPUT_W - 1)
            {
                float diff_x = floatarr[offs + 1] - floatarr[offs - 1];
                float diff_y = floatarr[offs + HRNET_NUM_OUTPUT_W] - floatarr[offs - HRNET_NUM_OUTPUT_W];
                hrnet_preds[b][0] += sign(diff_x) * 0.25;
                hrnet_preds[b][1] += sign(diff_y) * 0.25;
            }
        }

        /*transform_preds*/
        scale[0] *= 200;
        scale[1] *= 200;
        /* udp (Unbiased Data Processing) = False */
        scale_x = scale[0] / (HRNET_NUM_OUTPUT_W);
        scale_y = scale[1] / (HRNET_NUM_OUTPUT_H);
        coords_x = hrnet_preds[b][0];
        coords_y = hrnet_preds[b][1];
        hrnet_preds[b][0] = coords_x * scale_x + center[0] - scale[0] * 0.5;
        hrnet_preds[b][1] = coords_y * scale_y + center[1] - scale[1] * 0.5;
    }
    /* Clear the score in preparation for the update. */
    lowest_kpt_score = 0;
    score = 1;
    for (i = 0; i < HRNET_NUM_OUTPUT_C; i++)
    {
        /* Adopt the lowest score. */
        if (hrnet_preds[i][2] < score)
        {
            score = hrnet_preds[i][2];
        }
    }
    /* Update the score for display thread. */
    lowest_kpt_score = score;

    if (HRNET_TH_KPT < lowest_kpt_score)
    {
        coord_convert(result, hrnet_preds);
    }

    return 0;
}
