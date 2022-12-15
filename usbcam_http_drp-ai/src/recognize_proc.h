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
* File Name    : recognize_proc.h
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

#ifndef RECOGNIZEPROC_H
#define RECOGNIZEPROC_H

#pragma once
/*****************************************
* Includes
******************************************/
#include "includes.h"
#include "recognize/recognize_base.h"
#include "recognize/resnet/resnet_model.h"
#include "recognize/tinyyolov2/tinyyolov2_model.h"
#include "recognize/yolov3/yolov3_model.h"
#include "recognize/hrnet/hrnet_model.h"

using namespace std;
class RecognizeProc
{
public:
    RecognizeProc(shared_ptr<WebsocketServer> server)
    {
        _switch = true;
        p_recog_base = shared_ptr<RecognizeBase>(move(new RecognizeBase(server)));
    }
    void start_recognize();
    void stop_recognize();
    void switch_model(std::string model);
    void finish_recognize_thread() { blRunPredict = false; }
private:
    pthread_t thPredict;

    // thread control
    void predict_thread();
    static void* predict_thread_wrapper(void* object);
    static void run_predict(RecognizeProc* arg);
    bool _switch;

    shared_ptr<RecognizeBase> p_recog_base;

private:
    bool blRunPredict;
};

#endif 
