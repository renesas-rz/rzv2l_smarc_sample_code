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
* File Name    : recognize_base.cpp
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "recognize_base.h"
#include "../image_converter.h"
#include "../command/object_detection.h"
#include "../command/camera_image.h"
#include "../command/cpu_usage.h"
#include "../include/lwsock.hpp"
#include "common/recognize_define.h"

/**
 * @brief RecognizeBase
 * @details  Construct a new Recognize Base:: Recognize Base object
 * @param server reference to websocker server
 */
RecognizeBase::RecognizeBase(shared_ptr<WebsocketServer> server)
{
    _drp = make_shared<DRPProc>();
    _server = server;

    _pthread_ai_inf = 0;
    _pthread_capture = 0;
    _pthread_framerate = 0;
}
/**
 * @brief print_measure_log
 * @details Print measurement log to console
 * @param item measurement item
 * @param time elapsed time
 * @param unit unit
 */
void print_measure_log(string item, float time, string unit)
{
    printf("[MeasLog],%s, %.1f, [%s]\n", item.c_str(), time, unit.c_str());
}

/**
 * @brief print_measure_log
 * @details Print measurement log to console
 * @param item measurement item
 * @param log log
 */
void print_measure_log(string item, string log)
{
    printf("[MeasLog],%s,%s\n", item.c_str(), log.c_str());
}

/**
 * @brief initialize
 * @details Initialization for recognize process.
 * @param model DRP-AI Model
 * @return int32_t success:0 error: != 0
 */
int32_t RecognizeBase::initialize(IRecognizeModel* model)
{
    std::cout << "############ INIT ############" << std::endl;
    _model = shared_ptr<IRecognizeModel>(move(model));

    std::cout << "DRP PREFIX:" << _model->model_prefix << std::endl;
    std::cout << "outbuff :" << _model->outBuffSize << std::endl;

    _outBuffSize = _model->outBuffSize;
    dir = _model->model_dir + "/";
    address_file = dir + _model->model_prefix + "_addrmap_intm.txt";
    //drpai_file_path[0] = dir + "/drp_desc.bin";
    //drpai_file_path[1] + "/" + _model->model_prefix + "_drpcfg.mem";
    //drpai_file_path[2] + "/drp_param.bin";
    //drpai_file_path[3] + "/aimac_desc.bin";
    //drpai_file_path[4] + "/" + _model->model_prefix + "_weight.dat";

    return 0;
}

/**
 * @brief recognize_start
 * @details load drp data / start threads
 * @return int32_t success:0 error: != 0
 */
int32_t RecognizeBase::recognize_start()
{
    printf("RZ/V DRP-AI Sample Application\n");
    printf("Model :  %s\n", _model->model_prefix.c_str());
    printf("Input : USB Camera\n");

    /* Create Camera Instance */
    _capture = make_shared<Camera>();
    /* Init and Start Camera */
    _capture->set_w(_model->_capture_w);
    _capture->set_h(_model->_capture_h);
    _capture->set_c(_model->_capture_c);
    int8_t ret = _capture->start_camera();
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to initialize USB Camera.\n");
        return -1;
    }

    std::cout << "file:" << address_file.c_str() << std::endl;
    ret = _drp->read_addrmap_txt(address_file, &drpai_address);

    /*DRP-AI Driver Open*/
    ret = _drp->drp_open();

    /* Load DRP-AI Data from Filesystem to Memory via DRP-AI Driver */
    drpai_file_path[0] = dir + "/drp_desc.bin";
    drpai_file_path[1] = dir + "/" + _model->model_prefix + "_drpcfg.mem";
    drpai_file_path[2] = dir + "/drp_param.bin";
    drpai_file_path[3] = dir + "/aimac_desc.bin";
    drpai_file_path[4] = dir + "/" + _model->model_prefix + "_weight.dat";

    ret = _drp->load_drpai_data(drpai_address, drpai_file_path);

    if (ret != 0)
    {
        fprintf(stderr, "[ERROR] Failed to Load DRP-AI Data.\n");
        return -1;
    }

    wake_ = false;
    capture_enabled.store(true);

    /* capture thread */
    _capture_running = true;
    int32_t create_thread_cap = pthread_create(&_pthread_capture, NULL, capture_thread, this);
    if (0 != create_thread_cap)
    {
        fprintf(stderr, "[ERROR] Failed to create AI Inference Thread.\n");
        return -1;
    }

#ifdef INFERENE_ON

    /* inference thread */
    _inf_running = true;
    int32_t create_thread_ai = pthread_create(&_pthread_ai_inf, NULL, inference_thread, this);
    if (0 != create_thread_ai)
    {
        fprintf(stderr, "[ERROR] Failed to create AI Inference Thread.\n");
        return -1;
    }

#endif // INFERENE_ON


    /* framerate thread */
    _fps_runnning = true;
    int32_t framerate_thread_cap = pthread_create(&_pthread_framerate, NULL, framerate_thread, this);
    if (0 != framerate_thread_cap)
    {
        fprintf(stderr, "[ERROR] Failed to create Framerate Thread.\n");
        return -1;
    }

    return 0;
}
/**
 * @brief recognize_end
 * @details Recognize end proc
 */
void RecognizeBase::recognize_end()
{
    end_all_threads();
    close_camera();
    if (_drp)_drp->drp_close();

    std::cout << "********************** END *********************" << std::endl;
}

/**
 * @brief capture_thread
 * @details usb camera capture and store umda buffer address for DRP-AI
 * @param arg pointer to itself
 * @return void* 
 */
void* RecognizeBase::capture_thread(void* arg)
{
    RecognizeBase* me = (RecognizeBase*)arg;
    shared_ptr<Camera> capture = me->_capture;

    /*First Loop Flag*/
    uint32_t capture_addr = 0;
    int8_t ret = 0;


    CameraImage notify;
    vector<uint8_t> output;

    printf("_capture Thread Starting\n");
    while (me->_capture_running)
    {
        errno = 0;
#ifdef SEQUENCTCIAL
        std::cout << "[cam_thread]waiting for capture enable..." << std::endl;
        while (!me->capture_enabled.load())
        {
            usleep(1);
        }

        {
            Measuretime m("capture time");
            capture_addr = capture->capture_image();
        }
        me->capture_address = capture_addr;

        {
            Measuretime m("copy time");

            me->input_data = capture->get_img();

            me->capture_enabled.store(false);
            /* sync with inference thread*/
            unique_lock<mutex> lock(me->mtx_);
            me->wake_ = true;
            me->cv_.notify_all();
            std::cout << "[cam_thread]signaled for capture enable !!" << std::endl;

        }
#else
        /* _capture USB camera image and stop updating the _capture buffer */
        {
            Measuretime mmm("capture_time");
            capture_addr = capture->capture_image();
        }

#ifdef INFERENE_ON
        if (me->capture_enabled.load())
        {
            Measuretime m("capture enable proc time");
            me->capture_enabled.store(false);
            capture->sync_inference_buf_capture();
            me->capture_address = capture_addr;
            me->input_data = capture->get_img();
            /* sync with inference thread*/
            unique_lock<mutex> lock(me->mtx_);
            me->wake_ = true;
            me->cv_.notify_all();
        }
#endif
#endif

        me->_camera_frame_count.store(me->_camera_frame_count.load() + 1);


        if (0 == capture_addr)
        {
            fprintf(stderr, "[ERROR] Failed to _capture image from camera.\n");
            break;
        }
        else
        {
#ifdef SEND_CAMERA_ON
            uint8_t* send_image = capture->get_img();

            notify.img = me->get_send_image(send_image);
            {
                Measuretime m("Command create and send time[camera]");
                /* Send ImageNotify*/
                me->_server->send_command(notify.CreateRequest());
            }
#endif
        }
        /* IMPORTANT: Place back the image buffer to the _capture queue */
#ifdef SEQUENCTCIAL

        while (!me->capture_enabled.load())
        {
            usleep(1);
        }
        ret = capture->capture_qbuf();

#endif

#ifdef INFERENE_ON 
#ifndef SEQUENCTCIAL

        /* IMPORTANT: Place back the image buffer to the _capture queue */
        if (capture->get_buf_capture_index() != capture->get_inference_buf_capture_index())
        {
            {
                Measuretime m("Deque capture buf time");
                ret = capture->capture_qbuf();
            }
        }
#endif
#else
        {
            Measuretime m("Deque capture buf time");
            ret = capture->capture_qbuf();
        }
#endif
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to enqueue _capture buffer.\n");
            break;
        }
    } /*End of Loop*/

    /*To terminate the loop in AI Inference Thread.*/
    unique_lock<mutex> lock(me->mtx_);
    me->wake_ = true;
    me->cv_.notify_all();

    cout << "<<<<<<<<<<<<<<<<<<<<< Capture Thread Terminated >>>>>>>>>>>>>>>>>>" << endl;
    
    pthread_exit(NULL);
    me->_pthread_capture = 0;

    return NULL;
}

/**
 * @brief inference thread
 * @details inference and send results
 * @param arg pointer to itself
 * @return void* 
 */
void* RecognizeBase::inference_thread(void* arg)
{
    RecognizeBase* me = (RecognizeBase*)arg;

    shared_ptr<Camera> capture = me->_capture;

    /*Variable for getting Inference output data*/
    drpai_data_t drpai_data;
    /*Inference Variables*/
    int8_t inf_status = 0;
    drpai_data_t proc[DRPAI_INDEX_NUM];
    int32_t inf_cnt = -1;
    drpai_status_t drpai_status;
    /*Variable for checking return value*/
    int8_t ret = 0;
    /*Variable for Performance Measurement*/
    timespec start_time;
    timespec inf_end_time;
    float ai_time;
    printf("Inference Thread Starting\n");

    proc[DRPAI_INDEX_INPUT].address = me->drpai_address.data_in_addr;
    proc[DRPAI_INDEX_INPUT].size = me->drpai_address.data_in_size;
    proc[DRPAI_INDEX_DRP_CFG].address = me->drpai_address.drp_config_addr;
    proc[DRPAI_INDEX_DRP_CFG].size = me->drpai_address.drp_config_size;
    proc[DRPAI_INDEX_DRP_PARAM].address = me->drpai_address.drp_param_addr;
    proc[DRPAI_INDEX_DRP_PARAM].size = me->drpai_address.drp_param_size;
    proc[DRPAI_INDEX_AIMAC_DESC].address = me->drpai_address.desc_aimac_addr;
    proc[DRPAI_INDEX_AIMAC_DESC].size = me->drpai_address.desc_aimac_size;
    proc[DRPAI_INDEX_DRP_DESC].address = me->drpai_address.desc_drp_addr;
    proc[DRPAI_INDEX_DRP_DESC].size = me->drpai_address.desc_drp_size;
    proc[DRPAI_INDEX_WEIGHT].address = me->drpai_address.weight_addr;
    proc[DRPAI_INDEX_WEIGHT].size = me->drpai_address.weight_size;
    proc[DRPAI_INDEX_OUTPUT].address = me->drpai_address.data_out_addr;
    proc[DRPAI_INDEX_OUTPUT].size = me->drpai_address.data_out_size;

    /*DRP-AI Output Memory Preparation*/
    drpai_data.address = me->drpai_address.data_out_addr;
    drpai_data.size = me->drpai_address.data_out_size;

    printf("Inference Loop Starting\n");

    vector<uint8_t> inputimage;
    recognizeData_t data;

    /*Inference Loop Start*/
    while (me->_inf_running)
    {
        std::cout << "[inf_thread]waiting for inference start..." << std::endl;
        /*Checks if image frame from _capture Thread is ready.*/
        {
            Measuretime m("Inference start wait time");

            unique_lock<mutex> lock(me->mtx_);
            me->cv_.wait(lock, [me] {return me->wake_; });
            me->wake_ = false;
        }

        /*Registers physical address & size of input data to DRP-AI*/
        proc[DRPAI_INDEX_INPUT].address = (uintptr_t)me->capture_address;

        /*Gets inference starting time*/
        me->get_time(start_time);

        /*Start DRP-AI Driver*/
        errno = 0;
        ret = me->_drp->drp_start(&proc[0]);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to run DRPAI_START: errno=%d\n", errno);
            break;
        }
        inf_cnt++;
        printf("[START] Start DRP-AI inference...\n");
        printf("Inference ----------- No. %d\n", (inf_cnt + 1));


        ret = me->_drp->drp_pselect(NULL);
        if (0 == ret)
        {
            fprintf(stderr, "[ERROR] DRP-AI Inference pselect() Timeout: errno=%d\n", errno);
            break;
        }
        else if (0 > ret)
        {
            /*Checks if DRPAI pselect ended without issue*/
            fprintf(stderr, "[ERROR] DRP-AI Inference pselect() Error: errno=%d\n", errno);
            //ret = ioctl(me->drpai_fd, DRPAI_GET_STATUS, &drpai_status);
            ret = me->_drp->drp_getStatus(&drpai_status);
            if (-1 == ret)
            {
                fprintf(stderr, "[ERROR] Failed to run DRPAI_GET_STATUS : errno=%d\n", errno);
                break;
            }
        }
        else
        {
            /*Do nothing*/
        }
        /*Gets AI Inference End Time*/
        me->get_time(inf_end_time);
        /*Inference Time Result*/
        ai_time = (float)((me->timedifference_msec(start_time, inf_end_time)));
        print_measure_log("AI Inference Time", ai_time, "ms");


        /*Checks if DRPAI Inference ended without issue*/
        //inf_status = ioctl(me->drpai_fd, DRPAI_GET_STATUS, &drpai_status);
        inf_status = me->_drp->drp_getStatus(&drpai_status);
        if (0 == inf_status)
        {
            me->get_time(start_time);

            /*Process to read the DRPAI output data.*/
            shared_ptr<float> drpai_output_buf;
            drpai_output_buf.reset(new float[me->_outBuffSize], std::default_delete<float[]>());
            ret = me->get_result(me->_drp, drpai_output_buf, drpai_data.address, drpai_data.size);
            if (0 != ret)
            {
                fprintf(stderr, "[ERROR] Failed to get result from memory.\n");
                break;
            }


            me->get_time(inf_end_time);
            float get_time = (float)((me->timedifference_msec(start_time, inf_end_time)));
            print_measure_log("AI get output time", get_time, "ms");

            data.predict_image = me->input_data;
            data.predict_result = move(drpai_output_buf);
            data.drp_time_ms = ai_time;

            me->inference_postprocess(arg, data);


#ifdef SEQUENCTCIAL

#else
            {
                Measuretime m("Deque inference_capture_qbuf buf time");

                ret = capture->inference_capture_qbuf();
                if (0 != ret)
                {
                    fprintf(stderr, "[ERROR] Failed to enqueue _capture buffer.\n");
                    break;
                }
            }
#endif
            me->_ai_frame_count.store(me->_ai_frame_count.load() + 1);

            me->capture_enabled.store(true); /* Flag for _capture Thread. */

        }
        else
        {
            /* inf_status != 0 */
            fprintf(stderr, "[ERROR] DRPAI Internal Error: errno=%d\n", errno);
            break;
        }
    }
    /*End of Inference Loop*/

    /*To terminate the loop in _capture Thread.*/
    me->capture_enabled.store(true);

    cout << "<<<<<<<<<<<<<<<<<<<<< AI Inference Thread Terminated >>>>>>>>>>>>>>>>>>" << endl;
    pthread_exit(NULL);
    me->_pthread_ai_inf = 0;
    return NULL;
}

/**
 * @brief framerate_thread
 * @details framerate and cpu usage thread
 * @param arg pointer to itself
 * @return void* 
 */
void* RecognizeBase::framerate_thread(void* arg)
{
    RecognizeBase* me = (RecognizeBase*)arg;
    CPUUsage notify;
    while (me->_fps_runnning)
    {
        sleep(1);

        /* AI FPS */
        int32_t count = me->_ai_frame_count.load();
        me->_ai_frame_count.store(0);
        print_measure_log("------------------------------>AI FPS", count, "fps");
        /*Camerar FPS*/
        int32_t cam_count = me->_camera_frame_count.load();
        me->_camera_frame_count.store(0);
        print_measure_log("------------------------------>Camera FPS", cam_count, "fps");

        /* CPU usage*/
        string cpuUsage = me->_analyzer.get_cpu_usage(2);
        print_measure_log("CPU Usage", cpuUsage);

        /* Memory usage*/
        float memoryUsage = me->_analyzer.get_memory_usage();
        print_measure_log("Memory Usage", memoryUsage, "%");

        /* send CPU usage and Memory usage*/
        notify.cpu_usage = cpuUsage;
        notify.mem_usage = memoryUsage;

        me->_server->send_command(notify.CreateRequest());
    }
    cout << "<<<<<<<<<<<<<<<<<<<<< FPS Thread Terminated >>>>>>>>>>>>>>>>>>" << endl;
    pthread_exit(NULL);
    me->_pthread_framerate = 0;
    return NULL;
}
/**
 * @brief inference_postprocess
 * @details postprocess and send command
 * @param arg pointer to itself
 * @param data inference result data
 */
void RecognizeBase::inference_postprocess(void* arg, recognizeData_t& data)
{
    timespec start_time;
    timespec end_time;

    RecognizeBase* me = (RecognizeBase*)arg;

    me->get_time(start_time);
    {
        Measuretime m("Post process time");
        _model->inf_post_process(data.predict_result.get());
    }
    me->get_time(end_time);
    float post_time = (float)((me->timedifference_msec(start_time, end_time)));

    string b64;
    shared_ptr<PredictNotifyBase> notify;

#ifdef SEND_INFERENCE_RESULT_ON
    /* create send image (encoded to base64) */
    b64 = get_send_image(data.predict_image);
#endif

#ifdef COUT_INFERENCE_RESULT_ON
    _model->print_result();
#endif
    {
        Measuretime m("Create predict result time");
        notify = _model->get_command();
    }

    {
        Measuretime m("Command create and send time[predict]");

        /* Create websocket command*/
        notify->img = b64;
        notify->img_org_w = _model->_capture_w;
        notify->img_org_h = _model->_capture_h;
        notify->drp_time = data.drp_time_ms;
        notify->post_time = post_time;

        /*  Send websocket coomand*/
        me->_server->send_command(notify->CreateRequest());

    }

    data.predict_result.reset();
}

/**
 * @brief Get DRP-AI inference result
 * @details description
 * @param drp pointer to DRP class
 * @param[out] drpOutBuf pointer to store result
 * @param output_addr DRP output address 
 * @param output_size DRP output sizze
 * @return int8_t success:0
 */
int8_t RecognizeBase::get_result(shared_ptr<DRPProc> drp, shared_ptr<float> drpOutBuf, uint32_t output_addr, uint32_t output_size)
{
    drpai_data_t drpai_data;
    float drpai_buf[BUF_SIZE];
    drpai_data.address = output_addr;
    drpai_data.size = output_size;
    int32_t i = 0;
    int8_t ret = 0;

    errno = 0;
    /* Assign the memory address and size to be read */
    ret = drp->drp_assign(&drpai_data);
    if (-1 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_ASSIGN: errno=%d\n", errno);
        return -1;
    }

    float outsize_bufsize = drpai_data.size / BUF_SIZE;

    /* Read the memory via DRP-AI Driver and store the output to buffer */
    for (i = 0; i < outsize_bufsize; i++)
    {
        errno = 0;
        ret = drp->drp_read(drpai_buf, BUF_SIZE);
        if (-1 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            return -1;
        }
        memcpy(&(drpOutBuf.get()[BUF_SIZE / sizeof(float) * i]), drpai_buf, BUF_SIZE);
    }

    if (0 != (drpai_data.size % BUF_SIZE))
    {
        errno = 0;
        ret = drp->drp_read(drpai_buf, (drpai_data.size % BUF_SIZE));
        if (-1 == ret)
        {
            fprintf(stderr, "[ERROR] Failed to read via DRP-AI Driver: errno=%d\n", errno);
            return -1;
        }
        memcpy(&(drpOutBuf.get()[(drpai_data.size - (drpai_data.size % BUF_SIZE)) / sizeof(float)]), drpai_buf, (drpai_data.size % BUF_SIZE));
    }
    return 0;
}

/**
 * @brief end_all_threads
 * @details terminate all threads.
 * @return int32_t success :0
 */
int32_t RecognizeBase::end_all_threads()
{
    int32_t ret;
    int32_t ret_main;
    capture_enabled.store(true);

    _capture_running = false;
    _inf_running = false;
    _fps_runnning = false;

    if (0 != _pthread_capture)
    {
        ret = wait_join(&_pthread_capture, CAPTURE_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit _capture Thread on time.[%d]\n", ret);
            ret_main = -1;
        }
    }

    if (0 != _pthread_ai_inf)
    {
        ret = wait_join(&_pthread_ai_inf, AI_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit AI Inference Thread on time.[%d]\n", ret);
            ret_main = -1;
        }
    }

    if (0 != _pthread_framerate)
    {
        ret = wait_join(&_pthread_framerate, FRAMERATE_THREAD_TIMEOUT);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to exit Framerate Thread on time.\n");
            ret_main = -1;
        }
    }


    std::cout << "********************** ALL THREAD END *********************" << std::endl;

    return ret_main;
}

/**
 * @brief wait_join
 * @details wait for thread
 * @param p_join_thread target thread.
 * @param join_time timeout time[sec]
 * @return int8_t success:0
 */
int8_t RecognizeBase::wait_join(pthread_t* p_join_thread, uint32_t join_time)
{
    int8_t ret_err;
    struct timespec join_timeout;
    ret_err = clock_gettime(CLOCK_REALTIME, &join_timeout);
    if (0 == ret_err)
    {
        join_timeout.tv_sec += join_time;
        ret_err = pthread_timedjoin_np(*p_join_thread, NULL, &join_timeout);
    }
    return ret_err;
}
/**
 * @brief timedifference_msec
 * @details calc time diffrence t0 and t1
 * @param t0 time (start)
 * @param t1 time (end)
 * @return double diff time[ms]
 */
double RecognizeBase::timedifference_msec(timespec t0, timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}

/**
 * @brief get current time
 * @details description
 * @param[out] time_t reference to store current time
 * @return int32_t success:0
 */
int32_t RecognizeBase::get_time(timespec& time_t)
{
    int32_t ret = timespec_get(&time_t, TIME_UTC);
    if (0 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to get Inference Start Time\n");
    }
    return ret;
}

/**
 * @brief close_camera
 * @details Close USB Camea
 */
void RecognizeBase::close_camera()
{
    std::cout << "close_camera" << std::endl;
    /*Close USB Camera.*/
    if (NULL != _capture && 0 != _pthread_capture)
    {
        int8_t ret = _capture->close_camera();
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to close USB Camera.\n");
        }
        cout << "Camera Closed." << endl;
    }
}

/**
 * @brief get_send_image
 * @details Get base64 jpeg image
 * @param image inpuy yuyv image
 * @return string base64 jpeg image
 */
string RecognizeBase::get_send_image(uint8_t* image)
{
    vector<uint8_t> output;
    vector<uint8_t> input;
    {
        Measuretime mm("Jpeg compress Time");
        ImageConverter::compress_jpeg_turbo(image, _model->_capture_w, _model->_capture_h, output, JPEG_QUALUTY);
    }

    ////////////////////
    // jpeg->Base64
    string b64;
    {
        Measuretime m("Base64 encode time");
        b64 = lwsock::b64encode(output.data(), output.size());
    }
    return b64;
}
