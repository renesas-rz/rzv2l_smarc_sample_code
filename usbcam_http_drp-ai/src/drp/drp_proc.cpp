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
* File Name    : drp_proc.cpp
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

/*****************************************
* Includes
******************************************/
#include "drp_proc.h"
#include <iostream>
#include <signal.h>

/**
 * @brief drp_open
 * @details Open DRP-AI driver and store handle.
 * @return int8_t 
 */
int8_t DRPProc::drp_open()
{
    /*DRP-AI Driver Open*/
    errno = 0;
    _drpai_fd = open("/dev/drpai0", O_RDWR);
    if (0 > _drpai_fd)
    {
        fprintf(stderr, "[ERROR] Failed to open DRP-AI Driver: errno=%d\n", errno);
        return -1;
    }
    else
    {
        std::cout << "[OK] DRP-AI Driver Open OK" << std::endl;
    }
    return 0;
}

/**
 * @brief drp_close
 * @details Close DRP-AI driver
 * @return int8_t 
 */
int8_t DRPProc::drp_close()
{
    /*Close DRP-AI Driver.*/
    if (0 < _drpai_fd)
    {
        errno = 0;
        int32_t ret = close(_drpai_fd);
        if (0 != ret)
        {
            fprintf(stderr, "[ERROR] Failed to close DRP-AI Driver: errno=%d\n", errno);
        }
        else
        {
            _drpai_fd = 0;
        }
        return ret;
    }
    return 0;
}

/**
 * @brief read_addrmap_txt
 * @details Loads address and size of DRP-AI Object files into struct addr.
 * @param addr_file filename of addressmap file (from DRP-AI Object files)
 * @param drpai_address pointer to store address
 * @return int8_t 0 if succeeded
 *                 not 0 otherwise
 */
int8_t DRPProc::read_addrmap_txt(std::string addr_file, st_addr_t* drpai_address)
{
    std::string str;
    uint32_t l_addr;
    uint32_t l_size;
    std::string element, a, s;

    std::ifstream ifs(addr_file);
    if (ifs.fail())
    {
        fprintf(stderr, "[ERROR] Failed to open address map list : %s\n", addr_file.c_str());
        return -1;
    }

    while (getline(ifs, str))
    {
        std::istringstream iss(str);
        iss >> element >> a >> s;
        l_addr = strtol(a.c_str(), NULL, 16);
        l_size = strtol(s.c_str(), NULL, 16);

        if ("drp_config" == element)
        {
            drpai_address->drp_config_addr = l_addr;
            drpai_address->drp_config_size = l_size;
        }
        else if ("desc_aimac" == element)
        {
            drpai_address->desc_aimac_addr = l_addr;
            drpai_address->desc_aimac_size = l_size;
        }
        else if ("desc_drp" == element)
        {
            drpai_address->desc_drp_addr = l_addr;
            drpai_address->desc_drp_size = l_size;
        }
        else if ("drp_param" == element)
        {
            drpai_address->drp_param_addr = l_addr;
            drpai_address->drp_param_size = l_size;
        }
        else if ("weight" == element)
        {
            drpai_address->weight_addr = l_addr;
            drpai_address->weight_size = l_size;
        }
        else if ("data_in" == element)
        {
            drpai_address->data_in_addr = l_addr;
            drpai_address->data_in_size = l_size;
        }
        else if ("data" == element)
        {
            drpai_address->data_addr = l_addr;
            drpai_address->data_size = l_size;
        }
        else if ("data_out" == element)
        {
            drpai_address->data_out_addr = l_addr;
            drpai_address->data_out_size = l_size;
        }
        else if ("work" == element)
        {
            drpai_address->work_addr = l_addr;
            drpai_address->work_size = l_size;
        }
        else
        {
            /*Ignore other space*/
        }
    }

    return 0;
}

/**
 * @brief load_data_to_mem
 * @details Load DRP-AI Data from file.
 * @param data DRP-AI data file path
 * @param from DRP-AI data addrerss 
 * @param size Data size
 * @return int8_t 
 */
int8_t DRPProc::load_data_to_mem(std::string data, uint32_t from, uint32_t size)
{
    int8_t ret_load_data = 0;
    int8_t obj_fd;
    uint8_t drpai_buf[BUF_SIZE];
    drpai_data_t drpai_data;
    uint8_t assign_ret = 0;
    uint8_t rw_ret = 0;
    int32_t i = 0;

    printf("Loading : %s\n", data.c_str());
    errno = 0;
    obj_fd = open(data.c_str(), O_RDONLY);
    if (0 > obj_fd)
    {
        fprintf(stderr, "[ERROR] Failed to open: %s errno=%d\n", data.c_str(), errno);
        ret_load_data = -1;
        goto end;
    }

    drpai_data.address = from;
    drpai_data.size = size;

    errno = 0;
    assign_ret = drp_assign(&drpai_data);
    if (-1 == assign_ret)
    {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_ASSIGN: errno=%d\n", errno);
        ret_load_data = -1;
        goto end;
    }

    for (i = 0; i < (int32_t)(drpai_data.size / BUF_SIZE); i++)
    {
        errno = 0;
        rw_ret = read(obj_fd, drpai_buf, BUF_SIZE);
        if (0 > rw_ret)
        {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", data.c_str(), errno);
            ret_load_data = -1;
            goto end;
        }
        rw_ret = write(_drpai_fd, drpai_buf, BUF_SIZE);
        if (-1 == rw_ret)
        {
            fprintf(stderr, "[ERROR] Failed to write via DRP-AI Driver: errno=%d\n", errno);
            ret_load_data = -1;
            goto end;
        }
    }
    if (0 != (drpai_data.size % BUF_SIZE))
    {
        errno = 0;
        rw_ret = read(obj_fd, drpai_buf, (drpai_data.size % BUF_SIZE));
        if (0 > rw_ret)
        {
            fprintf(stderr, "[ERROR] Failed to read: %s errno=%d\n", data.c_str(), errno);
            ret_load_data = -1;
            goto end;
        }
        rw_ret = write(_drpai_fd, drpai_buf, (drpai_data.size % BUF_SIZE));
        if (-1 == rw_ret)
        {
            fprintf(stderr, "[ERROR] Failed to write via DRP-AI Driver: errno=%d\n", errno);
            ret_load_data = -1;
            goto end;
        }
    }
    goto end;

end:
    if (0 < obj_fd)
    {
        close(obj_fd);
    }
    return ret_load_data;
}

/**
 * @brief load_drpai_data
 * @details Loads DRP-AI Object files to memory via DRP-AI Driver.
 * @param drpai_address 
 * @param drpai_file_path 
 * @return int8_t 
 */
int8_t DRPProc::load_drpai_data(st_addr_t& drpai_address, std::string drpai_file_path[])
{
    std::cout << "load_drpai_data" << std::endl;

    uint32_t addr = 0;
    uint32_t size = 0;
    uint8_t i = 0;
    int8_t ret = 0;
    double diff = 0;

    struct timespec start_time, stop_time;
    /*Start load time*/
    ret = timespec_get(&start_time, TIME_UTC);
    if (0 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to run timespect_get().\n");
        return -1;
    }
    printf("[START] Loading DRP-AI Data...\n");
    for (i = 0; i < 5; i++)
    {
        switch (i)
        {
        case (INDEX_W):
            addr = drpai_address.weight_addr;
            size = drpai_address.weight_size;
            break;
        case (INDEX_C):
            addr = drpai_address.drp_config_addr;
            size = drpai_address.drp_config_size;
            break;
        case (INDEX_P):
            addr = drpai_address.drp_param_addr;
            size = drpai_address.drp_param_size;
            break;
        case (INDEX_A):
            addr = drpai_address.desc_aimac_addr;
            size = drpai_address.desc_aimac_size;
            break;
        case (INDEX_D):
            addr = drpai_address.desc_drp_addr;
            size = drpai_address.desc_drp_size;
            break;
        default:
            break;
        }
        std::cout << "addr = " << addr << std::endl;
        std::cout << "size = " << size << std::endl;

        ret = load_data_to_mem(drpai_file_path[i], addr, size);
        if (0 > ret)
        {
            fprintf(stderr, "[ERROR] Failed to load data from memory: %s\n", drpai_file_path[i].c_str());
            return -1;
        }
    }

    /*Stop load time*/
    ret = timespec_get(&stop_time, TIME_UTC);
    if (0 == ret)
    {
        fprintf(stderr, "[ERROR] Failed to run timespect_get().\n");
        return -1;
    }
    diff = timedifference_msec(start_time, stop_time);
    printf("[END] Loading DRP-AI Data : Total loading time %f s\n", diff * 0.001);
    return 0;
}

/**
 * @brief drp_pselect
 * 
 * @param sigset 
 * @return int8_t 
 */
int8_t DRPProc::drp_pselect(sigset_t* sigset)
{
    /*Setup pselect settings*/
    sigset_t sigsett;
    struct timespec tv;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(_drpai_fd, &rfds);
    tv.tv_sec = DRPAI_TIMEOUT;
    tv.tv_nsec = 0 ;
    sigemptyset(&sigsett);
    sigaddset(&sigsett, SIGUSR1);
    /*Wait Till The DRP-AI Ends*/

    return  pselect(_drpai_fd + 1, &rfds, NULL, NULL, &tv, &sigsett);
}

/**
 * @brief drp_start
 * @details Start DRP proc
 * @param drpData 
 * @return int8_t 
 */
int8_t DRPProc::drp_start(drpai_data_t drpData[])
{
    /*Start DRP-AI Driver*/
    errno = 0;
    int32_t ret = ioctl(_drpai_fd, DRPAI_START, &drpData[0]);
    if (0 != ret)
    {
        fprintf(stderr, "[ERROR] Failed to run DRPAI_START: errno=%d\n", errno);
    }
    return ret;
}

/**
 * @brief drp_getStatus
 * @details get drp status
 * @param drpai_status 
 * @return int8_t 
 */
int8_t DRPProc::drp_getStatus(drpai_status_t* drpai_status)
{
    return ioctl(_drpai_fd, DRPAI_GET_STATUS, drpai_status);
}

/**
 * @brief drp_assign
 * @details Assign drp data
 * @param drpai_data 
 * @return int8_t 
 */
int8_t DRPProc::drp_assign(drpai_data_t* drpai_data)
{
    return  ioctl(_drpai_fd, DRPAI_ASSIGN, drpai_data);
}

/**
 * @brief drp_read
 * @details Read DRP data
 * @param outDataBuff 
 * @param buffSize 
 * @return int8_t 
 */
int8_t DRPProc::drp_read(void* outDataBuff, int32_t buffSize)
{
    return read(_drpai_fd, outDataBuff, buffSize);
}

/**
 * @brief timedifference_msec
 * @details  computes the time difference in ms between two moments
 * @param t0 start time
 * @param t1 stop time
 * @return double the time difference in ms
 */
double DRPProc::timedifference_msec(struct timespec t0, struct timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}


