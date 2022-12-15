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
* File Name    : command_base.h
* Version      : 1.00
* Description  : RZ/V DRP-AI Sample Application for USB Camera version
***********************************************************************************************************************/

#pragma once
#ifndef COMMADBASE_H
#define COMMADBASE_H
/*****************************************
* Includes
******************************************/

#include "../includes.h"
#include "command_create_helper.h"
using namespace std;
class CommandBase :public CommandCreateHelper
{
public:
    CommandBase(string name) :command_name(name) {}
    virtual ~CommandBase() {}
    virtual std::string CreateRequest(void)
    {
        return CommandCreateHelper::SeliarizeCommandBody<CommandBase>(*this);
    }

    template<class Archive>
    void save(Archive& archive) const
    {
        archive(CEREAL_NVP(command_name));
    }

    template<class Archive>
    void load(Archive& archive)
    {
        archive(CEREAL_NVP(command_name));
    }
public:
    string command_name;
};

#endif
