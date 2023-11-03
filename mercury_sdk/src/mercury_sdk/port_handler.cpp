/*******************************************************************************
* Copyright (C) 2021 <Robot Articulation/code@robotarticulation.com> 
*
* Source files modified to support the Mercury range of digital servo motors from Robot Articulation
*******************************************************************************/

/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Original author: Zerom, Leon (RyuWoon Jung) */

#if defined(__linux__)
#include "port_handler.h"
#include "port_handler_linux.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "port_handler.h"
#include "port_handler_windows.h"
#endif

using namespace mercury;

PortHandler *PortHandler::getPortHandler(const char *port_name)
{
#if defined(__linux__)
  return (PortHandler *)(new PortHandlerLinux(port_name));
#elif defined(_WIN32) || defined(_WIN64)
  return (PortHandler *)(new PortHandlerWindows(port_name));
#endif
}
