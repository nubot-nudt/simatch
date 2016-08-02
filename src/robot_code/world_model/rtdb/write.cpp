/*
 * Frederico Miguel Santos - frederico.miguel.santos@gmail.com
 * CAMBADA robotic soccer team - www.ieeta.pt/atri/cambada
 * University of Aveiro
 * Copyright (C) 2009
 *
 * This file is part of RTDB middleware.
 * http://code.google.com/p/rtdb/
 *
 * RTDB middleware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTDB middleware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTDB middleware.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdio.h>
#include <signal.h>
#include "nubot/rtdb/rtdb_api.h"
#include "nubot/rtdb/rtdb_user.h"
#include "nubot/world_model/teammatesinfo.h"
#include "nubot/core/core.hpp"

int end = 0;

// *************************
//   signal catch
// *************************
static void signal_catch(int sig)
{
    if (sig == SIGINT)
        end = 1;
}

// *************************
//   main function
// *************************
int main(int argc, char **argv)
{
    nubot::Teammatesinfo rtdb_teammate;
    rtdb_teammate.robot_info_.setID(3);
    rtdb_teammate.robot_info_.setLocation(nubot::DPoint(10,10));

    if(signal(SIGINT, signal_catch) == SIG_ERR)
    {
        printf("Error registering signal handler");
        return -1;
    }

    if(DB_init() != 0)
        return -1;

    while(end == 0)
    {
        if(DB_put(TEAMMATESINFO, &rtdb_teammate) == -1)
        {
            DB_free();
            return -1;
        }

    }
    DB_free();
    return 0;
}

