/*
   * (C) 2015, Aurel Wildfellner
   *
   * This file is part of CraftUI.
   *
   * Beholder is free software: you can redistribute it and/or modify
   * it under the terms of the GNU General Public License as published by
   * the Free Software Foundation, either version 3 of the License, or
   * (at your option) any later version.
   *
   * Beholder is distributed in the hope that it will be useful,
   * but WITHOUT ANY WARRANTY; without even the implied warranty of
   * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   * GNU General Public License for more details.
   *
   * You should have received a copy of the GNU General Public License
   * along with CraftUI. If not, see <http://www.gnu.org/licenses/>. */

#include <iostream>
#include <unistd.h>

#include "ipcserver.h"
#include "uievents.pb.h"



int main(int argc, char **argv) {

    IPCServer ipcServ("tcp://127.0.0.1:9001");
    bool connected = ipcServ.bind();
    std::cout << "Connected: " <<  connected << std::endl;

    /* create two events */
    std::shared_ptr<craftui::Event> ev0(new craftui::Event);
    ev0->set_elementtype(ev0->BUTTON);
    ev0->set_id("buttonGREEN");
    ev0->set_trigger(ev0->TRIGGERED);

    std::shared_ptr<craftui::Event> ev1(new craftui::Event);
    ev1->set_elementtype(ev1->SLIDER);
    ev1->set_id("buttonRED");
    ev1->set_trigger(ev1->TRIGGERED);

    for (;;) {
        ipcServ.sendEvent(ev0);
        std::cout << "Sent ev0." << std::endl;
        sleep(1);
        ipcServ.sendEvent(ev1);
        std::cout << "Sent ev1." << std::endl;
        sleep(1);
    }

}

