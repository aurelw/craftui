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



#include "eventgenerator.h"



void EventGenerator::visit(Button& button) {
    std::cout << "Event: " << button.id << std::endl;
    TimePoint now = std::chrono::system_clock::now();
    bool isTriggered = isElementTriggered(button, now);
    bool isUntriggered = isElementUntriggered(button, now);
    updateBounceData(button.id, isTriggered, isUntriggered, now);

    if (isTriggered || isUntriggered) {
        std::shared_ptr<craftui::Event> ev(new craftui::Event);
        ev->set_elementtype(ev->BUTTON);
        ev->set_id(button.id);
        if (isTriggered) {
            ev->set_trigger(ev->TRIGGERED);
        } else if (isUntriggered) {
            ev->set_trigger(ev->UNTRIGGERED);
        }

        ipcServer.sendEvent(ev);
    }
}


void EventGenerator::visit(Slider& slider) {
    TimePoint now = std::chrono::system_clock::now();
    bool isTriggered = isElementTriggered(slider, now);
    bool isUntriggered = isElementUntriggered(slider, now);
    updateBounceData(slider.id, isTriggered, isUntriggered, now);

    if (isTriggered || isUntriggered) {
        std::shared_ptr<craftui::Event> ev(new craftui::Event);
        ev->set_elementtype(ev->SLIDER);
        ev->set_id(slider.id);
        if (isTriggered) {
            ev->set_trigger(ev->TRIGGERED);
        } else if (isUntriggered) {
            ev->set_trigger(ev->UNTRIGGERED);
        }
        //TODO add slider position to event and handle intrigger state

        ipcServer.sendEvent(ev);
    }
}


void EventGenerator::updateBounceData(const std::string& id, 
        bool isTriggered, bool isUntriggered, const TimePoint& now)
{
    if (isTriggered) {
        lastTriggered[id] = now;
        inTrigger[id] = true;
    } else if (isUntriggered) {
        lastUntriggered[id] = now;
        inTrigger[id] = false;
    }
}


bool EventGenerator::isElementTriggered(Element& element, const TimePoint& now) {
    bool eleTriggered = element.isTriggered();

    /* if element is already in trigger, not newly triggered */
    if (inTrigger.count(element.id) && inTrigger[element.id]) {
            return false;
    }

    /* not in triggered, check for bouncing */
    if (eleTriggered) {
        if (lastUntriggered.count(element.id)) {
            TimePoint last = lastUntriggered[element.id];
            TimeDuration duration = now-last;
            return (duration.count() > clickRate/2.);
        } else {
            return true;
        }
    } else {
        return false;
    }
    
}


bool EventGenerator::isElementUntriggered(Element& element, const TimePoint& now) {
    bool eleTriggered = element.isTriggered();

    /* if the element is not inTrigger it can't be untriggered */
    if (!inTrigger.count(element.id) || !inTrigger[element.id]) {
        return false;
    }

    if (!eleTriggered) {
        if (lastTriggered.count(element.id)) {
            TimePoint last = lastTriggered[element.id];
            TimeDuration duration = now-last;
            return (duration.count() > clickRate/2.);
        } else {
            return true;
        }
    } else {
        return false;
    }

}

