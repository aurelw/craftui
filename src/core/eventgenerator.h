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


#ifndef __EVENT_GENERATOR__H__
#define __EVENT_GENERATOR__H__

#include <chrono>
#include <ctime>
#include <map>

#include "elementvisitor.h"
#include "button.h"
#include "slider.h"

class EventGenerator : public ElementVisitor {

    public:

        virtual void visit(Button& button) override;
        virtual void visit(Slider& slider) override;

    private:

        typedef std::chrono::time_point<std::chrono::system_clock> TimePoint; 
        typedef std::chrono::duration<double> TimeDuration;

        bool isElementTriggered(Element& element, const TimePoint& now);
        bool isElementUntriggered(Element& element, const TimePoint& now);
        void updateBounceData(const std::string& id, 
            bool isTriggered, bool isUntriggered, const TimePoint& now);

        std::map<std::string, TimePoint> lastTriggered;
        std::map<std::string, TimePoint> lastUntriggered;
        std::map<std::string, bool> inTrigger;

        // minim duration between two clicks
        const float clickRate = 1.0;
};


#endif

