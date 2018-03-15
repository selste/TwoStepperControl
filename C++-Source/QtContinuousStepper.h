// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-18, wolfgang birkfellner
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

//---------------------------------------------------


// a child class of QtKineticStepper; this one does, in addition, carry a permanent motion. in TSC, this is the right ascension drive ...
#ifndef QTCONTINUOUSSTEPPER_H
#define QTCONTINUOUSSTEPPER_H

#include "QtKineticStepper.h"

class QtContinuousStepper:public QtKineticStepper {
protected:
    short RADirection = 1; // a value that takes +/-1; it inverts continuous motion, for instance when moving to the southern hemisphere
public:
    void startTracking(void); // start continuous motion to compensate for earth's rotation
    bool travelForNSteps(long,short,int,bool) override; // override this function from "QtKineticStepper" so that "RADirection" is also respected ...
    void setRADirection(short); // switch "RADirection"
};
#endif // QTCONTINUOUSSTEPPER_H
