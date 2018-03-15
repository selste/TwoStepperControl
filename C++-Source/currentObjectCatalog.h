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
// a class for organizing the catalogs of TSC

#ifndef CURRENTOBJECTCATALOG_H
#define CURRENTOBJECTCATALOG_H

#include <stdio.h>
#include <QString>
#include <vector>

class currentObjectCatalog {
public:
    currentObjectCatalog(QString);
    ~currentObjectCatalog(void);
    long getNumberOfObjects(void);
    std::string getNamesOfObjects(long);
    float getRADec(long);
    float getDeclDec(long);
    long getEpoch(void);

private:
    long numberOfObjects; // the numbe rof objects in the catalog
    long epoch; // the epoch of the catalog
    struct catalogEntry {
        long oindex;
        std::string oname;
        std::string oconstellation;
        float oRADec;
        float oDeclDec;
    };
    std::vector<catalogEntry> currentCatalogObjects; // wohooo - wolfi uses templates!!!
};

#endif // CURRENTOBJECTCATALOG_H
