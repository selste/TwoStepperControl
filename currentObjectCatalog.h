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

private:
    long numberOfObjects;
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
