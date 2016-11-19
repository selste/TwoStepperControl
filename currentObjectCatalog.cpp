#include "currentObjectCatalog.h"
#include <QDebug>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

//-------------------------------------------------
currentObjectCatalog::currentObjectCatalog(QString filename) {
// reading a .csv file with the special format
// number of datasets
// Constellation,ObjectName,RA[h],RA[min],RA[sec],Decl-Sign,Decl-Degree,Decl-Arcmin,Decl-Arcsec
// ....
    std::string constellation, name, decsign;   // string data
    long rah,ram,ras,decldeg,declamin,declasec;  // right ascension and declination
    short decSignNum;   // the sigm of declination as a number
    float radec, decldec;   // right ascension and declination as decimals
    long counter;           // guess what

    char delimiter(',');    // data are .csv - comma-separated
    QByteArray ba = filename.toLatin1();    //convert QString to QByteArray
    const char *cfilename = ba.data();      // convert this to a C-string
    std::ifstream infile(cfilename);        // open a file to read
    std::string line;   // define a line that is read until \n is encountered
    std::getline(infile, line);     // read that line
    std::istringstream iss(line);   // convert it to a stream so that the first line
                                    // can be converted to long
    iss >> this->numberOfObjects;
    // now read the other lines...
    for (counter = 0; counter < this->numberOfObjects; counter++) {
        std::getline(infile, constellation, delimiter);     //constellation
        std::getline(infile, name, delimiter);              // object name
        std::getline(infile, line, delimiter);              // RA-hrs
        std::istringstream issrah(line);
        issrah >> rah;
        std::getline(infile, line, delimiter);              // RA-mins
        std::istringstream issram(line);
        issram >> ram;
        std::getline(infile, line, delimiter);              // RA-secs
        std::istringstream issras(line);
        issras >> ras;
        std::getline(infile, line, delimiter);  // this is the sign of declination
        if (line.compare("-") == 0) {           // convert the sign-char to a number
           decSignNum = -1;
        } else {
            decSignNum = 1;
        }
        std::getline(infile, line, delimiter);  // Declination Degrees
        std::istringstream issdecldeg(line);
        issdecldeg >> decldeg;
        std::getline(infile, line, delimiter);  // Declination minutes of arc
        std::istringstream issdeclmin(line);
        issdeclmin >> declamin;
        std::getline(infile, line);             // Declination minutes of arc
        std::istringstream issdeclsec(line);
        issdeclsec >> declasec;
        radec = (rah+ram/60.0+ras/3600.0)*15.0;
        decldec = decSignNum*(decldeg+declamin/60.0+declasec/3600.0);
        currentCatalogObjects.push_back(catalogEntry());
        currentCatalogObjects[counter].oindex=counter;
        currentCatalogObjects[counter].oname=name;
        currentCatalogObjects[counter].oconstellation=constellation;
        currentCatalogObjects[counter].oRADec=radec;
        currentCatalogObjects[counter].oDeclDec=decldec;
    }

    infile.close(); // close the file
}

//-------------------------------------------------
currentObjectCatalog::~currentObjectCatalog() {
    this->currentCatalogObjects.clear();
}

//-------------------------------------------------
long currentObjectCatalog::getNumberOfObjects(void) {
    return numberOfObjects;
}

//-------------------------------------------------
std::string currentObjectCatalog::getNamesOfObjects(long index) {
    std::string catStr;

    if (currentCatalogObjects[index].oname.length() != 0) {
        catStr= currentCatalogObjects[index].oname + '-' +
                currentCatalogObjects[index].oconstellation;
    } else {
        catStr= currentCatalogObjects[index].oconstellation;
    }
    return catStr;
}

//--------------------------------------------------
float currentObjectCatalog::getRADec(long index) {
    return currentCatalogObjects[index].oRADec;
}

//--------------------------------------------------
float currentObjectCatalog::getDeclDec(long index) {
    return currentCatalogObjects[index].oDeclDec;
}

//--------------------------------------------------

