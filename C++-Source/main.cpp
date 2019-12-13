// this code is part of "TSC", a free control software for astronomical telescopes
// Copyright (C)  2016-19, wolfgang birkfellner, mark sproul
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
// Edit history:
//  Dec. 3., 2018 <MLS> added command line options

#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <stdio.h>
#include "tsc_globaldata.h"

int main(int argc, char *argv[]) {
    int ii;
    bool framelessWindow = true;
    QApplication a(argc, argv);
    MainWindow w;

    for (ii=1; ii < argc; ii++) {
        if (argv[ii][0] == '-') {
            switch (argv[ii][1]) {
            case 'w': framelessWindow = false; break;
            }
        }
    }
    if (framelessWindow == true) {
        w.setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    }
    w.show();

    QFile f(":qdarkstyle/style.qss");
    if (!f.exists()) {
        qDebug() << "Unable to set stylesheet, file not found\n";
    }
    else
    {
        f.open(QFile::ReadOnly | QFile::Text);
        QTextStream ts(&f);
        qApp->setStyleSheet(ts.readAll());
    }

    return a.exec();
}
