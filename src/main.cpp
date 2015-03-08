#include <QApplication>
#include "ControlWindow.h"

using namespace server;

int main(int argc, char** argv) {
    QApplication app(argc, argv);

    ControlWindow s(argc, argv);

	return app.exec();
}

