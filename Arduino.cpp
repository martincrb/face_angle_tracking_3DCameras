#include "Arduino.h"


Arduino::Arduino()
{
}


Arduino::~Arduino()
{
}

void Arduino::init(wchar_t *COMport) {
	SP = new Serial(COMport);
	
}
void Arduino::close() {
	delete SP;
}
void Arduino::sendData(QString data) {
	SP->WriteData((char *) data.toStdString().c_str(), 8);
}