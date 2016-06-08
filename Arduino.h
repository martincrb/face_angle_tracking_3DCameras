#pragma once
#include <qstring.h>
#include "Serial.h"
class Arduino
{
public:
	Arduino();
	~Arduino();
	void init(wchar_t *COMport);
	void close();
	void sendData(QString data);

private:
	Serial *SP;
};

