#pragma once
//Because MS api *****
#include <windows.h>
#include <dshow.h>
#include <qstringlist.h>

#pragma comment(lib, "strmiids.lib")

static class MSUtils
{
public:
	MSUtils();
	~MSUtils();
	
	static void getInputDevices(QStringList &inputDevices);

private:
	static HRESULT EnumerateDevices(REFGUID category, IEnumMoniker **ppEnum);
	static void DisplayDeviceInformation(IEnumMoniker *pEnum);
	static void FillDeviceInformation(IEnumMoniker *pEnum, QStringList &inputDevices);

	bool COMIsLoaded;
};

