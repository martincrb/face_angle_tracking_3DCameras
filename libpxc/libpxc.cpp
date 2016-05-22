/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include <windows.h>
#include <tchar.h>

// In order to avoid linking to STD runtime for libpxc.lib we are not using types defined in PXC API
//#include "pxcsession.h"
#include "pxcversion.h"

#define RSSDK_REG_MAIN       TEXT("Core")
#define RSSDK_REG_LOCAL      TEXT("LocalRuntime")

#define LOAD_LIBRARY_FLAGS  0x00001100 

#ifdef _DEBUG
#define SESSION_RELATIVE_PATH  (sizeof(void*) == 4 ? L"\\bin\\win32_debug\\libpxcsession.dll" : L"\\bin\\x64_debug\\libpxcsession.dll")
#else
#define SESSION_RELATIVE_PATH  (sizeof(void*) == 4 ? L"\\bin\\win32\\libpxcsession.dll" : L"\\bin\\x64\\libpxcsession.dll")
#endif

#define PRINT_INFO(PARAMS)

#define PXCAPI __stdcall

class PXCSession;

extern "C" PXCSession* PXCAPI PXCSession_Create(void);

static PXCSession *LoadSessionLibrary(wchar_t *filepath, int options) {
	PXCSession *instance = 0;
	int sts = -1; // PXC_STATUS_FEATURE_UNSUPPORTED;
    HMODULE module = LoadLibrary(filepath);
    if (module) {
        PRINT_INFO((L"The SDK INFO: Loading session library %s\n", filepath));

        typedef int (PXCAPI *FUNC_PXCSession_CreateExt)(int version_major, int version_minor, int version_build, int reserved, int options, int reserved2, PXCSession **instance);
        FUNC_PXCSession_CreateExt pPXCSession_CreateExt = (FUNC_PXCSession_CreateExt)GetProcAddress(module, "PXCSession_CreateExt");
        if (pPXCSession_CreateExt) {
            sts = (*pPXCSession_CreateExt)(PXC_VERSION_MAJOR, PXC_VERSION_MINOR, PXC_VERSION_BUILD, 0, options, 0, &instance);
        }
    }
    if (sts >= 0 /*PXC_STATUS_NO_ERROR*/)
    {
        PRINT_INFO((L"The SDK INFO: Loaded session library: %s\n", filepath));
    } else
    {
        PRINT_INFO((L"The SDK INFO: FAILED to load session library: %s\n", filepath));
    }
    return instance;
}


PXCSession* PXCAPI PXCSession_Create(void) {
    LONG err_code;
    HKEY key = 0;
    DWORD type = 0;
    
    // try LocalRuntime registry key if set
    for (int i = 0; i < (sizeof(void*)/4); i++)
    {
        err_code = RegOpenKeyEx(HKEY_LOCAL_MACHINE, (!i) ? RSSDK_REG_DEV TEXT("\\Dispatch") : RSSDK_REG_DEV32 TEXT("\\Dispatch"), 0, KEY_READ, &key);
        if (err_code != ERROR_SUCCESS) continue;

        wchar_t local_path[MAX_PATH] = L"";
        DWORD size = sizeof(local_path);
        err_code = RegGetValue(key, 0, RSSDK_REG_LOCAL, RRF_RT_REG_SZ, &type, local_path, &size);
        RegCloseKey(key);
        if (err_code != ERROR_SUCCESS || !local_path[0]) continue;

        wchar_t filepath[MAX_PATH] = L"";
        if (local_path[0] == '.') // relative path
        {
            // get module folder
            HMODULE handle = 0;
            GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCWSTR)LoadSessionLibrary, &handle);
            GetModuleFileName(handle,filepath,MAX_PATH);
            wchar_t *pSlash=wcsrchr(filepath,L'\\');
            if (pSlash) pSlash[1]=0;
            PRINT_INFO((L"The SDK INFO: Application folder: %s\n", filepath));
            wcscat_s<MAX_PATH>(filepath,local_path);
            wcscat_s<MAX_PATH>(filepath,L"\\libpxcsession.dll");
        } else // absolute path
        {
            wcscpy_s<MAX_PATH>(filepath,local_path);
            wcscat_s<MAX_PATH>(filepath,SESSION_RELATIVE_PATH);
        }
        PXCSession *session=LoadSessionLibrary(filepath, 1);
        if (session) return session;

        // try __FILE__ at compilation time
        wchar_t filename[MAX_PATH];
        MultiByteToWideChar(CP_UTF8, 0, __FILE__, -1, filename, sizeof(filename));
        wchar_t *pSlash=wcsrchr(filename,L'\\');
        if (pSlash) *pSlash=0;
        wcscat_s(filename, MAX_PATH, L"\\..\\..");
        wcscat_s(filename, MAX_PATH, SESSION_RELATIVE_PATH);
        session = LoadSessionLibrary(filename, 1);
        if (session) return session;
    }

    // standard registry key set by SDK installer
    wchar_t dll_path[MAX_PATH] = L"";
    err_code = RegOpenKeyEx(HKEY_LOCAL_MACHINE, RSSDK_REG_DISPATCH, 0, KEY_READ, &key);
    if (err_code == ERROR_SUCCESS) {
        DWORD size = sizeof(dll_path);
        RegGetValue(key, 0, RSSDK_REG_MAIN, RRF_RT_REG_SZ, &type, dll_path, &size);
        RegCloseKey(key);
    }
    if (dll_path[0])
    {
        PRINT_INFO((L"The SDK INFO: Loading core library from path specified in %s\n", RSSDK_REG_DISPATCH L"\\" RSSDK_REG_MAIN));
        return LoadSessionLibrary(dll_path, 0);
    } else
    {
        PRINT_INFO((L"The SDK INFO: FAILED to open registry key %s\n", RSSDK_REG_DISPATCH L"\\" RSSDK_REG_MAIN));
	    return 0;
    }
}
