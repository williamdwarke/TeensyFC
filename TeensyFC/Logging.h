#ifndef _Logging_h
#define _Logging_h
    
    #define LOG_BUF_SIZE    256
    extern char logBuf[LOG_BUF_SIZE];

    int loggingInit();
    void startLog();
    int writeLog(const char *logMsg);
    void stopLog();
    void loggingEnd();
    
#endif