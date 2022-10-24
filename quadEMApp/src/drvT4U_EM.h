/*
 * drvT4U_EM.h
 *                              
 * Asyn driver that inherits from the drvQuadEM class to control the Sydor
 * T4U electrometer
 * Author: Iain Marcuson
 *
 * Created October 24, 2022
 */

#include <forward_list>
#include <cstdint>

#include "drvQuadEM.h"

#define MAX_COMMAND_LEN 256
#define MAX_PORTNAME_LEN 32
#define MAX_IPNAME_LEN 16
#define MAX_RANGES 8

typedef struct {
    int32_t reg_num;            // Register address on T4U
    int asyn_num;               // The asyn param number assigned
} T4U_Reg_T;

/** Class to control the Sydor T4U Electrometer */
class drvT4U_EM : public drvQuadEM {
public:
    drvT4U_EM(const char *portName, const char *qtHostAddress, int ringBufferSize);

    /* These are the methods we implement from asynPortDriver */
    void report(FILE *fp, int details);

    /* These are the metods that are new to this class */
    void readThread(void);
    virtual void exitHandler();

    /* These are functions extended from drvQuadEM */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

protected:
    int P_BiasN;
#define FIRST_T4U_COMMAND P_BiasN
    int P_BiasP;

    /* These are the methods we implement from quadEM */
    virtual asynStatus setAcquire(epicsInt32 value);
    virtual asynStatus setPingPong(epicsInt32 value);
    virtual asynStatus setIntegrationTime(epicsFloat64 value);
    virtual asynStatus setRange(epicsInt32 value);
    virtual asynStatus setValuesPerRead(epicsInt32 value);
    virtual asynStatus readStatus();
    virtual asynStatus reset();

private:
    /* Our data */
    char *broadcastAddress_;
    char tcpCommandPortName_[MAX_PORTNAME_LEN];
    char tcpDataPortName_[MAX_PORTNAME_LEN];
    asynUser *pasynUserTCPCommand_;
    asynUser *pasynUserTCPData_;
    epicsEventID acquireStartEvent_;
    epicsEventID writeCmdEvent_;
    double ranges_[MAX_RANGES];
    double scaleFactor_;
    int readingActive_;
    char ipAddress_[MAX_IPNAME_LEN];
    char outString_[MAX_COMMAND_LEN];
    char inString_[MAX_COMMAND_LEN];
    
    std::forward_list<T4U_Reg_T> pidRegData_; /* Holds parameters for PID regs */

    asynStatus writeReadMeter();
    asynStatus getFirmwareVersion();
    void process_reg(const T4U_Reg T *reg_lookup, double value);
    asynStatus readResponse();
};
