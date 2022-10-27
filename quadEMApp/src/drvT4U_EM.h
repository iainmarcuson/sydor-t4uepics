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
#define T4U_EM_TIMEOUT 0.2
#define MAX_CHAN_READS 16       // The maximum number of channel reads to be sent in one message


#define P_BiasN_En_String "QE_BIAS_N"
#define P_BiasP_En_String "QE_BIAS_P"
#define P_BiasN_Voltage_String "QE_BIAS_N_VOLTAGE"
#define P_BiasP_Voltage_String "QE_BIAS_P_VOLTAGE"

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
    void cmdReadThread(void);
    void dataReadThread(void);
    virtual void exitHandler();

    /* These are functions extended from drvQuadEM */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

protected:
    int P_BiasN_En;
#define FIRST_T4U_COMMAND P_BiasN_En
    int P_BiasP_En;
    int P_BiasN_Voltage;
    int P_BiasP_Voltage;

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
    epicsEventId acquireStartEvent_;
    epicsEventId writeCmdEvent_;
    double ranges_[MAX_RANGES];
    double scaleFactor_;
    int readingActive_;
    char firmwareVersion_[MAX_COMMAND_LEN];
    char ipAddress_[MAX_IPNAME_LEN];
    char outCmdString_[MAX_COMMAND_LEN];
    char inCmdString_[MAX_COMMAND_LEN];
    double readCurr_[MAX_CHAN_READS*4]; // The values read from the socket
    
    std::forward_list<T4U_Reg_T> pidRegData_; /* Holds parameters for PID regs */

    asynStatus writeReadMeter();
    asynStatus getFirmwareVersion();
    void process_reg(const T4U_Reg_T *reg_lookup, double value);
    asynStatus readResponse();
    int32_t readTextCurrVals(asynOctet *pasynOctet, void *octetPvt);
};
