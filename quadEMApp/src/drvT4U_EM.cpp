/*
 * drvT4U_EM.cpp
 * 
 * Asyn driver that inherits from the drvQuadEM class to control 
 * the Sydor T4U Electrometer
 *
 * Author: Iain Marcuson
 *
 * Created October 24, 2022
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <cstdint>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsEvent.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <drvAsynIPPort.h>
#include <iocsh.h>

#include <epicsExport.h>
#include "drvT4U_EM.h"

#define BROADCAST_TIMEOUT 0.2
#define NSLS_EM_TIMEOUT   0.1

#define COMMAND_PORT    13001
#define DATA_PORT       13002
#define MIN_INTEGRATION_TIME 400e-6
#define MAX_INTEGRATION_TIME 1.0
#define FREQUENCY 1e6
// 2^20 is maximum counts for 20-bit ADC
#define MAX_COUNTS 1048576.0

typedef enum {
  Phase0, 
  Phase1, 
  PhaseBoth
} PingPongValue_t;

static const char *driverName="drvT4U_EM";
static void readThread(void *drvPvt);
static void dataReadThread(void *drvPvt);


/** Constructor for the drvT4U_EM class.
  * Calls the constructor for the drvQuadEM base class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] qtHostAddress The address of the Qt middle layer
  * \param[in] ringBufferSize The number of samples to hold in the input ring buffer.
  *            This should be large enough to hold all the samples between reads of the
  *            device, e.g. 1 ms SampleTime and 1 second read rate = 1000 samples.
  *            If 0 then default of 2048 is used.
  */
drvT4U_EM::drvT4U_EM(const char *portName, const char *qtHostAddress, int ringBufferSize) 
   : drvQuadEM(portName, ringBufferSize)
  
{
    asynStatus status;
    const char *functionName = "drvT4U_EM";
    char tempString[256];
    T4U_Reg_T curr_reg;

    
    ipAddress_[0] = 0;
    firmwareVersion_[0] = 0;
    
    // Range scale factors
    //-=-=TODO Put in actual numbers
    ranges_[0]=1.0;
    ranges_[1]=1.0;
    ranges_[2]=1.0;
    ranges_[3]=1.0;
    ranges_[4]=1.0;
    ranges_[5]=1.0;
    ranges_[6]=1.0;
    ranges_[7]=1.0;
    
    acquireStartEvent_ = epicsEventCreate(epicsEventEmpty);

    // Create the parameters
    createParam(P_BiasN_En_String, asynParamInt32, &P_BiasN_En);
    createParam(P_BiasP_En_String, asynParamInt32, &P_BiasP_En);
    createParam(P_BiasN_Voltage_String, asynParamFloat64, &P_BiasN_Voltage);
    createParam(P_BiasP_Voltage_String, asynParamFloat64, &P_BiasP_Voltage);
#include "gc_t4u_cpp_params.cpp"
    
    // Create the port names
    strcpy(tcpCommandPortName_, "TCP_Command_");
    strcat(tcpCommandPortName_, portName); // -=-= TODO Add length check?
    strcpy(tcpDataPortName_, "TCP_Data_");
    strcat(tcpDataPortName_, portName);

    // Connect the ports

    // First the command port
    epicsSnprintf(tempString, sizeof(tempString), "%s:%d", qtHostAddress, COMMAND_PORT);
    status = (asynStatus)drvAsynIPPortConfigure(tcpCommandPortName_, tempString, 0, 0, 0);
    printf("Attempted command port: %s connection to: %s\nStatus: %d\n", tcpCommandPortName_, tempString, status);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error calling drvAsynIPPortConfigure for command port=%s, IP=%s, status=%d\n", 
            driverName, functionName, tcpCommandPortName_, tempString, status);
        return;
    }

    // Connect to the command port
    status = pasynOctetSyncIO->connect(tcpCommandPortName_, 0, &pasynUserTCPCommand_, NULL);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error connecting to TCP Command port, status=%d, error=%s\n", 
            driverName, functionName, status, pasynUserTCPCommand_->errorMessage);
        return;
    }

    // Now the data port
    epicsSnprintf(tempString, sizeof(tempString), "%s:%d", qtHostAddress, DATA_PORT);
    status = (asynStatus)drvAsynIPPortConfigure(tcpDataPortName_, tempString, 0, 0, 0);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error calling drvAsynIPPortConfigure for data port=%s, IP=%s, status=%d\n", 
            driverName, functionName, tcpDataPortName_, tempString, status);
        return;
    }

    // Connect to the command port
    status = pasynOctetSyncIO->connect(tcpDataPortName_, 0, &pasynUserTCPData_, NULL);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error connecting to TCP Data port, status=%d, error=%s\n", 
            driverName, functionName, status, pasynUserTCPData_->errorMessage);
        return;
    }
    
    
    acquiring_ = 0;
    readingActive_ = 0;
    setIntegerParam(P_Model, QE_ModelNSLS_EM);
    setIntegerParam(P_ValuesPerRead, 5);
    acquiring_ = 1;
    drvQuadEM::setAcquire(1);

    // Do everything that needs to be done when connecting to the meter initially.
    // Note that the meter could be offline when the IOC starts, so we put this in
    // the reset() function which can be done later when the meter is online.
//    lock();
//    drvQuadEM::reset();
//    unlock();

    /* Create the thread that reads commands from the meter */
    status = (asynStatus)(epicsThreadCreate("drvT4U_EM_Cmd_Task",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::readThread,
                          this) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure for Command, status=%d\n", driverName, functionName, status);
        return;
    }

    /* Create the thread that reads the meter */
    status = (asynStatus)(epicsThreadCreate("drvT4U_EM_Data_Task",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::dataReadThread,
                          this) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate Data failure, status=%d\n", driverName, functionName, status);
        return;
    }
    
    callParamCallbacks();
}

void drvT4U_EM::report(FILE *fp, int details)
{
    return;
}

void drvT4U_EM::exitHandler()
{
    return;
}

asynStatus drvT4U_EM::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    int channel;
    int reg_lookup;
    const char *paramName;
    const char *functionName = "writeInt32";

    getAddress(pasynUser, &channel);

    // Debugging
    printf("%s: function %i\n", functionName, function);
    fflush(stdout);
    
    /* Set the parameter in the parameter library. */
    status |= setIntegerParam(channel, function, value);
    
    // Fetch the parameter string name
    getParamName(function, &paramName);

    // Debugging
    printf("%s: function %i name %s\n", functionName, function, paramName);
    fflush(stdout);
    
    if (function == P_BiasN_En)
    {
        if (value)              // Turn on
        {
            epicsSnprintf(outCmdString_, sizeof(outCmdString_), "bs 0 0x200\r\n");
        }
        else                    // Turn off
        {
            epicsSnprintf(outCmdString_, sizeof(outCmdString_), "bc 0 0x200\r\n");
        }
        writeReadMeter();
    }
    else if (function = P_Range)
    {
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr 3 %i\r\n", value);
        writeReadMeter();
    }

    printf("About to return from %s\n", functionName);
    fflush(stdout);
    return (asynStatus)drvQuadEM::writeInt32(pasynUser, value);
}

asynStatus drvT4U_EM::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    int status = asynSuccess;
    int channel;
    int reg_lookup;
    const char *paramName;
    const char *functionName = "writeFloat64";
    
    getAddress(pasynUser, &channel);
    
    /* Set the parameter in the parameter library. */
    status |= setDoubleParam(channel, function, value);
    
    // Fetch the parameter string name
    getParamName(function, &paramName);

    // Debugging
    printf("%s: function %i name %s\n", functionName, function, paramName);
    fflush(stdout);
    
    if (function == P_BiasN_Voltage)
    {
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr 5 %i\r\n", (int) value);
        writeReadMeter();
    }

    printf("About to exit from %s", functionName);
    fflush(stdout);
    return (asynStatus)drvQuadEM::writeFloat64(pasynUser, value);
}

asynStatus drvT4U_EM::setAcquire(epicsInt32 value)
{
    return asynSuccess;
}

asynStatus drvT4U_EM::setPingPong(epicsInt32 value)
{
    return asynSuccess;
}

asynStatus drvT4U_EM::setIntegrationTime(epicsFloat64 value)
{
    return asynSuccess;
}

asynStatus drvT4U_EM::setRange(epicsInt32 value)
{
    return asynSuccess;
}

asynStatus drvT4U_EM::setValuesPerRead(epicsInt32 value)
{
    return asynSuccess;
}

asynStatus drvT4U_EM::readStatus()
{
    return asynSuccess;
}

asynStatus drvT4U_EM::reset()
{
    return asynSuccess;
}

// Just does a write; reading is handling in its own thread
asynStatus drvT4U_EM::writeReadMeter()
{
    size_t nwrite;
    int eomReason;
    asynStatus status=asynSuccess;
    asynOctet *pasynOctet;
    asynInterface *pasynInterface;
    void *octetPvt;

    // Debugging
    printf("Entered writeReadMeter()\n");
    fflush(stdout);

    if (strlen(outCmdString_) != 0) // Actual command
    {
        status = pasynOctetSyncIO->write(pasynUserTCPCommand_, outCmdString_, strlen(outCmdString_), T4U_EM_TIMEOUT, &nwrite);
        printf("Write status %i\n", (int) status);
        fflush(stdout);
    }
        
    return status;
}

asynStatus drvT4U_EM::getFirmwareVersion()
{
    return asynSuccess;
}

void drvT4U_EM::process_reg(const T4U_Reg_T *reg_lookup, double value)
{
    return;
}

void drvT4U_EM::cmdReadThread(void)
{
    asynStatus status;
    size_t nRead;
    int eomReason;
    int i;
    char InData[MAX_COMMAND_LEN];
    size_t nRequest = MAX_COMMAND_LEN;
    static const char *functionName = "cmdReadThread";

    status = asynSuccess;       // -=-= FIXME Used for a different call

    // Loop forever
    lock();
    while(1)
    {
        unlock();
        epicsThreadSleep(0.001);
        memset(InData, '\0', MAX_COMMAND_LEN);
        status = pasynOctetSyncIO->read(pasynUserTCPCommand_, InData, nRequest, T4U_EM_TIMEOUT, &nRead, &eomReason);
        lock();
        printf("Received command: %s\n", InData);
        fflush(stdout);
    }
    return;
}

void drvT4U_EM::dataReadThread(void)
{
    asynStatus status;
    size_t nRead;
    int eomReason;
    int i;
    char InData[MAX_COMMAND_LEN];
    size_t nRequest = 1;        // Read only one byte here to pass to the parser
    int32_t data_read;          // How many full readings we did
    static const char *functionName = "dataReadThread";

    status = asynSuccess;       // -=-= FIXME Used for a different call
    
    // Loop forever
    lock();
    while(1)
    {
        unlock();
        epicsThreadSleep(0.001);
        memset(InData, '\0', MAX_COMMAND_LEN);
        status = pasynOctetSyncIO->read(pasynUserTCPData_, InData, nRequest, T4U_EM_TIMEOUT, &nRead, &eomReason);

        data_read = -1;         // Set to flush if invalid header
        if (nRead == 1)
        {
            // -=-= DEBUGGING
            //printf("Data Read: %c\n", InData[0]);
            //fflush(stdout);
            // Having received read data, read type and pass to handler
            if (InData[0] == 'r')   // "r"ead
            {
                data_read = readTextCurrVals();
            }
            else                // Bad header
            {
                data_read = -1; // Flag error
            }
            
            if (data_read < 0)  // Error reading data
            {
                pasynOctetSyncIO->flush(pasynUserTCPData_);
            }
        }

        lock();
        // -=-= DEBUGGING
        //printf("Received %i counts\n", data_read);
        if (data_read > 0)
        {
            for (int data_idx = 0; data_idx < (data_read*4); )
            {
                //printf("%f, %f, %f, %f\n", readCurr_[data_idx++],
                //       readCurr_[data_idx++], readCurr_[data_idx++], readCurr_[data_idx++]);
            data_idx += 4;
                computePositions(&readCurr_[data_idx-4]);
            }
        }
        //fflush(stdout);
    }
    return;

}


asynStatus drvT4U_EM::readResponse()
{
    return asynSuccess;
}

// Reads the current values from a text stream.  Returns number of full reads, or negative number on failure
int32_t drvT4U_EM::readTextCurrVals()
{
    char InData[MAX_COMMAND_LEN + 1];
    int data_matched;
    asynStatus status;
    int eomReason;
    size_t nRead;                  // How many bytes read in one command
    uint32_t bytes_read = 0;    // Cumulative bytes read
    size_t nRequest = 1;           // Read one byte at a time
    int read_vals[4];           // Hold the four read values
    

    // Null out the input array
    memset(InData, '\0', sizeof(InData));

    // Prepare to do the reads
    // We have the lock from the calling function
    
    while (1)
    {
        status = pasynOctetSyncIO->read(pasynUserTCPData_, InData+bytes_read, nRequest, 0.01, &nRead, &eomReason);
        if (nRead == 0)         // Problem reading
        {
            return -1;          // Error, so pass it up
        }
        
        bytes_read++;

        if (InData[bytes_read-1] == '\n') // End of a read command
        {
            break;
        }

        if (bytes_read >= MAX_COMMAND_LEN) // Too large
        {
            return -1;          // Return error
        }
    }

    // Now parse the values
    data_matched = sscanf(InData, "ead %i , %i , %i , %i ",
                          &read_vals[0], &read_vals[1], &read_vals[2], &read_vals[3]);
    if (data_matched != 4)        // Bad format
    {
        return -1;              // Return error
    }

    // Convert to expected double
    for (uint data_idx = 0; data_idx < 4; data_idx++)
    {
        readCurr_[data_idx] = read_vals[data_idx];
    }

    return 1;                   // Read one set
}


void readThread(void *drvPvt)
{
    drvT4U_EM *pPvt = (drvT4U_EM *)drvPvt;

    pPvt->cmdReadThread();
}

void dataReadThread(void *drvPvt)
{
    drvT4U_EM *pPvt = (drvT4U_EM *)drvPvt;

    pPvt->dataReadThread();
}


extern "C" {

// EPICS iocsh callable function to call constructor for the drvT4U_EM class.
//-=-= TODO doxygen
int drvT4U_EMConfigure(const char *portName, const char *qtHostAddress, int ringBufferSize)
{
    new drvT4U_EM(portName, qtHostAddress, ringBufferSize);
    return (asynSuccess);
}

// EPICS iocsh shell commands

static const iocshArg initArg0 = { "portName", iocshArgString};
static const iocshArg initArg1 = { "qt host address", iocshArgString};
static const iocshArg initArg2 = { "ring buffer size",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2};

static const iocshFuncDef initFuncDef = {"drvT4U_EMConfigure",3,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    drvT4U_EMConfigure(args[0].sval, args[1].sval, args[2].ival);
}

void drvT4U_EMRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(drvT4U_EMRegister);

}

