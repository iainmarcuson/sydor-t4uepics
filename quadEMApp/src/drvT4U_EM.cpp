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

#define REG_T4U_CTRL            0
#define BIAS_N_EN_MASK          (1<<9)
#define BIAS_P_EN_MASK          (1<<10)

#define REG_T4U_FREQ            1

#define REG_T4U_RANGE           3
#define RANGE_SEL_MASK          0x3
#define RANGE_AUTO_MASK         (1<<7)

#define REG_PIDX_CTRL           55
#define REG_PIDY_CTRL           65
#define PID_EN_MASK             0x1

#define REG_OUTPUT_MODE         93
#define OUTPUT_MODE_MASK        0x7

#define REG_PID_CUTOUT_MODE     94
#define CUTOUT_ENABLE_MASK      0x01
#define HYST_REENABLE_MASK      0x02

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

    // Set a range index to a default
    currRange_ = 0;
    // Range scale factors
    //-=-=TODO Put in actual numbers
    ranges_[0]=5e6;
    ranges_[1]=14955.12;
    ranges_[2]=47.0;
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
    createParam(P_SampleFreq_String, asynParamInt32, &P_SampleFreq);
    createParam(P_DACMode_String, asynParamInt32, &P_DACMode);
    createParam(P_PIDEn_String, asynParamInt32, &P_PIDEn);
    createParam(P_PIDCuEn_String, asynParamInt32, &P_PIDCuEn);
    createParam(P_PIDHystEn_String, asynParamInt32, &P_PIDHystEn);
    
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
    else if (function == P_BiasP_En)
    {
        if (value)              // Turn on
        {
            epicsSnprintf(outCmdString_, sizeof(outCmdString_), "bs 0 0x400\r\n");
        }
        else                    // Turn off
        {
            epicsSnprintf(outCmdString_, sizeof(outCmdString_), "bc 0 0x400\r\n");
        }
        writeReadMeter();
    }
    else if (function == P_SampleFreq)
    {
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr %i %i\r\n",
                      REG_T4U_FREQ, value);
        writeReadMeter();
    }
    else if (function == P_Range)
    {
        // Clip the range if needed to the limits
        if (value < 0)
        {
            value = 0;
        }
        else if (value > 2)
        {
            value = 2;
        }
        // Set in the parameter library again, since it may have been changed
        // above.
        status |= setIntegerParam(channel, function, value);
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr 3 %i\r\n", value);
        writeReadMeter();
        
        currRange_ = value;
    }
    else if (function == P_DACMode)
    {
        int calc_reg = 93; // Base register
        calc_reg = (calc_reg << 16) + 1; // Multiple functions in this register, so set to function 1 for DAC Mode
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr %i %i\r\n", calc_reg, value);
        writeReadMeter();
    }
    else if (function == P_PIDEn)
    {
        char *enable_cmd[2] = {"bc", "bs"}; // Off does bc; On does bs
        
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "%s 55 1\r\n",
                      enable_cmd[value]); // Write to X
        writeReadMeter();

        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "%s 65 1\r\n",
                      enable_cmd[value]); // Write to Y
        writeReadMeter();
    }
    else if ((function == P_PIDCuEn) || (function == P_PIDHystEn))
    {
        char *enable_cmd[2] = {"bc", "bs"}; // Off does bc; on does bs.
        int reg = REG_PID_CUTOUT_MODE;
        int mask;

        if (function == P_PIDCuEn)
        {
            mask = CUTOUT_ENABLE_MASK;
        }
        else
        {
            mask = HYST_REENABLE_MASK;
        }

        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "%s %i %i\r\n",
                      enable_cmd[value], reg, mask); // Write to X
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
    T4U_Reg_T *pid_reg;
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

    if ((pid_reg = findRegByAsyn(function)) != nullptr)
    {
        int out_val = scaleParamToReg(value, pid_reg);
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr %i %i\r\n",
                      pid_reg->reg_num, out_val);
        writeReadMeter();
    }
    else if (function == P_BiasN_Voltage)
    {
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr 5 %i\r\n", (int) value);
        writeReadMeter();
    }
    else if (function == P_BiasP_Voltage)
    {
        epicsSnprintf(outCmdString_, sizeof(outCmdString_), "wr 4 %i\r\n", (int) value);
        writeReadMeter();
    }

    printf("About to exit from %s", functionName);
    fflush(stdout);
    if (function < FIRST_T4U_COMMAND)
    {
        return (asynStatus)drvQuadEM::writeFloat64(pasynUser, value);
    }
    else
    {
        return asynSuccess;
    }
            
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
    size_t nRequest = 1;
    int processRet;
    static const char *functionName = "cmdReadThread";

    status = asynSuccess;       // -=-= FIXME Used for a different call

    // Loop forever
    lock();
    while(1)
    {
        int totalBytesRead;
        bool commandReceived;
        unlock();
        epicsThreadSleep(0.001);
        totalBytesRead = 0;
        memset(InData, '\0', MAX_COMMAND_LEN);
        commandReceived = false; // No proper command recieved yet
        while (1)
        {
            status = pasynOctetSyncIO->read(pasynUserTCPCommand_, InData+totalBytesRead, nRequest, T4U_EM_TIMEOUT, &nRead, &eomReason);
            if (nRead == 1) // Read a byte successfully
            {
                totalBytesRead++;
                if (*(InData+totalBytesRead-1) == '\n') // End of a command
                {
                    commandReceived = true;
                    break;      // Break to parse the command
                }

                if (totalBytesRead >= (MAX_COMMAND_LEN -1)) // Too long
                {
                    break;
                }
            }
            else                // No byte read -- assume an error
            {
                break;
            }
        }
        lock();
        
        if (commandReceived)    //  We got a valid command
        {
            //-=-= DEBUGGING
            printf("Received command: %s\n", InData);
            fflush(stdout);
            
            processRet = processReceivedCommand(InData); // Process it
            
            if (processRet < 0) // Some variety of error
            {
                if (processRet == -1) // Unknown register
                {
                    printf("Error parsing: %s\nUnknown register.\n", InData);
                    fflush(stdout);
                }
                else
                {
                    printf("Unknown error parsing: %s\b", InData);
                    fflush(stdout);
                }      
            }
        }
        else                    // We did not get a full command one way or another
        {
            if (totalBytesRead > 0) // Read part of a command
            {
                unlock();
                status = pasynOctetSyncIO->flush(pasynUserTCPCommand_); // Flush the buffer and try again
                lock();
            }
        }    
        
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
        callParamCallbacks();
        //fflush(stdout);
    }
    return;

}

int32_t drvT4U_EM::processReceivedCommand(char *cmdString)
{
    int32_t reg_num, reg_val;
    int args_parsed;
    T4U_Reg_T *pid_reg;

    // We only accept rr commands.
    args_parsed = sscanf(cmdString, " rr %i %i\n", &reg_num, &reg_val);

    if (args_parsed != 2)       // Not a command we accept, or invalid number of arguments
    {
        return 1;              // We don't handle the command, but that is possibly okay, since it could be e.g. a valid "bs" command
    }

    pid_reg = findRegByNum(reg_num); // See if this is a PID register that needs scaling

    if (pid_reg)                // It is a PID register
    {
        double raw_percent;
        double final_val;
        raw_percent = (reg_val - pid_reg->reg_min)/(pid_reg->reg_max - pid_reg->reg_min);

        final_val = raw_percent*(pid_reg->pv_max - pid_reg->pv_min) + pid_reg->pv_min;

        setDoubleParam(pid_reg->asyn_num, final_val);
    }
    else                        // Not a PID register
    {
        if (reg_num == REG_T4U_CTRL)
        {
            if (reg_val & BIAS_N_EN_MASK)
            {
                setIntegerParam(P_BiasN_En, 1);
            }
            else
            {
                setIntegerParam(P_BiasN_En, 0);
            }

            if (reg_val & BIAS_P_EN_MASK)
            {
                setIntegerParam(P_BiasP_En, 1);
            }
            else
            {
                setIntegerParam(P_BiasP_En, 0);
            }
        }
        else if (reg_num == REG_T4U_FREQ)
        {
            setIntegerParam(P_SampleFreq, reg_val);
        }
        else if (reg_num == REG_T4U_RANGE)
        {
            //-=-= TODO We may incorporate autorange later
            int range_val = reg_val & RANGE_SEL_MASK;
            setIntegerParam(P_Range, range_val);
        }
        else if (reg_num == REG_PIDX_CTRL) // PID X control -- assumed same as PID Y
        {
            int enable_val = reg_val & PID_EN_MASK;
            setIntegerParam(P_PIDEn, enable_val);
        }
        else if (reg_num == REG_PIDY_CTRL) // Assumed same as PID X, so swallow
        {
            // Just swallow it.
        }
        else if (reg_num == REG_OUTPUT_MODE)
        {
            //-=-= TODO PID Inhibit, External Trigger Enable, and Calc Mode
            // to be added later
            int dac_mode = reg_val & OUTPUT_MODE_MASK;
            setIntegerParam(P_DACMode, dac_mode);
        }
        else if (reg_num == REG_PID_CUTOUT_MODE)
        {
            if (reg_val & CUTOUT_ENABLE_MASK)
            {
                setIntegerParam(P_PIDCuEn, 1);
            }
            else
            {
                setIntegerParam(P_PIDCuEn, 0);
            }

            if (reg_val & HYST_REENABLE_MASK)
            {
                setIntegerParam(P_PIDHystEn, 1);
            }
            else
            {
                setIntegerParam(P_PIDHystEn, 0);
            }
        }
        else                    // An unhandled command
        {
            return -1;          // Flag an error
        }
            
    }
    
    return 0;                   // Return a success
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
        readCurr_[data_idx] = rawToCurrent(read_vals[data_idx]);
    }

    return 1;                   // Read one set
}

T4U_Reg_T *drvT4U_EM::findRegByNum(const int regNum)
{
    for (auto it = pidRegData_.begin(); it != pidRegData_.end(); it++)
    {
        if (it->reg_num == regNum)
        {
            return &(*it);
        }
    }
    return nullptr;
}

T4U_Reg_T *drvT4U_EM::findRegByAsyn(const int asynParam)
{
    for (auto it = pidRegData_.begin(); it != pidRegData_.end(); it++)
    {
        if (it->asyn_num == asynParam)
        {
            return &(*it);
        }
    }
    return nullptr;
}

double drvT4U_EM::rawToCurrent(int rawVal)
{
    double calcCurrent;
    const double kVREF = 1.50;

    calcCurrent = rawVal/524288.0 * kVREF / ranges_[currRange_];
    
    return calcCurrent;
}

double drvT4U_EM::scaleParamToReg(double value, const T4U_Reg_T *reg_info, bool clip /*= false*/)
{
    double percent_orig;
    double scaled_value;

    //-=-= Not clipped here
    percent_orig = (value - reg_info->pv_min)/(reg_info->pv_max - reg_info->pv_min);
    
    scaled_value = percent_orig*(reg_info->reg_max - reg_info->reg_min)+reg_info->reg_min;

    if (!clip)                  // Not clipping -- default
    {
        return scaled_value;
    }

    if (scaled_value > reg_info->reg_max)
    {
        scaled_value = reg_info->reg_max;
    }
    else if (scaled_value < reg_info->reg_min)
    {
        scaled_value = reg_info->reg_min;
    }

    return scaled_value;
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

