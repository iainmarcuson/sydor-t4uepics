#!/home/iainm/src/epics/synApps3/support/synApps/support/quadEM-R9-4/bin/linux-x86_64/quadEMTestApp

errlogInit(5000)
< envPaths

# Tell EPICS all about the record types, device-support modules, drivers,
# etc. in this build
dbLoadDatabase("$(QUADEM)/dbd/quadEMTestApp.dbd")
quadEMTestApp_registerRecordDeviceDriver(pdbbase)

# The search path for database files
# Note: the separator between the path entries needs to be changed to a semicolon (;) on Windows
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db:$(QUADEM)/db")

< $(QUADEM)/iocBoot/iocT4U_EM/T4U_EM.cmd
