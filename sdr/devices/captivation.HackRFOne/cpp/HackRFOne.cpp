/**************************************************************************

    This is the device code. This file contains the child class where
    custom functionality can be added to the device. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/
#include "HackRFOne.h"

PREPARE_LOGGING(HackRFOne_i)

HackRFOne_i::HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    HackRFOne_base(devMgr_ior, id, lbl, sftwrPrfl),
   _device(NULL)
{
}

HackRFOne_i::HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    HackRFOne_base(devMgr_ior, id, lbl, sftwrPrfl, compDev),
   _device(NULL)
{
}

HackRFOne_i::HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    HackRFOne_base(devMgr_ior, id, lbl, sftwrPrfl, capacities),
   _device(NULL)
{
}

HackRFOne_i::HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    HackRFOne_base(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev),
   _device(NULL)
{
}

HackRFOne_i::~HackRFOne_i()
{
}

void HackRFOne_i::constructor()
{
    /***********************************************************************************
     This is the RH constructor. All properties are properly initialized before this function is called

     For a tuner device, the structure frontend_tuner_status needs to match the number
     of tuners that this device controls and what kind of device it is.
     The options for devices are: TX, RX, RX_DIGITIZER, CHANNELIZER, DDC, RC_DIGITIZER_CHANNELIZER

     For example, if this device has 5 physical
     tuners, 3 RX_DIGITIZER and 2 CHANNELIZER, then the code in the construct function
     should look like this:

     this->addChannels(3, "RX_DIGITIZER");
     this->addChannels(2, "CHANNELIZER");

     The incoming request for tuning contains a string describing the requested tuner
     type. The string for the request must match the string in the tuner status.
    ***********************************************************************************/

   bool deviceReady = resetDriver();
    if (deviceReady) {
       this->addChannels(1, "RX_DIGITIZER");
       this->addChannels(1, "TX");
    }
}

bool HackRFOne_i::resetDriver() {

   int status = HACKRF_SUCCESS;
   if (_device != NULL) {
      status = hackrf_close(_device);
      if (status != HACKRF_SUCCESS) {
         LOG_WARN(HackRFOne_i, "Error while releasing hackrf device - Reason: " <<
                           hackrf_error_name(static_cast<hackrf_error>(status)));
      }
      _device = NULL;
   }

   // Multiple calls to 'hackrf_exit' are allowed and will have no effect
   // if the driver is already closed.
   status = hackrf_exit();
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, "Error while closing hackrf driver - Reason: " <<
                       hackrf_error_name(static_cast<hackrf_error>(status)));
   }

   status = hackrf_init();
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, "Error while initializing hackrf driver - Reason: " <<
                       hackrf_error_name(static_cast<hackrf_error>(status)));
      return false;
   }

   status = hackrf_open_by_serial(NULL, &_device);
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, "Error while opening hackrf device controller - Reason: " <<
                       hackrf_error_name(static_cast<hackrf_error>(status)));
      return false;
   }

   return true;
}

/***********************************************************************************************

    Basic functionality:

        The service function is called by the serviceThread object (of type ProcessThread).
        This call happens immediately after the previous call if the return value for
        the previous call was NORMAL.
        If the return value for the previous call was NOOP, then the serviceThread waits
        an amount of time defined in the serviceThread's constructor.

    SRI:
        To create a StreamSRI object, use the following code:
                std::string stream_id = "testStream";
                BULKIO::StreamSRI sri = bulkio::sri::create(stream_id);

        To create a StreamSRI object based on tuner status structure index 'idx' and collector center frequency of 100:
                std::string stream_id = "my_stream_id";
                BULKIO::StreamSRI sri = this->create(stream_id, this->frontend_tuner_status[idx], 100);

    Time:
        To create a PrecisionUTCTime object, use the following code:
                BULKIO::PrecisionUTCTime tstamp = bulkio::time::utils::now();


    Ports:

        Data is passed to the serviceFunction through by reading from input streams
        (BulkIO only). The input stream class is a port-specific class, so each port
        implementing the BulkIO interface will have its own type-specific input stream.
        UDP multicast (dataSDDS and dataVITA49) ports do not support streams.

        The input stream from which to read can be requested with the getCurrentStream()
        method. The optional argument to getCurrentStream() is a floating point number that
        specifies the time to wait in seconds. A zero value is non-blocking. A negative value
        is blocking.  Constants have been defined for these values, bulkio::Const::BLOCKING and
        bulkio::Const::NON_BLOCKING.

        More advanced uses of input streams are possible; refer to the REDHAWK documentation
        for more details.

        Input streams return data blocks that automatically manage the memory for the data
        and include the SRI that was in effect at the time the data was received. It is not
        necessary to delete the block; it will be cleaned up when it goes out of scope.

        To send data using a BulkIO interface, create an output stream and write the
        data to it. When done with the output stream, the close() method sends and end-of-
        stream flag and cleans up.

        NOTE: If you have a BULKIO dataSDDS or dataVITA49  port, you must manually call
              "port->updateStats()" to update the port statistics when appropriate.

        Example:
            // This example assumes that the device has two ports:
            //  An input (provides) port of type bulkio::InShortPort called dataShort_in
            //  An output (uses) port of type bulkio::OutFloatPort called dataFloat_out
            // The mapping between the port and the class is found
            // in the device base class header file

            bulkio::InShortStream inputStream = dataShort_in->getCurrentStream();
            if (!inputStream) { // No streams are available
                return NOOP;
            }

            // Get the output stream, creating it if it doesn't exist yet
            bulkio::OutFloatStream outputStream = dataFloat_out->getStream(inputStream.streamID());
            if (!outputStream) {
                outputStream = dataFloat_out->createStream(inputStream.sri());
            }

            bulkio::ShortDataBlock block = inputStream.read();
            if (!block) { // No data available
                // Propagate end-of-stream
                if (inputStream.eos()) {
                   outputStream.close();
                }
                return NOOP;
            }

            if (block.sriChanged()) {
                // Update output SRI
                outputStream.sri(block.sri());
            }

            // Get read-only access to the input data
            redhawk::shared_buffer<short> inputData = block.buffer();

            // Acquire a new buffer to hold the output data
            redhawk::buffer<float> outputData(inputData.size());

            // Transform input data into output data
            for (size_t index = 0; index < inputData.size(); ++index) {
                outputData[index] = (float) inputData[index];
            }

            // Write to the output stream; outputData must not be modified after
            // this method call
            outputStream.write(outputData, block.getStartTime());

            return NORMAL;

        If working with complex data (i.e., the "mode" on the SRI is set to
        true), the data block's complex() method will return true. Data blocks
        provide a cxbuffer() method that returns a complex interpretation of the
        buffer without making a copy:

            if (block.complex()) {
                redhawk::shared_buffer<std::complex<short> > inData = block.cxbuffer();
                redhawk::buffer<std::complex<float> > outData(inData.size());
                for (size_t index = 0; index < inData.size(); ++index) {
                    outData[index] = inData[index];
                }
                outputStream.write(outData, block.getStartTime());
            }

        Interactions with non-BULKIO ports are left up to the device developer's discretion

    Messages:

        To receive a message, you need (1) an input port of type MessageEvent, (2) a message prototype described
        as a structure property of kind message, (3) a callback to service the message, and (4) to register the callback
        with the input port.

        Assuming a property of type message is declared called "my_msg", an input port called "msg_input" is declared of
        type MessageEvent, create the following code:

        void HackRFOne_i::my_message_callback(const std::string& id, const my_msg_struct &msg){
        }

        Register the message callback onto the input port with the following form:
        this->msg_input->registerMessage("my_msg", this, &HackRFOne_i::my_message_callback);

        To send a message, you need to (1) create a message structure, (2) a message prototype described
        as a structure property of kind message, and (3) send the message over the port.

        Assuming a property of type message is declared called "my_msg", an output port called "msg_output" is declared of
        type MessageEvent, create the following code:

        ::my_msg_struct msg_out;
        this->msg_output->sendMessage(msg_out);

    Accessing the Device Manager and Domain Manager:

        Both the Device Manager hosting this Device and the Domain Manager hosting
        the Device Manager are available to the Device.

        To access the Domain Manager:
            CF::DomainManager_ptr dommgr = this->getDomainManager()->getRef();
        To access the Device Manager:
            CF::DeviceManager_ptr devmgr = this->getDeviceManager()->getRef();

    Properties:

        Properties are accessed directly as member variables. For example, if the
        property name is "baudRate", it may be accessed within member functions as
        "baudRate". Unnamed properties are given the property id as its name.
        Property types are mapped to the nearest C++ type, (e.g. "string" becomes
        "std::string"). All generated properties are declared in the base class
        (HackRFOne_base).

        Simple sequence properties are mapped to "std::vector" of the simple type.
        Struct properties, if used, are mapped to C++ structs defined in the
        generated file "struct_props.h". Field names are taken from the name in
        the properties file; if no name is given, a generated name of the form
        "field_n" is used, where "n" is the ordinal number of the field.

        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A boolean called scaleInput

            if (scaleInput) {
                dataOut[i] = dataIn[i] * scaleValue;
            } else {
                dataOut[i] = dataIn[i];
            }

        Callback methods can be associated with a property so that the methods are
        called each time the property value changes.  This is done by calling
        addPropertyListener(<property>, this, &HackRFOne_i::<callback method>)
        in the constructor.

        The callback method receives two arguments, the old and new values, and
        should return nothing (void). The arguments can be passed by value,
        receiving a copy (preferred for primitive types), or by const reference
        (preferred for strings, structs and vectors).

        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A struct property called status

        //Add to HackRFOne.cpp
        HackRFOne_i::HackRFOne_i(const char *uuid, const char *label) :
            HackRFOne_base(uuid, label)
        {
            addPropertyListener(scaleValue, this, &HackRFOne_i::scaleChanged);
            addPropertyListener(status, this, &HackRFOne_i::statusChanged);
        }

        void HackRFOne_i::scaleChanged(float oldValue, float newValue)
        {
            LOG_DEBUG(HackRFOne_i, "scaleValue changed from" << oldValue << " to " << newValue);
        }

        void HackRFOne_i::statusChanged(const status_struct& oldValue, const status_struct& newValue)
        {
            LOG_DEBUG(HackRFOne_i, "status changed");
        }

        //Add to HackRFOne.h
        void scaleChanged(float oldValue, float newValue);
        void statusChanged(const status_struct& oldValue, const status_struct& newValue);

    Allocation:

        Allocation callbacks are available to customize the Device's response to
        allocation requests. For example, if the Device contains the allocation
        property "my_alloc" of type string, the allocation and deallocation
        callbacks follow the pattern (with arbitrary function names
        my_alloc_fn and my_dealloc_fn):

        bool HackRFOne_i::my_alloc_fn(const std::string &value)
        {
            // perform logic
            return true; // successful allocation
        }
        void HackRFOne_i::my_dealloc_fn(const std::string &value)
        {
            // perform logic
        }

        The allocation and deallocation functions are then registered with the Device
        base class with the setAllocationImpl call. Note that the variable for the property is used rather
        than its id:

        this->setAllocationImpl(my_alloc, this, &HackRFOne_i::my_alloc_fn, &HackRFOne_i::my_dealloc_fn);



************************************************************************************************/
int HackRFOne_i::serviceFunction()
{
    LOG_DEBUG(HackRFOne_i, "serviceFunction() example log message");

    return NOOP;
}

/*************************************************************
Functions supporting tuning allocation
*************************************************************/
void HackRFOne_i::deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to set the 'enabled' member of fts to indicate that tuner as enabled
    ************************************************************/
    #warning deviceEnable(): Enable the given tuner  *********

    fts.enabled = true;
    return;
}
void HackRFOne_i::deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to reset the 'enabled' member of fts to indicate that tuner as disabled
    ************************************************************/
    #warning deviceDisable(): Disable the given tuner  *********
    fts.enabled = false;
    return;
}
bool HackRFOne_i::deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id){
/************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
      At a minimum, bandwidth, center frequency, and sample_rate have to be set
      If the device is tuned to exactly what the request was, the code should be:
        fts.bandwidth = request.bandwidth;
        fts.center_frequency = request.center_frequency;
        fts.sample_rate = request.sample_rate;

    return true if the tuning succeeded, and false if it failed
    ************************************************************/
    #warning deviceSetTuning(): Evaluate whether or not a tuner is added  *********
    return true;
}
bool HackRFOne_i::deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id) {
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    return true if the tune deletion succeeded, and false if it failed
    ************************************************************/
    #warning deviceDeleteTuning(): Deallocate an allocated tuner  *********
    return true;
}

/*************************************************************
Functions servicing the tuner control port
*************************************************************/
std::string HackRFOne_i::getTunerType(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].tuner_type;
}

bool HackRFOne_i::getTunerDeviceControl(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if (getControlAllocationId(idx) == allocation_id)
        return true;
    return false;
}

std::string HackRFOne_i::getTunerGroupId(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].group_id;
}

std::string HackRFOne_i::getTunerRfFlowId(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].rf_flow_id;
}

void HackRFOne_i::setTunerCenterFrequency(const std::string& allocation_id, double freq) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (freq<0) throw FRONTEND::BadParameterException("Center frequency cannot be less than 0");
    // set hardware to new value. Raise an exception if it's not possible
    this->frontend_tuner_status[idx].center_frequency = freq;
}

double HackRFOne_i::getTunerCenterFrequency(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].center_frequency;
}

void HackRFOne_i::setTunerBandwidth(const std::string& allocation_id, double bw) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (bw<0) throw FRONTEND::BadParameterException("Bandwidth cannot be less than 0");
    // set hardware to new value. Raise an exception if it's not possible
    this->frontend_tuner_status[idx].bandwidth = bw;
}

double HackRFOne_i::getTunerBandwidth(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].bandwidth;
}

void HackRFOne_i::setTunerAgcEnable(const std::string& allocation_id, bool enable)
{
    throw FRONTEND::NotSupportedException("setTunerAgcEnable not supported");
}

bool HackRFOne_i::getTunerAgcEnable(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerAgcEnable not supported");
}

void HackRFOne_i::setTunerGain(const std::string& allocation_id, float gain)
{
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (gain < 0) throw FRONTEND::BadParameterException("Gain cannot be less than 0 dB");

    int status = HACKRF_SUCCESS;
    unsigned int gainInt = static_cast<unsigned int>(gain);
    if (frontend_tuner_status[idx].tuner_type == "TX") {
       if (gainInt > 47) {
          throw FRONTEND::BadParameterException("TX-gain cannot be greater than 47 dB");
       }
       status = hackrf_set_txvga_gain(_device, gainInt);
       if (status != HACKRF_SUCCESS) {
           std::string errorMsg = "Failed to set TX-gain on device - Reason: " +
                                  std::string(hackrf_error_name(static_cast<hackrf_error>(status)));
           throw FRONTEND::FrontendException(errorMsg.c_str());
       }
    } else {
       // TODO: Add in additional LNA gain
       //        * VGA_GAIN range is 0-62 (2dB steps)
       //        * LNA_GAIN range is 0-40 (8dB steps)
       if (gainInt > 62) {
          throw FRONTEND::BadParameterException("RX-Gain cannot be greater than 62 dB");
       }
       if (gainInt % 2) {
          gainInt = gainInt - (gainInt % 2);
          LOG_WARN(HackRFOne_i, "RX-Gain must be in 2-dB steps. Rounding to nearest gain of " <<
                              gainInt << " dB");
       }
       status = hackrf_set_vga_gain(_device, gainInt);
       if (status != HACKRF_SUCCESS) {
           std::string errorMsg = "Failed to set RX-gain on device - Reason: " +
                                  std::string(hackrf_error_name(static_cast<hackrf_error>(status)));
           throw FRONTEND::FrontendException(errorMsg.c_str());
       }
    }

    // set hardware to new value. Raise an exception if it's not possible
    this->frontend_tuner_status[idx].gain = static_cast<float>(gainInt);
}

float HackRFOne_i::getTunerGain(const std::string& allocation_id)
{
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].gain;
}

void HackRFOne_i::setTunerReferenceSource(const std::string& allocation_id, long source)
{
    throw FRONTEND::NotSupportedException("setTunerReferenceSource not supported");
}

long HackRFOne_i::getTunerReferenceSource(const std::string& allocation_id)
{
    throw FRONTEND::NotSupportedException("getTunerReferenceSource not supported");
}

void HackRFOne_i::setTunerEnable(const std::string& allocation_id, bool enable) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    // set hardware to new value. Raise an exception if it's not possible
    this->frontend_tuner_status[idx].enabled = enable;
}

bool HackRFOne_i::getTunerEnable(const std::string& allocation_id) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].enabled;
}

void HackRFOne_i::setTunerOutputSampleRate(const std::string& allocation_id, double sr) {
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    if(allocation_id != getControlAllocationId(idx))
        throw FRONTEND::FrontendException(("ID "+allocation_id+" does not have authorization to modify the tuner").c_str());
    if (sr<0) throw FRONTEND::BadParameterException("Sample rate cannot be less than 0");
    // set hardware to new value. Raise an exception if it's not possible
    this->frontend_tuner_status[idx].sample_rate = sr;
}

double HackRFOne_i::getTunerOutputSampleRate(const std::string& allocation_id){
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].sample_rate;
}

/*************************************************************
Functions servicing the RFInfo port(s)
- port_name is the port over which the call was received
*************************************************************/
std::string HackRFOne_i::get_rf_flow_id(const std::string& port_name)
{
    return std::string("none");
}

void HackRFOne_i::set_rf_flow_id(const std::string& port_name, const std::string& id)
{
}

frontend::RFInfoPkt HackRFOne_i::get_rfinfo_pkt(const std::string& port_name)
{
    frontend::RFInfoPkt pkt;
    return pkt;
}

void HackRFOne_i::set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt &pkt)
{
}

