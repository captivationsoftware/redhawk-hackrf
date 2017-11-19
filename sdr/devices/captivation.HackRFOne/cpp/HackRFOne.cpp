/**************************************************************************

    This is the device code. This file contains the child class where
    custom functionality can be added to the device. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/
#include "HackRFConstants.h"
#include "HackRFOne.h"

using namespace HackRFConstants;

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
	resetDriver();
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

	this->setThreadDelay(0.00001);

	// Pre-allocate buffers
    _buffers.resize(32);
    for (size_t i = 0; i < _buffers.size(); i++) {
    	_buffers[i].reserve(262144);
    }

    // Attempt to locate/instantiate device driver
	reset();

    // Automatically start device
    start();
}

void HackRFOne_i::addChannels(size_t num, const std::string& tunerType)
{
    frontend_tuner_status.resize(frontend_tuner_status.size()+num);
    tuner_allocation_ids.resize(tuner_allocation_ids.size()+num);
    for (std::vector<frontend_tuner_status_struct_struct>::reverse_iterator iter=frontend_tuner_status.rbegin();
    																		iter!=frontend_tuner_status.rbegin()+num;
    																		iter++) {
        iter->enabled = false;
        iter->tuner_type = tunerType;
    }
}

bool HackRFOne_i::reset()
{
	// Move all pre-allocated buffers into 'available' list
	_filledBuffers.clear();
	_availableBuffers.clear();
    for (size_t i = 0; i < _buffers.size(); i++) {
    	_availableBuffers.push_back(&_buffers[i]);
    }

	// Reset the available tuners
	for (size_t i = 0; i < frontend_tuner_status.size(); i++) {
		this->removeTunerMapping(i);
	}
	this->setNumChannels(0);

	// Attempt to release (if loaded) and re-acquire HackRF device
    bool deviceReady = resetDriver();
    if (deviceReady) {
    	this->addChannels(1, "RX_DIGITIZER");
    	this->addChannels(1, "TX");
    }
    return deviceReady;
}

bool HackRFOne_i::resetDriver() {
   int status = HACKRF_SUCCESS;
   if (_device != NULL) {
      status = hackrf_close(_device);
      if (status != HACKRF_SUCCESS) {
         LOG_WARN(HackRFOne_i, errorString("Error while releasing hackrf device", status));
      }
      _device = NULL;
   }

   // Multiple calls to 'hackrf_exit' are allowed and will have no effect
   // if the driver is already closed.
   status = hackrf_exit();
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, errorString("Error while closing hackrf driver", status));
   }

   status = hackrf_init();
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, errorString("Error while initializing hackrf driver", status));
      return false;
   }

   status = hackrf_open_by_serial(NULL, &_device);
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, errorString("Error while opening hackrf device controller", status));
      return false;
   }

   read_partid_serialno_t serial;
   status = hackrf_board_partid_serialno_read(_device, &serial);
   if (status != HACKRF_SUCCESS) {
      LOG_WARN(HackRFOne_i, errorString("Error while determing device serial number", status));
      return false;
   }

   LOG_INFO(HackRFOne_i, "Successfully opened HackRF device: "
		                  << serial.part_id[0] << "-"
		                  << serial.part_id[1] << ":"
						  << serial.serial_no[0] << "-"
						  << serial.serial_no[1] << "-"
						  << serial.serial_no[2] << "-"
						  << serial.serial_no[3]);
   return true;
}

void HackRFOne_i::resetStream(frontend_tuner_status_struct_struct& fts)
{
	// Initialize SRI
	std::string streamId = ossie::generateUUID();
	_currentSRI = create(streamId, fts);
	_currentSRI.mode = 1; // Always complex output

	// Initialize starting time-stamp
	_timestamp = bulkio::time::utils::now();

    // Close any existing streams
	if (!(!_currentStream)) {
		_currentStream.close();
		_currentStream = bulkio::OutOctetStream();
	}

	// Initialize the new stream
	_currentStream = dataOctet_out->createStream(_currentSRI);
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
	// Always service data immediately
	if (_device && hackrf_is_streaming(_device) == HACKRF_TRUE) {
		return pushData();
	}

	// Not servicing data - Check if device is ready...
	if (!_device) {
		reset();
	}

	// Device driver is ready - Validate it's still accessible
    if (_device) {
		std::string versionStr(' ', 100);
		int status = hackrf_version_string_read(_device, &versionStr[0], versionStr.size());
		if (status != HACKRF_SUCCESS) {
			reset();
		}
    }

    return NOOP;
}

int HackRFOne_i::pushData()
{
	std::pair< std::vector<unsigned char>*, BULKIO::PrecisionUTCTime> buffer;

	// Find a buffer thats ready
	{
		boost::mutex::scoped_lock lock(_filledLock);
		if (_filledBuffers.empty()) {
			return NOOP;
		}

		buffer = _filledBuffers.front();
		_filledBuffers.pop_front();
	}

    // Ensure we have an active stream to push
	if (!(!_currentStream)) {
		_currentStream.write((*buffer.first), buffer.second);
	}

	// Return buffer to available queue
	{
		boost::mutex::scoped_lock lock(_availableLock);
		_availableBuffers.push_back(buffer.first);
	}

	return NORMAL;
}

int hackrf_rx(hackrf_transfer* transfer)
{
	return ((HackRFOne_i*)transfer->rx_ctx)->hackrfRXCallback(transfer);
}

int HackRFOne_i::hackrfRXCallback(hackrf_transfer* transfer)
{
	// Maintain and propagate time-stamp forward
	//   Propagating BEFORE handling to retain proper timing during data-drops
	BULKIO::PrecisionUTCTime currentTS = _timestamp;
	_timestamp = bulkio::time::utils::addSampleOffset(currentTS,
			                                          transfer->valid_length / 2,
													  _currentSRI.xdelta);
	std::vector<unsigned char>* buffer;
	{
		boost::mutex::scoped_lock lock(_availableLock);
		if (_availableBuffers.empty()) {
			LOG_WARN(HackRFOne_i, "All buffers are currently in use... Dropping data!");
			return 0;
		}
		buffer = _availableBuffers.front();
		_availableBuffers.pop_front();
	}

	buffer->clear();
	buffer->insert(buffer->begin(), &transfer->buffer[0], &transfer->buffer[transfer->valid_length]);

	LOG_DEBUG(HackRFOne_i, "RECVD HACKRF TRANSFER |"
			               << " ValidLength=" << transfer->valid_length
						   << " BufferLength=" << transfer->buffer_length
						   << " OutputLength=" << buffer->size());

	{
		boost::mutex::scoped_lock lock(_filledLock);
		_filledBuffers.push_back(std::make_pair(buffer,currentTS));
	}
	return 0;
}

bool HackRFOne_i::updateHackRFSampleRate(frontend_tuner_status_struct_struct &fts,
		                                 double sampleRate, double tolerance)
{
	// Check validity of request and round to nearest valid value
	fts.sample_rate = sampleRate;
	if (!withinBounds(fts.sample_rate, tolerance, One::ALLOWED_SAMPLE_RATES)) {
		LOG_WARN(HackRFOne_i, "Requested sample rate '" << fts.sample_rate << "' is not supported!");
		return false;
	}

	// Apply sample rate to hackrf device
	int result = hackrf_set_sample_rate(_device, fts.sample_rate);
	if (result != HACKRF_SUCCESS) {
		LOG_WARN(HackRFOne_i, errorString("Failed setting filter bandwidth", result));
		return false;
	}

	return true;
}

bool HackRFOne_i::updateHackRFBandwidth(frontend_tuner_status_struct_struct &fts,
		                                double bandwidth, double tolerance)
{
	// Check validity of request and round to nearest valid value
	fts.bandwidth = bandwidth;
	if (!withinBounds(fts.bandwidth, tolerance, One::ALLOWED_BW_FILTERS)) {
		LOG_WARN(HackRFOne_i, "Requested bandwidth '" << fts.bandwidth << "' is not supported!");
		return false;
	}

	// Apply bandwidth to hackrf device
	int result = hackrf_set_baseband_filter_bandwidth(_device, fts.bandwidth);
	if (result != HACKRF_SUCCESS) {
		LOG_WARN(HackRFOne_i, errorString("Failed setting filter bandwidth", result));
		return false;
	}

	return true;
}

bool HackRFOne_i::updateHackRFFrequency(frontend_tuner_status_struct_struct &fts,
		                                double frequency, double tolerance)
{
	// Check validity of request and round to nearest valid value
	fts.center_frequency = frequency;
	if (!withinBounds(fts.center_frequency, One::MIN_RF_FREQ, One::MAX_RF_FREQ)) {
		LOG_WARN(HackRFOne_i, "Requested center frequency '" << fts.center_frequency << "' is not supported!");
		return false;
	}

	// Apply frequency to hackrf device
	int result = hackrf_set_freq(_device, fts.center_frequency);
	if (result != HACKRF_SUCCESS) {
		LOG_WARN(HackRFOne_i, errorString("Failed setting frequency", result));
		return false;
	}

	return true;
}

bool HackRFOne_i::updateHackRFGainRX(frontend_tuner_status_struct_struct &fts,
		                             double gain, double tolerance)
{
	// Two gain-stages on the HackRF - Attempting to match gain on both stages
	int gainPerStage = static_cast<int>(gain / 2.0);

	// LNA gain has widest gain-steps so determine
	int lnaGainFloor = gainPerStage - (gainPerStage % static_cast<int>(One::LNA_STEP_SIZE));
	double lnaGain = findNearest(lnaGainFloor, One::ALLOWED_LNA_GAIN_VALUES);

	double vgaGain = (gain - lnaGain);
	if (!withinBounds(vgaGain, tolerance, One::ALLOWED_RX_VGA_VALUES)) {
		LOG_WARN(HackRFOne_i, "Requested gain '" << gain << "' is not supported -"
				              << " (LNA_GAIN_MAX|VGA_GAIN_MAX)="
							  << One::ALLOWED_LNA_GAIN_VALUES.back() << "|"
							  << One::ALLOWED_RX_VGA_VALUES.back() << ")");
		return false;
	}

    int status = hackrf_set_lna_gain(_device, lnaGain);
    if (status != HACKRF_SUCCESS) {
        LOG_WARN(HackRFOne_i, errorString("Failed to set LNA-gain", status).c_str());
        return false;
    }

    status = hackrf_set_vga_gain(_device, vgaGain);
    if (status != HACKRF_SUCCESS) {
    	LOG_WARN(HackRFOne_i, errorString("Failed to set RX-VGA-gain", status).c_str());
    	return false;
    }

    fts.gain = lnaGain + vgaGain;
    return true;
}

bool HackRFOne_i::updateHackRFGainTX(frontend_tuner_status_struct_struct &fts,
		                             double gain, double tolerance)
{
	// TODO: Populate TX gain
	return false;
}

/*************************************************************
Functions supporting tuning allocation
*************************************************************/
void HackRFOne_i::deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to set the 'enabled' member of fts to indicate that tuner as enabled
    ************************************************************/

	// Reset the stream for this tuner
	resetStream(fts);

	// Setup 'reasonable' gain as default
	updateHackRFGainRX(fts, One::DEFAULT_LNA_GAIN+One::DEFAULT_VGA_GAIN, 100.0);

	// Enable RX on the hardware
	if (fts.tuner_type == "RX_DIGITIZER") {
		hackrf_start_rx(_device, hackrf_rx, this);
	}

	// TODO: Add TX

    fts.enabled = true;
    return;
}

void HackRFOne_i::deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id){
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    Make sure to reset the 'enabled' member of fts to indicate that tuner as disabled
    ************************************************************/
	if (fts.tuner_type == "RX_DIGITIZER") hackrf_stop_rx(_device);
	if (fts.tuner_type == "TX") hackrf_stop_tx(_device);

	// Close the current stream
	if (!(!_currentStream)) {
		_currentStream.close();
		_currentStream = bulkio::OutOctetStream();
	}

    fts.enabled = false;
    return;
}

bool HackRFOne_i::deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id)
{
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
      At a minimum, bandwidth, center frequency, and sample_rate have to be set
      If the device is tuned to exactly what the request was, the code should be:
        fts.bandwidth = request.bandwidth;
        fts.center_frequency = request.center_frequency;
        fts.sample_rate = request.sample_rate;

    return true if the tuning succeeded, and false if it failed
    ************************************************************/
	bool success = true;
	success &= updateHackRFBandwidth(fts, request.bandwidth, request.bandwidth_tolerance);
	success &= updateHackRFSampleRate(fts, request.sample_rate, request.sample_rate_tolerance);
	success &= updateHackRFFrequency(fts, request.center_frequency, 0.0);

	if (success) {
		LOG_INFO(HackRFOne_i, "Successfully configured HackRF with (SR|BW|FREQ)=("
						      << uint64_t(fts.sample_rate) << "|"
						      << uint64_t(fts.bandwidth) << "|"
						      << uint64_t(fts.center_frequency) << ")");
	}
    return success;
}

bool HackRFOne_i::deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id) {
    /************************************************************
    modify fts, which corresponds to this->frontend_tuner_status[tuner_id]
    return true if the tune deletion succeeded, and false if it failed
    ************************************************************/
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

	bool success = updateHackRFFrequency(this->frontend_tuner_status[idx], freq, 0.0);
	if (!success) {
		throw FRONTEND::BadParameterException("Failed to update center frequency");
	}
    resetStream(this->frontend_tuner_status[idx]);
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

    bool success = updateHackRFBandwidth(this->frontend_tuner_status[idx], bw, 20.0);
    if (!success) {
    	throw FRONTEND::BadParameterException("Failed to update bandwidth");
    }
    resetStream(this->frontend_tuner_status[idx]);
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

    // Update for RX_DIGITIZER tuner
    if (frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
    	bool success = updateHackRFGainRX(this->frontend_tuner_status[idx], gain, 20.0);
    	if (!success) {
    		float maxGain = One::ALLOWED_RX_VGA_VALUES.back() + One::ALLOWED_LNA_GAIN_VALUES.back();
    		std::stringstream ss;
    		ss << "RX-gain cannot be greater than " << maxGain << " dB";
    		throw FRONTEND::BadParameterException(ss.str().c_str());
    	}
    }

    if (frontend_tuner_status[idx].tuner_type == "RX_DIGITIZER") {
    	// TODO: Implement TX gain
    }
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

    if (enable) {
    	deviceEnable(this->frontend_tuner_status[idx], idx);
    } else {
    	deviceDisable(this->frontend_tuner_status[idx], idx);
    }
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

    bool success = updateHackRFSampleRate(this->frontend_tuner_status[idx], sr, 20.0);
    if (!success) {
    	throw FRONTEND::BadParameterException("Failed to update sample rate");
    }
    resetStream(this->frontend_tuner_status[idx]);
}

double HackRFOne_i::getTunerOutputSampleRate(const std::string& allocation_id){
    long idx = getTunerMapping(allocation_id);
    if (idx < 0) throw FRONTEND::FrontendException("Invalid allocation id");
    return frontend_tuner_status[idx].sample_rate;
}

std::string HackRFOne_i::errorString(const std::string& message, int status) {
	return message + " - Reason: " + hackrf_error_name(static_cast<hackrf_error>(status));
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

