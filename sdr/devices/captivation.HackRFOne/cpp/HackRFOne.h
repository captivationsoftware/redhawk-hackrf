#ifndef HACKRFONE_I_IMPL_H
#define HACKRFONE_I_IMPL_H

#include <hackrf.h>

#include "HackRFOne_base.h"

class HackRFOne_i : public HackRFOne_base
{
    ENABLE_LOGGING

    public:
        HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        HackRFOne_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~HackRFOne_i();

        void constructor();

        bool reset();

        bool resetDriver();

        void resetStream(frontend_tuner_status_struct_struct& fts);

        void addChannels(size_t num, const std::string& tunerType);

        int serviceFunction();

        int pushData();

        int hackrfRXCallback(hackrf_transfer* transfer);

        bool updateHackRFSampleRate(frontend_tuner_status_struct_struct &fts, double sampleRate, double tolerance);

        bool updateHackRFBandwidth(frontend_tuner_status_struct_struct &fts, double bandwidth, double tolerance);

        bool updateHackRFFrequency(frontend_tuner_status_struct_struct &fts, double freq, double tolerance);

        bool updateHackRFGainRX(frontend_tuner_status_struct_struct &fts, double gain, double tolerance);

        bool updateHackRFGainTX(frontend_tuner_status_struct_struct &fts, double gain, double tolerance);

    protected:
        std::string getTunerType(const std::string& allocation_id);
        bool getTunerDeviceControl(const std::string& allocation_id);
        std::string getTunerGroupId(const std::string& allocation_id);
        std::string getTunerRfFlowId(const std::string& allocation_id);
        double getTunerCenterFrequency(const std::string& allocation_id);
        void setTunerCenterFrequency(const std::string& allocation_id, double freq);
        double getTunerBandwidth(const std::string& allocation_id);
        void setTunerBandwidth(const std::string& allocation_id, double bw);
        bool getTunerAgcEnable(const std::string& allocation_id);
        void setTunerAgcEnable(const std::string& allocation_id, bool enable);
        float getTunerGain(const std::string& allocation_id);
        void setTunerGain(const std::string& allocation_id, float gain);
        long getTunerReferenceSource(const std::string& allocation_id);
        void setTunerReferenceSource(const std::string& allocation_id, long source);
        bool getTunerEnable(const std::string& allocation_id);
        void setTunerEnable(const std::string& allocation_id, bool enable);
        double getTunerOutputSampleRate(const std::string& allocation_id);
        void setTunerOutputSampleRate(const std::string& allocation_id, double sr);
        std::string get_rf_flow_id(const std::string& port_name);
        void set_rf_flow_id(const std::string& port_name, const std::string& id);
        frontend::RFInfoPkt get_rfinfo_pkt(const std::string& port_name);
        void set_rfinfo_pkt(const std::string& port_name, const frontend::RFInfoPkt& pkt);

    private:

        //! Pointer to the initialized HackRF device controller
        hackrf_device* _device;

        BULKIO::PrecisionUTCTime _timestamp;

        BULKIO::StreamSRI _currentSRI;

        bulkio::OutOctetStream _currentStream;

        std::vector< std::vector<unsigned char> > _buffers;

        std::deque< std::vector<unsigned char>* > _availableBuffers;

        std::deque< std::pair<std::vector<unsigned char>*, BULKIO::PrecisionUTCTime> > _filledBuffers;

        boost::mutex _availableLock;

        boost::mutex _filledLock;

        BULKIO::StreamSRI createSRI(std::string &stream_id, frontend_tuner_status_struct_struct &frontend_status, double collector_frequency = -1.0);

        std::string errorString(const std::string& message, int status);

        template <typename T>
        bool withinBounds(T& value, T min, T max, double tolerance=0.0) {
            T delta = static_cast<T>(value * (tolerance / 100.0));
            // Only adding tolerance-deltas to avoid negative unsigned types
            if (((value+delta) >= min) && (value <= (max+delta))) {
                value = std::max(std::min(value, max), min);
                return true;
            }
            return false;
        }

        template <typename T>
        bool withinBounds(T& value, double tolerance, std::vector<T> values) {
            // Find the nearest value in the set
            T closestValue = findNearest(value, values);

            // Perform closest-value checks using doubles (for std::fabs)
            double closestDouble = static_cast<double>(closestValue);
            double valueDouble = static_cast<double>(value);
            double maxDelta = valueDouble * (tolerance / 100.0);

            // Check if closest value is within tolerance
            if (std::fabs(closestDouble-valueDouble) >= maxDelta) {
                return false;
            }

            value = closestValue;
            return true;
        }

        template <typename T1, typename T2>
        T1 findNearest(T1 value, std::vector<T2> vals) {
            assert(vals.empty() == false);

            // Only one value in set - That value is the closest
            if (vals.size() == 1) return vals[0];

            // Sort values for faster minimum detection
            std::sort(vals.begin(), vals.end());

            // Perform closest-value checks using doubles (for std::fabs)
            double doubleValue = static_cast<double>(value);

            // Initial iterative parameters with the first
            T1 bestValue = static_cast<T1>(vals[0]);
            double lowestDelta = std::fabs(doubleValue - bestValue);

            // Iterate through remaining data points to find nearest
            for (size_t i = 1; i < vals.size(); i++) {
                double v = static_cast<double>(vals[i]);
                double delta = std::fabs(doubleValue - v);
                if (delta < lowestDelta) {
                    bestValue = static_cast<T1>(vals[i]);
                    lowestDelta = delta;
                } else {
                    // Since the values are sorted, any increase in error means
                    // the closest value was found on the previous iteration
                    break;
                }
            }
            return bestValue;
        }


        ////////////////////////////////////////
        // Required device specific functions // -- to be implemented by device developer
        ////////////////////////////////////////

        // these are pure virtual, must be implemented here
        void deviceEnable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        void deviceDisable(frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceSetTuning(const frontend::frontend_tuner_allocation_struct &request, frontend_tuner_status_struct_struct &fts, size_t tuner_id);
        bool deviceDeleteTuning(frontend_tuner_status_struct_struct &fts, size_t tuner_id);

};

#endif // HACKRFONE_I_IMPL_H
