/*
 * HackRFConstants.h
 *
 *  Created on: Nov 17, 2017
 *      Author: daniel.barbalato
 */

#ifndef HACKRFCONSTANTS_H_
#define HACKRFCONSTANTS_H_

#include <vector>

namespace HackRFConstants {

struct One {
	static const float DEFAULT_LNA_GAIN = 16.0;
	static const float DEFAULT_VGA_GAIN = 16.0;

    static const double MIN_RF_FREQ = 0.0;
    static const double MAX_RF_FREQ = 7250.0e6;
    static const double FILTER_BW_MIN = 1.75e6;
    static const double FILTER_BW_MAX = 28.0e6;

    static const double LNA_STEP_SIZE = 8.0;
    static const double RX_VGA_STEP_SIZE = 2.0;
    static const double TX_VGA_STEP_SIZE = 1.0;

    static const double FILTER_BW_VALUES[];
    static const double SAMPLE_RATE_VALUES[];
    static const double LNA_GAIN_VALUES[];
	static const double RX_VGA_GAIN_VALUES[];
	static const double TX_VGA_GAIN_VALUES[];

    static const std::vector<double> ALLOWED_BW_FILTERS;
    static const std::vector<double> ALLOWED_SAMPLE_RATES;
	static const std::vector<double> ALLOWED_LNA_GAIN_VALUES;
	static const std::vector<double> ALLOWED_RX_VGA_VALUES;
	static const std::vector<double> ALLOWED_TX_VGA_VALUES;
};

const double One::FILTER_BW_VALUES[] = {1.75e6,  2.5e6,  3.5e6,  5.0e6,  5.5e6,  6.0e6,  7.0e6,  8.0e6,
		                                 9.0e6, 10.0e6, 12.0e6, 14.0e6, 15.0e6, 20.0e6, 24.0e6, 28.0e6};
const double One::SAMPLE_RATE_VALUES[] = {4.0e6, 8.0e6, 10.0e6, 12.5e6, 16.0e6, 20.0e6};
const double One::LNA_GAIN_VALUES[] = {0.0, 8.0, 16.0, 24.0, 32.0, 40.0};
const double One::RX_VGA_GAIN_VALUES[] = { 0.0,  2.0,  4.0,  6.0,  8.0, 10.0, 12.0, 14.0,
		                                  16.0, 18.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0,
										  32.0, 34.0, 36.0, 38.0, 40.0, 42.0, 44.0, 46.0,
										  48.0, 50.0, 52.0, 54.0, 56.0, 58.0, 60.0, 62.0};
const double One::TX_VGA_GAIN_VALUES[] = { 0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,
		                                  10.0, 11,0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0,
                                          20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0,
										  30.0, 31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0,
										  40.0, 41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0};

const std::vector<double> One::ALLOWED_BW_FILTERS = std::vector<double>(FILTER_BW_VALUES, FILTER_BW_VALUES+16);
const std::vector<double> One::ALLOWED_SAMPLE_RATES = std::vector<double>(SAMPLE_RATE_VALUES, SAMPLE_RATE_VALUES+6);
const std::vector<double> One::ALLOWED_LNA_GAIN_VALUES = std::vector<double>(LNA_GAIN_VALUES, LNA_GAIN_VALUES+6);
const std::vector<double> One::ALLOWED_RX_VGA_VALUES = std::vector<double>(RX_VGA_GAIN_VALUES, RX_VGA_GAIN_VALUES+32);
const std::vector<double> One::ALLOWED_TX_VGA_VALUES = std::vector<double>(TX_VGA_GAIN_VALUES, TX_VGA_GAIN_VALUES+48);

}; // End HackRFConstants namespace

#endif /* HACKRFCONSTANTS_H_ */
