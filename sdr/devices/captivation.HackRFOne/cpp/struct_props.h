#ifndef STRUCTPROPS_H
#define STRUCTPROPS_H

/*******************************************************************************************

    AUTO-GENERATED CODE. DO NOT MODIFY

*******************************************************************************************/

#include <ossie/CorbaUtils.h>
#include <CF/cf.h>
#include <ossie/PropertyMap.h>
#include <bulkio/bulkio.h>
typedef bulkio::connection_descriptor_struct connection_descriptor_struct;

#include <frontend/fe_tuner_struct_props.h>

struct frontend_tuner_status_struct_struct : public frontend::default_frontend_tuner_status_struct_struct {
    frontend_tuner_status_struct_struct () : frontend::default_frontend_tuner_status_struct_struct()
    {
    };

    static std::string getId() {
        return std::string("FRONTEND::tuner_status_struct");
    };

    std::string available_bandwidth;
    std::string available_frequency;
    std::string available_gain;
    std::string available_sample_rate;
    double gain;
    bool scan_mode_enabled;
    bool supports_scan;
};

inline bool operator>>= (const CORBA::Any& a, frontend_tuner_status_struct_struct& s) {
    CF::Properties* temp;
    if (!(a >>= temp)) return false;
    const redhawk::PropertyMap& props = redhawk::PropertyMap::cast(*temp);
    if (props.contains("FRONTEND::tuner_status::allocation_id_csv")) {
        if (!(props["FRONTEND::tuner_status::allocation_id_csv"] >>= s.allocation_id_csv)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_bandwidth")) {
        if (!(props["FRONTEND::tuner_status::available_bandwidth"] >>= s.available_bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_frequency")) {
        if (!(props["FRONTEND::tuner_status::available_frequency"] >>= s.available_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_gain")) {
        if (!(props["FRONTEND::tuner_status::available_gain"] >>= s.available_gain)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::available_sample_rate")) {
        if (!(props["FRONTEND::tuner_status::available_sample_rate"] >>= s.available_sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::bandwidth")) {
        if (!(props["FRONTEND::tuner_status::bandwidth"] >>= s.bandwidth)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::center_frequency")) {
        if (!(props["FRONTEND::tuner_status::center_frequency"] >>= s.center_frequency)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::enabled")) {
        if (!(props["FRONTEND::tuner_status::enabled"] >>= s.enabled)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::gain")) {
        if (!(props["FRONTEND::tuner_status::gain"] >>= s.gain)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::group_id")) {
        if (!(props["FRONTEND::tuner_status::group_id"] >>= s.group_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::rf_flow_id")) {
        if (!(props["FRONTEND::tuner_status::rf_flow_id"] >>= s.rf_flow_id)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::sample_rate")) {
        if (!(props["FRONTEND::tuner_status::sample_rate"] >>= s.sample_rate)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::scan_mode_enabled")) {
        if (!(props["FRONTEND::tuner_status::scan_mode_enabled"] >>= s.scan_mode_enabled)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::supports_scan")) {
        if (!(props["FRONTEND::tuner_status::supports_scan"] >>= s.supports_scan)) return false;
    }
    if (props.contains("FRONTEND::tuner_status::tuner_type")) {
        if (!(props["FRONTEND::tuner_status::tuner_type"] >>= s.tuner_type)) return false;
    }
    return true;
}

inline void operator<<= (CORBA::Any& a, const frontend_tuner_status_struct_struct& s) {
    redhawk::PropertyMap props;
 
    props["FRONTEND::tuner_status::allocation_id_csv"] = s.allocation_id_csv;
 
    props["FRONTEND::tuner_status::available_bandwidth"] = s.available_bandwidth;
 
    props["FRONTEND::tuner_status::available_frequency"] = s.available_frequency;
 
    props["FRONTEND::tuner_status::available_gain"] = s.available_gain;
 
    props["FRONTEND::tuner_status::available_sample_rate"] = s.available_sample_rate;
 
    props["FRONTEND::tuner_status::bandwidth"] = s.bandwidth;
 
    props["FRONTEND::tuner_status::center_frequency"] = s.center_frequency;
 
    props["FRONTEND::tuner_status::enabled"] = s.enabled;
 
    props["FRONTEND::tuner_status::gain"] = s.gain;
 
    props["FRONTEND::tuner_status::group_id"] = s.group_id;
 
    props["FRONTEND::tuner_status::rf_flow_id"] = s.rf_flow_id;
 
    props["FRONTEND::tuner_status::sample_rate"] = s.sample_rate;
 
    props["FRONTEND::tuner_status::scan_mode_enabled"] = s.scan_mode_enabled;
 
    props["FRONTEND::tuner_status::supports_scan"] = s.supports_scan;
 
    props["FRONTEND::tuner_status::tuner_type"] = s.tuner_type;
    a <<= props;
}

inline bool operator== (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    if (s1.allocation_id_csv!=s2.allocation_id_csv)
        return false;
    if (s1.available_bandwidth!=s2.available_bandwidth)
        return false;
    if (s1.available_frequency!=s2.available_frequency)
        return false;
    if (s1.available_gain!=s2.available_gain)
        return false;
    if (s1.available_sample_rate!=s2.available_sample_rate)
        return false;
    if (s1.bandwidth!=s2.bandwidth)
        return false;
    if (s1.center_frequency!=s2.center_frequency)
        return false;
    if (s1.enabled!=s2.enabled)
        return false;
    if (s1.gain!=s2.gain)
        return false;
    if (s1.group_id!=s2.group_id)
        return false;
    if (s1.rf_flow_id!=s2.rf_flow_id)
        return false;
    if (s1.sample_rate!=s2.sample_rate)
        return false;
    if (s1.scan_mode_enabled!=s2.scan_mode_enabled)
        return false;
    if (s1.supports_scan!=s2.supports_scan)
        return false;
    if (s1.tuner_type!=s2.tuner_type)
        return false;
    return true;
}

inline bool operator!= (const frontend_tuner_status_struct_struct& s1, const frontend_tuner_status_struct_struct& s2) {
    return !(s1==s2);
}

#endif // STRUCTPROPS_H
