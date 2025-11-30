#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>

// Maximum number of neighbors that can be tracked
#define AP_SWARM_MAX_NEIGHBORS_DEFAULT 5

class AP_Swarm
{
public:
    // Constructor
    AP_Swarm();

    // Do not allow copies
    CLASS_NO_COPY(AP_Swarm);

    // Get singleton instance
    static AP_Swarm *get_singleton(void)
    {
        return _singleton;
    }

    // Initialize the swarm library
    void init();

    // Main update function - call at regular intervals (e.g., 10 Hz)
    void update();

    // Handle incoming MAVLink messages
    void handle_global_position_int(const mavlink_global_position_int_t &packet, uint8_t sysid);
    void handle_follow_target(const mavlink_follow_target_t &packet, uint8_t sysid);

    // Check if swarm mode is enabled
    bool enabled() const { return _enabled; }

    // Check if swarm has valid data
    bool have_target() const;

    // Get desired position for this vehicle in swarm
    bool get_target_location_and_velocity(Location &loc, Vector3f &vel_ned);

    // Get number of active neighbors
    uint8_t get_active_neighbor_count() const { return _active_neighbor_count; }

    uint8_t get_debug_level() const { return _debug; }

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

    // Formation types
    enum class FormationType : uint8_t
    {
        DISABLED = 0,
        LEADER_ONLY = 1,
        CIRCLE = 2,
        HORIZ_LINE = 3,
        VERT_LINE = 4,
        GRID = 5
    };

private:
    static AP_Swarm *_singleton;

    // Per-target data structure
    struct TargetEntry
    {
        uint8_t sysid;           // MAVLink system ID of this target
        bool active;             // Is this entry currently active/valid?
        uint32_t last_update_ms; // Last time data was received

        // Position and velocity data (NED frame relative to EKF origin)
        Vector3f pos_ned;   // Position in meters (North, East, Down)
        Vector3f vel_ned;   // Velocity in m/s (North, East, Down)
        Vector3f accel_ned; // Acceleration in m/s^2 (North, East, Down)

        // Raw data from messages (before conversion)
        Location location; // Global position (lat, lon, alt)

        // Heading information
        float heading_rad;       // Heading in radians
        float heading_rate_rads; // Heading rate in rad/s

        // Timing and jitter correction
        uint32_t msg_time_ms;   // Message timestamp from sender
        int32_t time_offset_ms; // Time offset correction for jitter

        // Message source tracking
        bool has_follow_target; // True if FOLLOW_TARGET data available

        // Constructor
        TargetEntry() : sysid(0),
                        active(false),
                        last_update_ms(0),
                        pos_ned(),
                        vel_ned(),
                        accel_ned(),
                        location(),
                        heading_rad(0.0f),
                        heading_rate_rads(0.0f),
                        msg_time_ms(0),
                        time_offset_ms(0),
                        has_follow_target(false)
        {
        }
    };

    // Array of target entries
    TargetEntry _targets[AP_SWARM_MAX_NEIGHBORS_DEFAULT];
    uint8_t _active_neighbor_count;

    // Find or create a target entry for a given sysid
    TargetEntry *get_target_entry(uint8_t sysid);

    // Find the leader target entry
    TargetEntry *get_leader_entry();

    // Remove stale targets
    void remove_stale_targets();

    // Convert global position to NED frame
    bool global_position_to_ned(const Location &loc, Vector3f &pos_ned);

    // Compute formation slot offset for this vehicle
    Vector3f compute_formation_offset(uint8_t slot_index);

    // Get sorted list of active sysids
    void get_sorted_active_sysids(uint8_t *sysids, uint8_t &count);

    // Find this vehicle's slot index in the formation
    int8_t get_my_slot_index();

    // Compute desired position based on formation and neighbors
    bool compute_desired_position(Vector3f &pos_ned, Vector3f &vel_ned);

    // Apply repulsive forces from nearby neighbors
    void apply_neighbor_repulsion(Vector3f &pos_ned, Vector3f &vel_ned);

    // References to vehicle systems (must be before parameters for initialization order)
    const AP_AHRS &_ahrs;

    // Parameters
    AP_Int8 _enabled;             // SWARM_ENABLE: Enable swarm mode
    AP_Int8 _formation_type;      // SWARM_FORMATION: Formation type
    AP_Float _radius;             // SWARM_RADIUS: Formation radius in meters
    AP_Float _spacing;            // SWARM_SPACING: Spacing between vehicles in meters
    AP_Int8 _leader_sysid;        // SWARM_LEADER_SYSID: System ID of leader
    AP_Int16 _neighbor_timeout_s; // SWARM_TIMEOUT: Neighbor timeout in s
    AP_Int8 _max_neighbors;       // SWARM_MAX_NEIGH: Maximum neighbors to track
    AP_Float _attract_gain;       // SWARM_ATTR_GAIN: Attraction gain
    AP_Float _repel_gain;         // SWARM_REPEL_GAIN: Repulsion gain
    AP_Float _repel_distance;     // SWARM_REPEL_DIST: Repulsion activation distance
    AP_Int8 _alt_type;            // SWARM_ALT_TYPE: Altitude frame type
    AP_Int8 _debug;               // SWARM_DEBUG: Debug level

    // Internal state
    uint32_t _last_update_ms;  // Last time update() was called
    Vector3f _desired_pos_ned; // Desired position output
    Vector3f _desired_vel_ned; // Desired velocity output
    bool _have_target;         // True if we have a valid target
};

namespace AP
{
    AP_Swarm *swarm();
};
