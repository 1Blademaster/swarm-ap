#include "AP_Swarm.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// Singleton instance
AP_Swarm *AP_Swarm::_singleton;

// Parameter table
const AP_Param::GroupInfo AP_Swarm::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Swarm Enable
    // @Description: Enable swarm mode
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Swarm, _enabled, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: FORMATION
    // @DisplayName: Formation Type
    // @Description: Type of formation for swarm
    // @Values: 0:Leader Only,1:Circle,2:Horizontal Line,3:Vertical Line,4:Grid
    // @User: Standard
    AP_GROUPINFO("FORMATION", 2, AP_Swarm, _formation_type, (uint8_t)FormationType::CIRCLE),

    // @Param: RADIUS
    // @DisplayName: Formation Radius
    // @Description: Radius of circular formation in meters
    // @Units: m
    // @Range: 1 100
    // @User: Standard
    AP_GROUPINFO("RADIUS", 3, AP_Swarm, _radius, 25.0f),

    // @Param: SPACING
    // @DisplayName: Formation Spacing
    // @Description: Spacing between vehicles in line/grid formations
    // @Units: m
    // @Range: 1 50
    // @User: Standard
    AP_GROUPINFO("SPACING", 4, AP_Swarm, _spacing, 20.0f),

    // @Param: LEADER_ID
    // @DisplayName: Leader System ID
    // @Description: MAVLink system ID of the leader vehicle
    // @Range: 1 255
    // @User: Standard
    AP_GROUPINFO("LEADER_ID", 5, AP_Swarm, _leader_sysid, 1),

    // @Param: TIMEOUT
    // @DisplayName: Neighbor Timeout
    // @Description: Time in seconds before a neighbor is considered lost
    // @Units: s
    // @Range: 1 30
    // @User: Standard
    AP_GROUPINFO("TIMEOUT", 6, AP_Swarm, _neighbor_timeout_s, 2.0f),

    // @Param: MAX_NEIGH
    // @DisplayName: Max Neighbors
    // @Description: Maximum number of neighbors to track
    // @Range: 1 10
    // @User: Advanced
    AP_GROUPINFO("MAX_NEIGH", 7, AP_Swarm, _max_neighbors, AP_SWARM_MAX_NEIGHBORS_DEFAULT),

    // @Param: REPEL_DIST
    // @DisplayName: Repulsion Distance
    // @Description: Distance threshold for neighbor repulsion
    // @Units: m
    // @Range: 1 20
    // @User: Advanced
    AP_GROUPINFO("REPEL_DIST", 10, AP_Swarm, _repel_distance, 20.0f),

    // @Param: ALT_TYPE
    // @DisplayName: Altitude Type
    // @Description: Altitude frame type for swarm operations
    // @Values: 0:Absolute,1:Relative,2:Terrain
    // @User: Advanced
    AP_GROUPINFO("ALT_TYPE", 11, AP_Swarm, _alt_type, 1),

    // @Param: DEBUG
    // @DisplayName: Debug Level
    // @Description: Debug output level
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("DEBUG", 12, AP_Swarm, _debug, 2),

    AP_GROUPEND};

// Constructor
AP_Swarm::AP_Swarm() : _active_neighbor_count(0),
                       _ahrs(AP::ahrs()),
                       _last_update_ms(0),
                       _have_target(false),
                       _last_unknown_formation_msg_time_ms(0)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Initialize the swarm library
void AP_Swarm::init()
{
    if (!_enabled)
    {
        return;
    }

    // Initialize all target entries
    for (uint8_t i = 0; i < _max_neighbors; i++)
    {
        _targets[i] = TargetEntry();
    }

    _active_neighbor_count = 0;
    _have_target = false;

    gcs().send_text(MAV_SEVERITY_INFO, "Swarm (%d): Initialized", (int)mavlink_system.sysid);
}

// Main update function - call at regular intervals
void AP_Swarm::update()
{
    if (!_enabled)
    {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // Limit update rate
    if (now_ms - _last_update_ms < 100)
    { // 10 Hz max
        return;
    }
    _last_update_ms = now_ms;

    Vector3f my_pos;
    if (!_ahrs.get_relative_position_NED_origin_float(my_pos))
    {
        // Can't get position, skip update
        return;
    }

    // Remove stale targets
    remove_stale_targets();

    if (mavlink_system.sysid == _leader_sysid)
    {
        // Leader does not compute formation position
        _have_target = false;
        return;
    }
    // Compute desired position based on formation
    _have_target = compute_desired_position(_desired_pos_ned);

    // Debug output
    if (_debug >= 3 && _have_target)
    {
        printf("Swarm (%d): Active neighbors: %d, Desired NED: %.1f,%.1f,%.1f\n",
               (int)mavlink_system.sysid,
               _active_neighbor_count,
               (double)_desired_pos_ned.x,
               (double)_desired_pos_ned.y,
               (double)_desired_pos_ned.z);
    }
}

// Handle GLOBAL_POSITION_INT message
void AP_Swarm::handle_global_position_int(const mavlink_global_position_int_t &packet, uint8_t sysid)
{
    if (!_enabled)
    {
        return;
    }

    // Don't track our own position
    if (sysid == mavlink_system.sysid)
    {
        return;
    }

    // Ignore GCS heartbeats for swarm tracking
    // if (sysid == 255)
    // {
    //     return;
    // }

    // Get target entry
    TargetEntry *target = get_target_entry(sysid);
    if (target == nullptr)
    {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    // Update location from packet
    target->location.lat = packet.lat;
    target->location.lng = packet.lon;
    target->location.alt = packet.alt / 10; // Convert from mm to cm
    target->location.relative_alt = false;  // GLOBAL_POSITION_INT uses absolute altitude

    // Convert to NED frame
    if (!global_position_to_ned(target->location, target->pos_ned))
    {
        // Conversion failed, don't update this target
        return;
    }

    // Update velocity (convert from cm/s to m/s)
    target->vel_ned.x = packet.vx * 0.01f; // North
    target->vel_ned.y = packet.vy * 0.01f; // East
    target->vel_ned.z = packet.vz * 0.01f; // Down

    // Update heading (convert from centidegrees to radians)
    if (packet.hdg != UINT16_MAX)
    {
        target->heading_rad = radians(packet.hdg * 0.01f);
    }

    // Update timing
    target->last_position_update_ms = now_ms;
    target->position_msg_time_ms = packet.time_boot_ms;
    target->active = true;

    if (_debug >= 3)
    {
        printf("Swarm (%d): GLOBAL_POS from sysid %d at NED: %.1f,%.1f,%.1f\n",
               (int)mavlink_system.sysid,
               sysid,
               (double)target->pos_ned.x,
               (double)target->pos_ned.y,
               (double)target->pos_ned.z);
    }
}

void AP_Swarm::handle_heartbeat(const mavlink_heartbeat_t &packet, uint8_t sysid)
{
    if (!_enabled)
    {
        return;
    }

    // Don't track our own heartbeat
    if (sysid == mavlink_system.sysid || sysid == 255 || packet.type == MAV_TYPE_GCS)
    {
        return;
    }

    if ((packet.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) == 0)
    {
        // If in targets list, remove from the list when disarmed
        TargetEntry *target = get_target_entry(sysid);
        if (target != nullptr)
        {
            // Delete from targets list
            *target = TargetEntry(); // Reset entry
            if (_debug >= 1)
            {
                gcs().send_text(MAV_SEVERITY_INFO, "Swarm (%d): Removed target sysid %d (disarmed)", (int)mavlink_system.sysid, sysid);
            }
        }
    }
    else
    {
        // If not in targets list, then add when armed
        TargetEntry *target = get_target_entry(sysid);
        if (target == nullptr)
        {
            // Create new target entry
            for (uint8_t i = 0; i < _max_neighbors; i++)
            {
                if (_targets[i].deleted)
                {
                    _targets[i] = TargetEntry(); // Reset entry
                    _targets[i].sysid = sysid;
                    _targets[i].deleted = false;
                    _targets[i].active = false;

                    if (_debug >= 1)
                    {
                        gcs().send_text(MAV_SEVERITY_INFO, "Swarm (%d): Added target sysid %d (armed)", (int)mavlink_system.sysid, sysid);
                    }
                    break;
                }
            }
        }
        else
        {
            // Already in targets list, do nothing
        }
    }

    if (_debug >= 3)
    {
        printf("Swarm (%d): HEARTBEAT from sysid %d\n",
               (int)mavlink_system.sysid,
               sysid);
    }
}

// Find a target entry for a given sysid
AP_Swarm::TargetEntry *AP_Swarm::get_target_entry(uint8_t sysid)
{
    if (sysid == mavlink_system.sysid || sysid == 255)
    {
        // Invalid sysid or our own sysid
        return nullptr;
    }

    // First, try to find existing entry
    for (uint8_t i = 0; i < _max_neighbors; i++)
    {
        if (_targets[i].deleted)
        {
            continue;
        }

        if (_targets[i].sysid == sysid)
        {
            return &_targets[i];
        }
    }

    return nullptr;
}

// Find the leader target entry
AP_Swarm::TargetEntry *AP_Swarm::get_leader_entry()
{
    for (uint8_t i = 0; i < _max_neighbors; i++)
    {
        if (!_targets[i].deleted && _targets[i].active && _targets[i].sysid == _leader_sysid)
        {
            // printf("Swarm (%d): Leader sysid %d found in slot %d\n", (int)mavlink_system.sysid, (int)_leader_sysid, i);
            return &_targets[i];
        }
        else if (_debug >= 2)
        {
            printf("Swarm (%d): Leader sysid %d not found in slot %d, target %d is active: %d\n", (int)mavlink_system.sysid, (int)_leader_sysid, i, (int)_targets[i].sysid, _targets[i].active);
        }
    }
    return nullptr;
}

// Remove stale targets
void AP_Swarm::remove_stale_targets()
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t timeout_ms = _neighbor_timeout_s * 1000; // Convert from s to ms

    _active_neighbor_count = 0;

    for (uint8_t i = 0; i < _max_neighbors; i++)
    {
        if (_targets[i].active)
        {
            if (now_ms - _targets[i].last_position_update_ms > timeout_ms)
            {
                // Target has timed out
                if (_debug >= 1)
                {
                    uint32_t time_since_update = now_ms - _targets[i].last_position_update_ms;
                    gcs().send_text(MAV_SEVERITY_INFO, "Swarm (%d): Removed stale target sysid %d (elapsed: %u ms, threshold: %u ms)",
                                    (int)mavlink_system.sysid, _targets[i].sysid,
                                    (unsigned int)time_since_update, (unsigned int)timeout_ms);
                }
                _targets[i].active = false;
            }
            else
            {
                _active_neighbor_count++;
            }
        }
    }
}

// Convert global position to NED frame relative to EKF origin
bool AP_Swarm::global_position_to_ned(const Location &loc, Vector3f &pos_ned)
{
    // Get EKF origin
    Location ekf_origin;
    if (!_ahrs.get_origin(ekf_origin))
    {
        return false;
    }

    // Calculate offset from origin
    Vector2f diff_ne = ekf_origin.get_distance_NE(loc);
    pos_ned.x = diff_ne.x;
    pos_ned.y = diff_ne.y;
    pos_ned.z = -(loc.alt - ekf_origin.alt) * 0.01f; // Convert from cm to m, and flip to Down

    return true;
}

// Compute formation slot offset for this vehicle
Vector3f AP_Swarm::compute_formation_offset(uint8_t slot_index)
{
    Vector3f offset;

    FormationType formation = (FormationType)_formation_type.get();

    switch (formation)
    {
    case FormationType::LEADER_ONLY:
        // For LEADER_ONLY, space followers out in a simple pattern behind/beside the leader
        // Use SWARM_SPACING parameter
        if (slot_index == 0)
        {
            // This is the leader
            offset.zero();
        }
        else
        {
            // Simple pattern: spread followers behind and to the side
            float spacing = _spacing;

            // Alternate left/right placement
            uint8_t follower_index = slot_index - 1; // 0-indexed for followers
            bool is_left = (follower_index % 2) == 0;

            // Place followers behind leader in staggered formation
            offset.x = -spacing * ((follower_index / 2) + 1);      // Behind (negative North)
            offset.y = is_left ? -spacing * 0.5f : spacing * 0.5f; // Left or right
            offset.z = 0.0f;                                       // Same altitude
        }
        break;

    case FormationType::CIRCLE:
    {
        // Arrange vehicles in a circle around leader

        // Checks there are at least 2 other drones to form a circle
        if (_active_neighbor_count <= 1)
        {
            offset.zero();
            break;
        }

        // Skip leader (index 0), so actual follower index is slot_index - 1
        if (slot_index == 0)
        {
            // This is the leader
            offset.zero();
        }
        else
        {
            float angle = 2.0f * M_PI * (slot_index - 1) / (_active_neighbor_count);
            float radius = _radius;
            offset.x = radius * cosf(angle); // North
            offset.y = radius * sinf(angle); // East
            offset.z = 0.0f;                 // Same altitude as leader
        }
        break;
    }

    case FormationType::HORIZ_LINE:
    {
        // Arrange vehicles in a line next to leader
        float spacing = _spacing;
        offset.x = 0.0f;
        offset.y = spacing * slot_index; // Line extends to the east (positive East)
        offset.z = 0.0f;
        break;
    }

    case FormationType::VERT_LINE:
    {
        // Arrange vehicles in a line behind leader
        float spacing = _spacing;
        offset.x = -spacing * slot_index; // Line extends to the south (negative North)
        offset.y = 0.0f;
        offset.z = 0.0f;
        break;
    }

    case FormationType::GRID:
    {
        // Arrange vehicles in a grid pattern
        float spacing = _spacing;
        uint8_t grid_width = 3; // TODO: Make this configurable
        uint8_t row = slot_index / grid_width;
        uint8_t col = slot_index % grid_width;
        offset.x = -spacing * row;                   // Rows extend south
        offset.y = spacing * (col - grid_width / 2); // Columns centered
        offset.z = 0.0f;
        break;
    }

    default:
        offset.zero();

        // Send a message to GCS every 5 seconds
        uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_unknown_formation_msg_time_ms > 5000)
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "Swarm (%d): Unknown formation type (%d), holding position",
                            (int)mavlink_system.sysid, (int)formation);
            _last_unknown_formation_msg_time_ms = now_ms;
        }
        break;
    }

    return offset;
}

// Get sorted list of active sysids
void AP_Swarm::get_sorted_active_sysids(uint8_t *sysids, uint8_t &count)
{
    count = 0;

    // Collect active sysids
    for (uint8_t i = 0; i < _max_neighbors && count < _max_neighbors; i++)
    {
        if (_targets[i].active)
        {
            sysids[count++] = _targets[i].sysid;
        }
    }

    // Add our own sysid if not already in the list
    uint8_t my_sysid = mavlink_system.sysid;
    bool found_self = false;
    for (uint8_t i = 0; i < count; i++)
    {
        if (sysids[i] == my_sysid)
        {
            found_self = true;
            break;
        }
    }

    if (!found_self && count < _max_neighbors)
    {
        sysids[count++] = my_sysid;
    }

    // Simple bubble sort (fine for small arrays)
    for (uint8_t i = 0; i < count - 1; i++)
    {
        for (uint8_t j = 0; j < count - i - 1; j++)
        {
            if (sysids[j] > sysids[j + 1])
            {
                uint8_t temp = sysids[j];
                sysids[j] = sysids[j + 1];
                sysids[j + 1] = temp;
            }
        }
    }

    // Print slots
    if (_debug >= 2)
    {
        printf("Swarm (%d): Active sysids in slots:", (int)mavlink_system.sysid);
        for (uint8_t i = 0; i < count; i++)
        {
            printf("[%d] %d", i, (int)sysids[i]);
        }
        printf("\n");
    }
}

// Find this vehicle's slot index in the formation
int8_t AP_Swarm::get_my_slot_index()
{
    uint8_t sysids[(int)_max_neighbors];
    uint8_t count;

    get_sorted_active_sysids(sysids, count);

    // Find our sysid in the sorted list
    uint8_t my_sysid = mavlink_system.sysid;

    // If we're the leader, return 0
    if (my_sysid == _leader_sysid)
    {
        return 0;
    }

    // For followers, count how many vehicles come before us (excluding leader)
    int8_t follower_index = 0;
    for (uint8_t i = 0; i < count; i++)
    {
        if (sysids[i] == my_sysid)
        {
            // Found ourselves, return our follower index (0-based, excluding leader)
            return follower_index + 1; // +1 because leader is at index 0
        }

        // Don't count the leader in follower indices
        if (sysids[i] != _leader_sysid)
        {
            follower_index++;
        }
    }

    // Not found in active list (shouldn't happen)
    return -1;
}

// Compute desired position based on formation and neighbors
bool AP_Swarm::compute_desired_position(Vector3f &pos_ned)
{
    // Get leader position
    TargetEntry *leader = get_leader_entry();
    if (leader == nullptr)
    {
        if (_debug >= 1)
        {
            gcs().send_text(MAV_SEVERITY_WARNING, "Swarm (%d): No leader found (sysid %d)", (int)mavlink_system.sysid, (int)_leader_sysid);
        }
        return false;
    }

    // Get our slot index
    int8_t slot_index = get_my_slot_index();
    if (slot_index < 0)
    {
        if (_debug >= 2)
        {
            printf("Swarm (%d): My sysid %d not found in active list\n", (int)mavlink_system.sysid, (int)mavlink_system.sysid);
        }
        return false;
    }

    Vector3f my_current_pos;
    if (!_ahrs.get_relative_position_NED_origin_float(my_current_pos))
    {
        return false;
    }

    // Compute formation offset
    Vector3f offset = compute_formation_offset(slot_index);

    // If offset is zero, then don't change position at all
    if (offset.is_zero())
    {
        pos_ned = my_current_pos;
        return true;
    }

    const float prediction_time_s = 2.0f;
    Vector3f leader_velocity_prediction = leader->vel_ned * prediction_time_s;
    Vector3f leader_position_predicted = leader->pos_ned + leader_velocity_prediction;
    pos_ned = leader_position_predicted + offset;

    if (_debug >= 2)
    {
        printf("Swarm (%d): Leader pos: (%.2f,%.2f), Vel: (%.2f,%.2f), Predicted pos: (%.2f,%.2f), Offset: (%.2f,%.2f), Desired pos: (%.2f,%.2f), Slot idx: %d\n",
               (int)mavlink_system.sysid,
               (double)leader->pos_ned.x, (double)leader->pos_ned.y,
               (double)leader->vel_ned.x, (double)leader->vel_ned.y,
               (double)leader_position_predicted.x, (double)leader_position_predicted.y,
               (double)offset.x, (double)offset.y,
               (double)pos_ned.x, (double)pos_ned.y,
               (int)slot_index);
    }

    // Apply neighbor repulsion
    apply_neighbor_repulsion(pos_ned);

    return true;
}

// Apply repulsive forces from nearby neighbors
void AP_Swarm::apply_neighbor_repulsion(Vector3f &pos_ned)
{
    // Get our CURRENT position (not desired position) for repulsion calculation
    Vector3f my_current_pos;
    Vector3f my_current_vel;
    if (!_ahrs.get_relative_position_NED_origin_float(my_current_pos) ||
        !_ahrs.get_velocity_NED(my_current_vel))
    {
        printf("Swarm (%d): Unable to get own position or velocity for repulsion\n", (int)mavlink_system.sysid);
        return;
    }

    for (uint8_t i = 0; i < _max_neighbors; i++)
    {
        if (!_targets[i].active)
        {
            continue;
        }

        // Don't repel from self
        if (_targets[i].sysid == mavlink_system.sysid)
        {
            continue;
        }

        apply_path_repulsion(my_current_pos, my_current_vel, pos_ned, _targets[i]);
    }
}

void AP_Swarm::apply_path_repulsion(Vector3f &my_current_pos, Vector3f &my_current_vel, Vector3f &pos_ned, TargetEntry &target)
{
    Vector2f target_vel_xy = Vector2f(target.vel_ned.x, target.vel_ned.y);
    float target_speed_xy = target_vel_xy.length();
    if (target_speed_xy > 0.5f) // Faster than 0.5m/s
    {
        Vector2f target_pos_xy(target.pos_ned.x, target.pos_ned.y);
        Vector2f my_pos_xy(my_current_pos.x, my_current_pos.y);

        // Get direction by normalizing velocity
        Vector2f target_dir_xy = target_vel_xy;
        target_dir_xy.normalize();

        // Vector from target to me
        Vector2f to_me_xy = my_pos_xy - target_pos_xy;

        // Calculate path projection length based on repel distance and target speed (higher speed, longer distance) for 3 seconds
        float path_projection_length = target_speed_xy * 3.0f;

        // Project to_me_xy onto target_dir_xy to find closest point on target's path
        float projection_length = to_me_xy.dot(target_dir_xy);
        if (projection_length >= -5.0f && projection_length <= path_projection_length)
        {
            Vector2f closest_point_on_path_xy = target_pos_xy + target_dir_xy * projection_length;

            // Get distance from me to closest point on path
            Vector2f path_to_me = my_pos_xy - closest_point_on_path_xy;
            float dist_to_path = path_to_me.length();

            float path_repel_distance = _repel_distance;

            if (dist_to_path < path_repel_distance)
            {
                // Repulsion vector points awy from path
                Vector2f repulsion_xy = path_to_me;
                repulsion_xy.normalize(); // Make it unit length

                float distance_difference = path_repel_distance - dist_to_path;
                repulsion_xy *= distance_difference; // Scale by how close we are

                // Apply repulsion to desired position
                pos_ned.x += repulsion_xy.x;
                pos_ned.y += repulsion_xy.y;

                if (_debug >= 2)
                {
                    printf("Swarm (%d): Repelling from target path, actual pos: (%.2f,%.2f), target pos: (%.2f,%.2f), repulsion: (%.2f,%.2f) from path point (%.2f,%.2f), dist to path %.2f\n",
                           (int)mavlink_system.sysid,
                           (double)my_current_pos.x, (double)my_current_pos.y,
                           (double)pos_ned.x, (double)pos_ned.y,
                           (double)repulsion_xy.x, (double)repulsion_xy.y,
                           (double)closest_point_on_path_xy.x, (double)closest_point_on_path_xy.y,
                           (double)dist_to_path);
                }
            }
        }
    }
    else
    { // target is not moving
        Vector2f target_pos_xy(target.pos_ned.x, target.pos_ned.y);
        Vector2f my_pos_xy(my_current_pos.x, my_current_pos.y);
        Vector2f my_vel_xy(my_current_vel.x, my_current_vel.y);
        float my_speed_xy = my_vel_xy.length();

        if (my_speed_xy > 0.5f) // Moving faster than 0.5 m/s
        {
            // Use my actual velocity direction for path prediction
            Vector2f my_direction_xy = my_vel_xy;
            my_direction_xy.normalize();

            // Vector from my current position to the stationary target
            Vector2f to_target_xy = target_pos_xy - my_pos_xy;

            // Project target position onto my path
            float projection_length = to_target_xy.dot(my_direction_xy);

            // Calculate how far ahead to look based on my speed, 3 seconds ahead
            float my_lookahead_distance = my_speed_xy * 3.0f;

            // Only check if target is ahead of me on my path
            if (projection_length > 0.0f && projection_length < my_lookahead_distance)
            {
                // Find closest point on my path to the target
                Vector2f closest_point_on_my_path = my_pos_xy + my_direction_xy * projection_length;

                // Distance from target to my path
                Vector2f target_to_my_path = target_pos_xy - closest_point_on_my_path;
                float dist_from_target_to_my_path = target_to_my_path.length();

                if (dist_from_target_to_my_path < _repel_distance)
                {
                    // My path will come too close to the stationary target
                    // Apply repulsion perpendicular to my path
                    Vector2f repulsion_xy = target_to_my_path * -1.0f; // Push away from target
                    repulsion_xy.normalize();

                    float distance_difference = _repel_distance - dist_from_target_to_my_path;
                    repulsion_xy *= distance_difference;

                    pos_ned.x += repulsion_xy.x;
                    pos_ned.y += repulsion_xy.y;

                    if (_debug >= 2)
                    {
                        printf("Swarm (%d): My path collision with stationary target sysid %d - dist_to_my_path: %.2f, projection: %.2f, my_speed: %.2f, repulsion: (%.2f,%.2f)\n",
                               (int)mavlink_system.sysid, target.sysid,
                               (double)dist_from_target_to_my_path, (double)projection_length, (double)my_speed_xy,
                               (double)repulsion_xy.x, (double)repulsion_xy.y);
                    }
                }
            }
        }
        else
        {
            // I'm not moving much, use simple point-to-point repulsion
            Vector2f to_target_xy = target_pos_xy - my_pos_xy;
            float dist_to_target = to_target_xy.length();

            if (dist_to_target < _repel_distance)
            {
                Vector2f repulsion_xy = to_target_xy * -1.0f;
                repulsion_xy.normalize();

                float distance_difference = _repel_distance - dist_to_target;
                repulsion_xy *= distance_difference;

                pos_ned.x += repulsion_xy.x;
                pos_ned.y += repulsion_xy.y;

                if (_debug >= 2)
                {
                    printf("Swarm (%d): POINT REPULSION from stationary target sysid %d - dist: %.2f, repulsion: (%.2f,%.2f)\n",
                           (int)mavlink_system.sysid, target.sysid,
                           (double)dist_to_target,
                           (double)repulsion_xy.x, (double)repulsion_xy.y);
                }
            }
        }
    }
}

// Check if swarm has valid target data
bool AP_Swarm::have_target() const
{
    return _have_target && _enabled;
}

// Get desired position and velocity for this vehicle in swarm
bool AP_Swarm::get_target_location(Location &loc)
{
    if (!_enabled || !have_target())
    {
        return false;
    }

    // Convert NED position back to global location
    Location ekf_origin;
    if (!_ahrs.get_origin(ekf_origin))
    {
        return false;
    }

    // Create location from NED offset
    loc = ekf_origin;
    loc.offset(_desired_pos_ned.x, _desired_pos_ned.y);
    loc.alt = ekf_origin.alt - _desired_pos_ned.z * 100.0f; // Convert m to cm and flip Down to up

    return true;
}

// Singleton accessor
namespace AP
{
    AP_Swarm *swarm()
    {
        return AP_Swarm::get_singleton();
    }
}
