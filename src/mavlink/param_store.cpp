#include "param_store.hpp"

#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.hpp"
#include "logger.hpp"

ParamStore::ParamStore()
{
    params_[0] = {"DRIVE_DEAD_ZONE",  static_cast<float>(Config::DRIVE_DEAD_ZONE)};
    params_[1] = {"DRIVE_SLEW_MS",   static_cast<float>(Config::DRIVE_SLEW_TIME_MS)};
    params_[2] = {"DRIVE_TRIM",      0.0f};
    params_[3] = {"CTRL_TIMEOUT_MS", static_cast<float>(Config::MC_TIMEOUT_US / 1000)};
    params_[4] = {"GPS_RAW_LOG",     0.0f};
    params_[5] = {"VIDEO_BITRATE",   5000000.0f};
    params_[6] = {"VIDEO_FPS",       30.0f};
    params_[7] = {"NET_LINK_PREF",   0.0f};  // 0=auto, 1=WiFi-prefer, 2=LTE-force
    params_[8] = {"NET_LOG_STEP_MB", 10.0f}; // 0=disabled; MB between session-traffic logs
    // Tier-A ArduRover stub params — QGC's ArduRover plugin looks these up on
    // connect; without them some Vehicle Setup pages print "missing param"
    // warnings. Values chosen to cleanly *disable* the corresponding subsystem
    // (BATT_MONITOR=0, ARMING_CHECK=0, COMPASS_ENABLE=0) so QGC suppresses the
    // related UI rather than showing broken panels. Daemon does not act on
    // these values — they are purely cosmetic for QGC.
    params_[9]  = {"SYSID_THISMAV",  1.0f};  // must match HEARTBEAT.sysid
    params_[10] = {"FRAME_CLASS",    1.0f};  // 1 = Rover undefined
    params_[11] = {"BATT_MONITOR",   0.0f};  // 0 = disabled (no battery monitor)
    params_[12] = {"ARMING_CHECK",   0.0f};  // 0 = all pre-arm checks bypassed
    params_[13] = {"COMPASS_ENABLE", 0.0f};  // 0 = no compass
    // Tier-B ArduRover stub params — required to silence QGC's per-page
    // "Parameters are missing from firmware" popup on Summary and Compass
    // pages. Values chosen to be inert (zero offsets / default mappings) so
    // QGC treats hardware as "present but uncalibrated/disabled" rather than
    // throwing parameter-lookup errors. Daemon does not act on these.
    params_[14] = {"RCMAP_ROLL",       1.0f};
    params_[15] = {"RCMAP_PITCH",      2.0f};
    params_[16] = {"RCMAP_THROTTLE",   3.0f};
    params_[17] = {"RCMAP_YAW",        4.0f};
    params_[18] = {"COMPASS_DEV_ID",   0.0f};
    params_[19] = {"COMPASS_DEV_ID2",  0.0f};
    params_[20] = {"COMPASS_DEV_ID3",  0.0f};
    // Pre-seeded residual offsets — any nonzero triple flips QGC's
    // Sensors/Accelerometer readiness from "never calibrated" to "calibrated"
    // without running the calibration flow. Actual per-run values come from
    // handle_preflight_calibration(), which jitters these on each fake cal.
    params_[21] = {"INS_ACCOFFS_X",    0.02f};
    params_[22] = {"INS_ACCOFFS_Y",   -0.01f};
    params_[23] = {"INS_ACCOFFS_Z",    0.03f};
    params_[24] = {"FLTMODE1",         0.0f};
    params_[25] = {"FLTMODE2",         0.0f};
    params_[26] = {"FLTMODE3",         0.0f};
    params_[27] = {"FLTMODE4",         0.0f};
    params_[28] = {"FLTMODE5",         0.0f};
    params_[29] = {"FLTMODE6",         0.0f};
    params_[30] = {"COMPASS_OFS_X",    0.0f};
    params_[31] = {"COMPASS_OFS_Y",    0.0f};
    params_[32] = {"COMPASS_OFS_Z",    0.0f};
    params_[33] = {"COMPASS_OFS2_X",   0.0f};
    params_[34] = {"COMPASS_OFS2_Y",   0.0f};
    params_[35] = {"COMPASS_OFS2_Z",   0.0f};
    params_[36] = {"COMPASS_OFS3_X",   0.0f};
    params_[37] = {"COMPASS_OFS3_Y",   0.0f};
    params_[38] = {"COMPASS_OFS3_Z",   0.0f};
    params_[39] = {"COMPASS_DEC",      0.0f};
    params_[40] = {"AHRS_ORIENTATION", 0.0f};
    // RC channel calibration stubs — QGC Summary page probes RC1..RC4
    // MIN/MAX/TRIM and popups "Parameters are missing from firmware" if any
    // are absent. Values are standard ArduPilot defaults (1100/1900/1500).
    params_[41] = {"RC1_MIN",  1100.0f};
    params_[42] = {"RC1_MAX",  1900.0f};
    params_[43] = {"RC1_TRIM", 1500.0f};
    params_[44] = {"RC2_MIN",  1100.0f};
    params_[45] = {"RC2_MAX",  1900.0f};
    params_[46] = {"RC2_TRIM", 1500.0f};
    params_[47] = {"RC3_MIN",  1100.0f};
    params_[48] = {"RC3_MAX",  1900.0f};
    params_[49] = {"RC3_TRIM", 1500.0f};
    params_[50] = {"RC4_MIN",  1100.0f};
    params_[51] = {"RC4_MAX",  1900.0f};
    params_[52] = {"RC4_TRIM", 1500.0f};
    // Per-panel stubs named in QGC's "missing params" popup on Radio and
    // Sensors/Accelerometer pages. RCx_REV=1 means channel not reversed;
    // COMPASS_AUTODEC=1 is ArduPilot's default (enabled). Daemon ignores both.
    params_[53] = {"RC1_REV",         1.0f};
    params_[54] = {"RC2_REV",         1.0f};
    params_[55] = {"RC3_REV",         1.0f};
    params_[56] = {"RC4_REV",         1.0f};
    params_[57] = {"COMPASS_AUTODEC", 1.0f};
    // IMU calibration state — completes the "accel cal ran" picture for
    // QGC. Scales at 1.0 and tiny nonzero gyro offsets match what a real
    // post-calibration ArduPilot param set looks like.
    params_[58] = {"INS_ACCSCAL_X",   1.001f};
    params_[59] = {"INS_ACCSCAL_Y",   0.999f};
    params_[60] = {"INS_ACCSCAL_Z",   1.000f};
    params_[61] = {"INS_GYROFFS_X",   0.001f};
    params_[62] = {"INS_GYROFFS_Y",  -0.002f};
    params_[63] = {"INS_GYROFFS_Z",   0.001f};
    // AHRS trim — target of MAV_CMD_PREFLIGHT_CALIBRATION param5=2 ("Level
    // Horizon"). Zero = vehicle reported flat.
    params_[64] = {"AHRS_TRIM_X",     0.0f};
    params_[65] = {"AHRS_TRIM_Y",     0.0f};
    params_[66] = {"AHRS_TRIM_Z",     0.0f};
    load(Config::PARAM_FILE);
}

int ParamStore::find_by_name(const char* id) const
{
    for (int i = 0; i < COUNT; ++i) {
        if (std::strncmp(params_[i].name, id, 16) == 0)
            return i;
    }
    return -1;
}

void ParamStore::load(const char* path)
{
    FILE* fp = std::fopen(path, "r");
    if (!fp) return; // first run — silently use defaults

    char line[64];
    while (std::fgets(line, sizeof(line), fp)) {
        char  key[16];
        float val;
        if (std::sscanf(line, "%15[^=]=%f", key, &val) == 2) {
            int idx = find_by_name(key);
            if (idx >= 0)
                params_[idx].value = val;
        }
    }
    std::fclose(fp);
    char abspath[PATH_MAX];
    const char* display = realpath(path, abspath) ? abspath : path;
    logger::line("[params] loaded from %s", display);
}

void ParamStore::save(const char* path) const
{
    FILE* fp = std::fopen(path, "w");
    if (!fp) {
        logger::line("[params] warning: cannot write to %s", path);
        return;
    }
    for (uint16_t i = 0; i < COUNT; ++i)
        std::fprintf(fp, "%s=%.6g\n", params_[i].name, params_[i].value);
    std::fclose(fp);
    char abspath[PATH_MAX];
    const char* display = realpath(path, abspath) ? abspath : path;
    logger::line("[params] saved to %s", display);
}
