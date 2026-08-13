# Configuration Reference

The reference file is `config/andromeda.toml`. Configuration is loaded into
frozen typed dataclasses. Unknown keys in typed sections raise a constructor
error, and missing required keys raise an input error.

## Simulation

| Key | Units | Purpose |
| --- | --- | --- |
| `clock_hz` | Hz | Integer hardware timebase frequency |
| `truth_rate_hz` | Hz | Truth stream rate, currently required to be 2000 |
| `pad_duration_s` | s | Stationary alignment interval before liftoff |
| `gravity_mps2` | m/s^2 | Gravity magnitude used by sensors and estimator |
| `seed` | none | Default deterministic run seed |

`clock_hz` must be an integer multiple of 2000.

## Launch

| Key | Units | Purpose |
| --- | --- | --- |
| `latitude_deg` | degrees | WGS84 launch latitude |
| `longitude_deg` | degrees | WGS84 launch longitude |
| `elevation_msl_m` | m | Launch elevation above mean sea level |
| `rail_length_m` | m | RocketPy launch rail length |
| `rail_inclination_deg` | degrees | Rail inclination using RocketPy convention |
| `rail_heading_deg` | degrees | Rail heading using RocketPy convention |

These values establish the ENU origin, ECEF conversion, initial yaw, and MSL to
launch-relative altitude conversion.

## Motor

| Key | Units | Purpose |
| --- | --- | --- |
| `burn_time_s` | s | Burn duration and boost-phase boundary |
| `total_impulse_ns` | N s | Total impulse used to form constant thrust |
| `dry_mass_kg` | kg | Motor dry mass |
| `dry_inertia_kgm2` | kg m^2 | Motor inertia `[I11, I22, I33]` |
| `nozzle_radius_m` | m | Nozzle exit radius |
| `throat_radius_m` | m | Nozzle throat radius |
| `grain_number` | none | Number of propellant grains |
| `grain_density_kgm3` | kg/m^3 | Propellant density |
| `grain_outer_radius_m` | m | Grain outer radius |
| `grain_initial_inner_radius_m` | m | Initial grain core radius |
| `grain_initial_height_m` | m | Grain height |
| `grain_separation_m` | m | Separation between grains |
| `grains_center_of_mass_position_m` | m | Grain center along motor axis |
| `center_of_dry_mass_position_m` | m | Dry motor center along motor axis |
| `nozzle_position_m` | m | Nozzle position along motor axis |

## Rocket

| Key | Units | Purpose |
| --- | --- | --- |
| `radius_m` | m | Body radius |
| `dry_mass_kg` | kg | Rocket mass without motor |
| `inertia_kgm2` | kg m^2 | Rocket inertia `[I11, I22, I33]` |
| `center_of_mass_without_motor_m` | m | Dry center along tail-to-nose axis |
| `power_off_drag` | none | Constant coast drag coefficient |
| `power_on_drag` | none | Constant powered drag coefficient |
| `motor_position_m` | m | Motor placement along body axis |
| `nose_length_m`, `nose_kind`, `nose_position_m` | mixed | Nose geometry and position |
| `fin_count` | none | Number of fins |
| `fin_root_chord_m`, `fin_tip_chord_m` | m | Fin chord dimensions |
| `fin_span_m`, `fin_position_m` | m | Fin span and body position |
| `fin_cant_angle_deg` | degrees | Fin cant angle |
| `tail_top_radius_m`, `tail_bottom_radius_m` | m | Tail radii |
| `tail_length_m`, `tail_position_m` | m | Tail geometry and position |

## ADIS16470

| Key | Units | Purpose |
| --- | --- | --- |
| `dec_rate` | register value | Output rate is `2000 / (dec_rate + 1)` Hz |
| `spi_clock_hz` | Hz | Burst SPI clock, at most 1 MHz |
| `temperature_c` | deg C | Constant reported sensor temperature |
| `accel_noise_rms_mg` | mg RMS | Per-axis internal-sample accelerometer noise |
| `gyro_noise_rms_dps` | deg/s RMS | Per-axis internal-sample gyro noise |
| `accel_bias_mps2` | m/s^2 | Initial three-axis sensor-frame bias |
| `gyro_bias_rps` | rad/s | Initial three-axis sensor-frame bias |
| `accel_bias_rw_mps2_sqrt_s` | m/s^2/sqrt(s) | Accelerometer bias random walk |
| `gyro_bias_rw_rps_sqrt_s` | rad/s/sqrt(s) | Gyro bias random walk |
| `accel_scale_error` | fraction | Per-axis multiplicative scale error |
| `gyro_scale_error` | fraction | Per-axis multiplicative scale error |
| `misalignment_rad` | rad | Small-angle cross-axis error vector |
| `sensor_to_body` | matrix | Proper orthonormal 3 by 3 mounting rotation |

`dec_rate` must be from 0 through 1999. The 2000 Hz internal stream is averaged
in groups of `dec_rate + 1`, not sample-skipped.

## ADXL375

| Key | Units | Purpose |
| --- | --- | --- |
| `enabled` | boolean | Generate ADXL events when true |
| `output_rate_hz` | Hz | Data-ready rate, at most 800 Hz |
| `spi_clock_hz` | Hz | Shared auxiliary SPI clock |
| `noise_density_mg_sqrt_hz` | mg/sqrt(Hz) | White noise density |
| `bias_mps2` | m/s^2 | Initial sensor-frame bias |
| `bias_rw_mps2_sqrt_s` | m/s^2/sqrt(s) | Bias random walk |
| `scale_error` | fraction | Per-axis multiplicative scale error |
| `misalignment_rad` | rad | Small-angle cross-axis error vector |
| `sensor_to_body` | matrix | Orthonormal 3 by 3 mounting rotation |

The output rate must be a positive integer divisor of `clock_hz`.

## BMP581

| Key | Units | Purpose |
| --- | --- | --- |
| `enabled` | boolean | Generate BMP events when true |
| `output_rate_hz` | Hz | Data-ready rate |
| `spi_clock_hz` | Hz | Shared auxiliary SPI clock |
| `pressure_oversampling` | none | Accepted values: 1 through 128 by powers of two |
| `temperature_oversampling` | none | Accepted values: 1, 2, 4, or 8 |
| `iir_coefficient` | none | Reserved configuration value |
| `pressure_noise_pa` | Pa RMS | Pressure white noise |
| `temperature_noise_c` | deg C RMS | Temperature white noise |
| `pressure_bias_pa` | Pa | Initial pressure bias |
| `pressure_bias_rw_pa_sqrt_s` | Pa/sqrt(s) | Pressure bias random walk |
| `transonic_error_peak_m` | m | Peak equivalent altitude disturbance |
| `transonic_mach_center` | Mach | Disturbance center |
| `transonic_mach_sigma` | Mach | Gaussian disturbance width |

Oversampling and IIR values are recorded but do not currently alter bandwidth
or conversion timing. Noise is configured directly with `pressure_noise_pa` and
`temperature_noise_c`.

## GNSS

| Key | Units | Purpose |
| --- | --- | --- |
| `enabled` | boolean | Generate solution and PPS events |
| `output_rate_hz` | Hz | Navigation solution rate |
| `pps_rate_hz` | Hz | PPS rate |
| `gps_week` | week | GPS week at simulation start |
| `start_tow_s` | s | GPS time of week at simulation start |
| `position_sigma_enu_m` | m RMS | East, north, up solution noise |
| `velocity_sigma_enu_mps` | m/s RMS | East, north, up velocity noise |
| `latency_mean_s` | s | Mean solution transport latency |
| `latency_jitter_s` | s RMS | Solution latency variation |
| `pps_jitter_ns` | ns RMS | PPS edge jitter and reported uncertainty |
| `clock_offset_ns` | ns | Local PPS edge offset |
| `clock_drift_ppm` | ppm | Local timebase drift applied to PPS edges |
| `antenna_lever_arm_body_m` | m | IMU origin to antenna in body axes |
| `position_bias_rw_m_sqrt_s` | m/sqrt(s) | Position bias random walk |
| `velocity_bias_rw_mps_sqrt_s` | m/s/sqrt(s) | Velocity bias random walk |
| `outage_entry_probability` | probability/sample | Enter correlated outage |
| `outage_recovery_probability` | probability/sample | Recover from outage |
| `high_acceleration_outage_g` | g | Optional acceleration-triggered outage, zero disables |

Rates must be positive integer divisors of `clock_hz`. Probabilities must be in
the inclusive range zero through one.

## Integration

| Key | Purpose |
| --- | --- |
| `history_duration_s` | Rewind/replay state history duration |
| `high_g_enter_fraction` | Fraction of ADIS 40 g range that enters ADXL mode |
| `high_g_exit_fraction` | Fraction of ADIS range below which exit can begin |
| `high_g_exit_hold_s` | Required continuous low-g time before returning to ADIS |
| `high_g_max_age_samples` | Maximum ADXL age in ADXL sample intervals |
| `overlap_nis_gate` | ADIS versus ADXL consistency threshold |
| `gnss_nis_gate` | Six-dimensional GNSS innovation threshold |
| `baro_nis_gate` | Barometer altitude innovation threshold |
| `baro_transonic_min_mach` | Lower Mach boundary for suppressed barometer updates |
| `baro_transonic_max_mach` | Upper Mach boundary for suppressed barometer updates |

History must exceed mean GNSS latency plus three latency standard deviations.

## Estimator

| Key | Units | Purpose |
| --- | --- | --- |
| `accel_bias_rw_mps2_sqrt_s` | m/s^2/sqrt(s) | Filter accelerometer-bias process noise |
| `gyro_bias_rw_rps_sqrt_s` | rad/s/sqrt(s) | Filter gyro-bias process noise |
| `initial_position_sigma_m` | m | Initial per-axis position uncertainty |
| `initial_velocity_sigma_mps` | m/s | Initial per-axis velocity uncertainty |
| `initial_attitude_sigma_deg` | degrees | Initial per-axis attitude uncertainty |
| `initial_accel_bias_sigma_mps2` | m/s^2 | Initial per-axis accelerometer-bias uncertainty |
| `initial_gyro_bias_sigma_rps` | rad/s | Initial per-axis gyro-bias uncertainty |

Initial standard deviations are squared to form the diagonal covariance.
