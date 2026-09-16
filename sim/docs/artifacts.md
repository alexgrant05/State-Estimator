# Run Artifacts

`digital_twin run` creates an output directory containing:

| Artifact | Purpose |
| --- | --- |
| `events.ndjson` | Complete timestamped logical event stream |
| `bno085_shtp.bin` | BNO085 replay for sensor-controller development |
| `adxl375_acquisitions.bin` | ADXL375 auxiliary SPI replay |
| `bmp581_acquisitions.bin` | BMP581 auxiliary SPI replay |
| `zed_f9p_uart.bin` | ZED-F9P UART replay |
| `states.csv` | Published state epochs, arrival epochs, state, biases, and covariance diagonal |
| `validation.json` | Machine-readable metrics and gates |
| `validation.md` | Human-readable validation summary |
| `errors.png` | Position, velocity, and attitude error plots |
| `manifest.json` | Reproducibility and artifact metadata |

Manifest schema version 3 records the seed, configuration hash, source revision,
dependency versions, active protocol versions and rates, expected high-dynamic
intervals, calibration status, artifact sizes and SHA-256 hashes, and overall
result. Use a new output directory for each run because the CLI does not
overwrite existing data.
