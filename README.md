# 📡 Trial on LoRaWAN Downlink Localization: A Gateway Beaconing Approach

This NS-3 simulation implements a LoRaWAN localization mechanism using gateway beaconing. Gateways periodically transmit downlink beacons, and End Devices (EDs) estimate their position using RSSI-based trilateration. Gaussian noise is added to RSSI measurements to emulate real-world variability.


### Compilation

Ns-3 adopts a development-oriented philosophy. Before you can run anything, you'll need to compile the ns-3 code. You have two options:

1. **Compile ns-3 as a whole:** Make all simulation modules available by configuring and building as follows (ensure you are in the `ns-3-dev` folder!):

   ```bash
   ./ns3 configure --enable-tests --enable-examples &&
   ./ns3 build
   ```

2. **Focus exclusively on the lorawan module:** To expedite the compilation process, as it can take more than 30/40 minutes on slow hardware, change the configuration as follows:

   ```bash
   ./ns3 clean &&
   ./ns3 configure --enable-tests --enable-examples --enable-modules lorawan &&
   ./ns3 build
   ```

   The first line ensures you start from a clean build state.

Finally, ensure tests run smoothly with:

```bash
./test.py
```

If the script reports that all tests passed you are good to go.

## Simulation Configuration

In our simulation, `scratch/` folder includes the following example files:

* `complete-network-example`
* `lora-downlink-benchmark`
* `lora-downlink-localization`

The above mentioned files, e.g, `lora-downlink-localization` script allows you to configure various simulation parameters using command-line arguments. Below are the available options:

| Argument              | Description                                                                 | Default Value |
|-----------------------|-----------------------------------------------------------------------------|---------------|
| `--nDevices`          | Number of end devices to include in the simulation.                         | `100`         |
| `--radius`            | The radius (in meters) of the area to simulate.                             | `1000`        |
| `--realisticChannel`  | Whether to use a more realistic channel model (e.g., buildings, shadowing). | `true`        |
| `--simulationTime`    | The time (in seconds) for which to simulate.                                | `600`         |
| `--appPeriod`         | The period (in seconds) for periodically transmitting applications.         | `600`         |
| `--print`             | Whether or not to print building information to a file.                     | `true`        |

---

## Running the Simulation

You can run the `lora-downlink-localization` example with custom parameters using the following command:

```bash
./ns3 --run "scratch/lora-downlink-localization --nDevices=200 --radius=500 --realisticChannel=true --simulationTime=1200 --appPeriod=300 --print=true"
```




