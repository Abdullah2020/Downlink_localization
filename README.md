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

## Usage examples

The module includes the following examples:

* `complete-network-example`
* `lora-downlink-benchmark`
* `lora-downlink-localization`

Examples can be run via the `./ns3 run scratch/lora-downlink-localization` command (refer to `./ns3 run --help` for more options).
