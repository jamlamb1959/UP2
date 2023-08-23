# UP2
So this will be the start of a program that will provide a "hotspot" SSID: GW that can be used to provide an uplink to mqtt servers that are available on the
internet.

Port 1959 is a UDP listener and will attempt to publish any message it receives.

# Build
This packages uses platformio(pio).
Using github.com build runner that assumes the runner is running in the local
environment(look at makefile.yml for details.)
