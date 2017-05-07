# puma-rtclient
Simple remote client for the Puma. Works with the puma-ctrl control server.

### Build instructions

```shell
# build math lib
cd include
mkdir -p build && cd build
cmake .. && make

cd ../../Robot
mkdir -p build && cd build
cmake .. && make
```

### To run
1. Change LAN IPV4 to manual settings and pick an address such as 192.168.2.100. Set subnet mask to 255.255.255.0.
2. Telnet to remote server (192.168.2.2), log in.
3. Then run
```shell
cd puma
./run
```
4. Only if it says 'waiting for remote client', run client executable on local machine.

