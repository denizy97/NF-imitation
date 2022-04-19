# NF-imitation
This makes 2 Novint Falcon devices imitate each other.

They send each other their positions, then create a spring between their current position and the position they receive from the other.

With high spring constants and/or high latency the system becomes unstable. Be mindful. Don't sue me if your Novint Falcon breaks. 😛

## Build
I'm lazy so there's no makefile.
```
g++ -o imitation main.cpp -lnifalcon -lpthread
```

## Run
If you run it with no parameters, it prints info about the correct usage.
```
sudo ./imitation spring_constant damping ip_other port_self port_other nf_id
```

Example (with good values):
```
sudo ./imitation 500 4000 127.0.0.1 2323 2525 0
```
