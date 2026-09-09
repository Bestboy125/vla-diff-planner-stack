# Ubuntu ground-station deployment

This deployment serves the production React frontend from the FastAPI backend in
an isolated Docker container. It uses host networking so the backend can receive
onboard observations on TCP 8080 and reach the onboard bridge directly.

The target machine must already contain the local `ground-image:latest` base image.
Private values live only in `config.env`, which is ignored by Git and installed with
mode 0600. The default configuration always sets `CONTROL_OUTPUT_ENABLED=false`.

Install or update:

```bash
cd ~/vla-ground-station/ground_station
./linux/deploy.sh
```

One-command start after installation:

```bash
~/vla-ground-station/ground_station/linux/start.sh
```

Guarded Live start reads the operator token and confirmation phrase from
`~/Desktop/vla_control_credentials.txt`. The file must contain exactly two lines,
be owned by the current user, and have mode 0600. Before replacing a locked
container, the script verifies the credential values, onboard bridge TCP port,
and three NTP clock samples:

```bash
~/vla-ground-station/ground_station/linux/start-live.sh
```

The backend still requires both values on every Live API request. Starting Live
mode does not create a mission, arm, take off, or publish a trajectory.

Synchronize the laptop clock against the onboard NTP server and verify three
samples within the configured 300 ms limit:

```bash
~/vla-ground-station/ground_station/linux/sync-time-with-onboard.sh
```

The script checks the server before changing `systemd-timesyncd`, so an offline
onboard computer leaves the existing time configuration untouched. The deployed
Ubuntu desktop also has a `同步机载电脑时间` launcher for the same command.

Stop:

```bash
~/vla-ground-station/ground_station/linux/stop.sh
```

The local UI is `http://127.0.0.1:8080`; the onboard-LAN UI is
`http://192.168.5.4:8080`. Starting the container does not unlock the aircraft,
start the flight stack, or send a task.
