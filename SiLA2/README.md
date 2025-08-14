# SiLA2 features and implementations

## Useful commands:

### Install FTIR server

Navigate to `SiLA2` folder and run `pip install -e .`
It is advised to use a python virtual environment for just this server installed the `requirements.txt`

### Start FTIR server
IP address 0.0.0.0 listens on all addresses improving auto discovery. Port might have to be changed if multiple servers run on the same computer.

`python -m ftir_sila2_package --ip-address 0.0.0.0 --port [port=50052] [--quiet|--verbose|--debug]`

More options at: https://sila2.gitlab.io/sila_python/content/server/start_server.html

## TODO: 
IMPROVE THIS markdown file
Add recovery to check_home().
Better progress tracking for observable commands.