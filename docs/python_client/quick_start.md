# Quick Start

## monoDrive Python Client Quick Start

The monoDrive Python Client is Open Source Software for connecting to and 
configuring the monoDrive Simulator and Scenario Editor. To get started, 
clone the client from the monoDrive repository:

```bash
$ git clone git@github.com:monoDriveIO/simulator-python-client.git 
```

### Installation

The monoDrive Python Client supports installation from git or locally with the 
cloned repository. It is recommended to install the client inside of a virtual 
environment such as `conda` (see instructions below), but this is not necessary. 

Install from Git repository:

```bash
$ pip install git+ssh://git@github.com/monoDriveIO/simulator-python-client.git#egg=monodrive
```

or for https:

```bash
$ pip install git+https://github.com/monoDriveIO/simulator-python-client.git#egg=monodrive
```

or, to install from a local repository:

```bash
$ cd /path/to/repo/simulator-python-client
$ pip install -e .
```

### Recommended Environment

It is recommended to use the Anaconda distribution for creating virtual 
environments when using the monoDrive Python Client:

* [Anaconda for Python 3.7](https://www.anaconda.com/distribution/#download-section)

After installation, the environment can be created using:

```bash
$ conda create --name monodrive-python
...
$ conda activate monodrive-python
```

Then follow the instructions above, to install the `monodrive` package with `pip`.

### Running Examples

The `examples` directory in the monoDrive Python Client repository contains 
several examples for connecting to a running instance of the 
[monoDrive Simulator](../../Simulator.md) or 
[Scenario Editor](../../Scenario_editor.md) and working with the simulation 
environment. To run an example, first ensure that the monoDrive Simulator is 
running or the monoDrive Scenario Editor is running and in "Play" mode.

```bash
$ conda activate monodrive-python
(monodrive-python) $ python examples/closed_loop.py
```
### Troubleshooting

- If the simulator is running on another machine, the host information in 
`examples/configuration/simulator.json` will need to be updated

```json
"simulator_ip": <IP OF SIMULATOR MACHINE>
```

- On running the script, the client is successfully connected if the simulator i
begins replaying the trajectory file.