# OpenCyc 4.0 

Opencyc was discontinued as of 2017 by its publisher Cycorp, Inc. This package allows the local use of the last available version of opencyc, opencyc 4.0.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

A working KnowRob installation is required in order to use the launch-file.


### Installing

As a sub-module of KnowRob AddOns this package does not require any specific instructions for use.

You can simply run the launch-file, using

```
roslaunch opencyc opencyc.launch
```

which will take care of the installation process. Note, that it will take longer when run for the first time, since it downloads the whole opencyc ontology.

To browse opencyc 4.0 after the launchfile was executed, fire up your local browser with this uri:

```
 http://localhost:3602/cgi-bin/cg?cb-start
```

Alternatively, the link is being deployed at the end of the launch progress and can be accessed directly from the terminal with a mouse click while holding down ctrl. 

## Deployment

Anyone can use this package to browse opencyc 4.0. Please take note, that running the launch-file will download the entire opencyc ontology (about 665MB) to your local drive.


## Built With

* [catkin](http://wiki.ros.org/catkin)


## Authors

* **IAI-Bremen** - *Initial work*
* **Lukas S. ** - *embedding opencyc 4.0 into the opencyc package of KnowRob*


## License

BSD.

## Troubleshooting

In case something went wrong during the installation, delete the folder /../opencyc-4.0/ and run /../../opencyc.launch one more time. 
