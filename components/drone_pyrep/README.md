# dronePyRep

This is a pseudo-component, a Python adapter, that used the PyRep module to start CoppeliaSim and access the simulated scene. This method is much faster that the remote communcation through middlewares.
The main limitation is that it cannot be regenerated using RoboComp's code generator, because it is Qt-free. The reason is the incompatibility between the CoppeliaSim own version of Qt and the version usually installed in the machine.

Please follow these steps:

- Install https://github.com/stepjam/PyRep

. If you have a joystick, start ~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish
. Check the config file to set the ranges of the axis.


## Configuration parameters
As any other component, *dronePyRep* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for implements interfaces
CameraRGBDSimple.Endpoints=tcp -p 10096
Laser.Endpoints=tcp -p 10003
OmniRobot.Endpoints=tcp -p 10004
CoppeliaUtils.Endpoints = tcp -p 10666

# Endpoints for subscriptions interfaces
JoystickAdapterTopic.Endpoints=tcp -p 13100

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
Ice.MessageSizeMax=20004800
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <dronePyRep's path> 
```

After editing the new config file we can run the component:

```
./run.sh
```
