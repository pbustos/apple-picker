# drone_controller
Intro to component here


## Configuration parameters
As any other component, *drone_controller* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
# Endpoints for subscriptions interfaces
CameraRGBDSimpleProxy = camerargbdsimple:tcp -h localhost -p 10096
CoppeliaUtilsProxy = coppeliautils:tcp -h localhost -p 10666

# This property is used by the clients to connect to IceStorm.
TopicManager.Proxy=IceStorm/TopicManager:default -p 9999

Ice.MessageSizeMax=20004800
Ice.Warn.Connections=0
Ice.Trace.Network=0
Ice.Trace.Protocol=0
```

## Starting the component
Run the script
```
./run.sh
```

