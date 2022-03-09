


### Libraries / Dependencies

ArduinoJSON
elapsedMillis
BluetoothSerial
Adafruit_PWMServoDriver


## API

Joint angles are always [base, rotate, finger_a, finger_b]

### Get Joints
Request:
```
{
    "uuid": "<uuid str>",
    "request": "joint_get"
}
```

Response:
```
{
    "uuid": "<uuid str>",
    "angles": [<integer>]
}
```

### Set Joints
Request:
```
{
    "uuid": "<uuid str>".
    "request": "joint_set",
    "angles": [<integer>]  
}
```

Response:
```
{
    "uuid": "<uuid str>",
    "angles": [<integer>] // New angles
}
```

### Push Update
Request:
```
{
    "uuid": "<uuid str>",
    "request": "push_update"
}
```

Response:
```
{
    "uuid": "<uuid str>",
    "sensors": {
        "names": ["<name>", ...],
        "values": [<integer / obj>, ...],
    },
    "angles": [<integer>, ...]
}
```

### Get Metadata
Request:
```
{
    "uuid": "<uuid str>",
    "request": "metadata"
}
```

Response:
```
{
    "uuid": "<uuid str>",
    "chip_id": "<chip id str>",
    "software_id": "<software version>"
}
```


### Update Msg (pushed periodically by microcontroller)
Message:
```
{
    "uuid": "update",
    "sensors": {
        "names": ["<name>", ...],
        "values": [<integer / obj>, ...],
    },
    "angles": [<integer>, ...]
}
```