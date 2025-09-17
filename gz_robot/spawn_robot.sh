#!/bin/bash
sleep 2
gz service -s /world/car_world/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --req 'sdf_filename: "./description/robot.sdf", name: "my_robot", pose: {position: {x: 0, y: 0, z: 0.0}}'
