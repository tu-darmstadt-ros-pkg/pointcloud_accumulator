#!/bin/bash

if [ "$(type -t add_rosrs_setup_env)" == "function" ]; then
  add_rosrs_setup_env HECTOR_USE_POINTCLOUD_ACCUMULATOR "true,false" "Can turn the pointcloud_accumulator node off to save computational resources." 
fi
