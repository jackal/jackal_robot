JACKAL_MAG_CONFIG=$(catkin_find --etc --first-only jackal_base mag_config.yaml)
if [ -z "$JACKAL_MAG_CONFIG" ]; then
  JACKAL_MAG_CONFIG=$(catkin_find --share --first-only jackal_base config/mag_config_default.yaml)
fi

export JACKAL_MAG_CONFIG
