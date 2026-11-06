#!/usr/bin/env bash
# Source before creating any ROS participant, including the camera compressor.
_XNAVDP_DDS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Use UDP for local data. The XML profile adds the server TCP transport.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export XNAVDP_ROS_DOMAIN_ID="${XNAVDP_ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID="$XNAVDP_ROS_DOMAIN_ID"
export FASTRTPS_DEFAULT_PROFILES_FILE="${XNAVDP_DDS_PROFILE:-$_XNAVDP_DDS_DIR/client_dds.xml}"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset CYCLONEDDS_URI
