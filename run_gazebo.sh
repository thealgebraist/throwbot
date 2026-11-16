#!/usr/bin/env zsh
set -e
export GZ_IP=127.0.0.1
echo "Starting Gazebo Sim Server (-s) and GUI (-g) for macOS..."
echo "Drive forward: GZ_IP=127.0.0.1 gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.4}, angular: {z: 0.0}'"
echo "Skid-steer:    GZ_IP=127.0.0.1 gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 1.5}'"
gz sim -s -r /Users/anders/.gemini/antigravity/scratch/outrunner_buggy_sim/buggy_test_world.sdf &
SERVER_PID=$!
trap "kill $SERVER_PID 2>/dev/null || true" EXIT
sleep 2
gz sim -g
