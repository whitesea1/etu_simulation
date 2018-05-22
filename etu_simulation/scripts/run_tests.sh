#!/usr/bin/env bash
trap "exit" INT
while :
do
    roslaunch etu_simulation time_moves.launch
done
