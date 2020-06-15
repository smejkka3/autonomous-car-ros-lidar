#!/bin/bash
if ! [ -x "$(command -v tmux)" ]; then
  echo 'Error: tmux is not installed.' >&2
  exit 1
fi
[[ -z "$1" ]] && grep="" || grep="| grep $1"
tmux kill-session -t rosc &> /dev/null
tmux new -s rosc -d
tmux split-window -d -t rosc
tmux split-window -d -t rosc

#tmux select-layout even-vertical
tmux resize-pane -t rosc.0 -U 40
tmux resize-pane -t rosc.1 -U 20
tmux resize-pane -t rosc.2 -D 30

tmux send-keys -t rosc.0 "morse run fourwd morse/fourwd/default.py" enter
tmux send-keys -t rosc.1 "roslaunch odometry_agent odometry_agent.launch is_simulation:=true" enter
tmux send-keys -t rosc.2 "roslaunch pose_follower navigation_stack_fourwd.launch is_simulation:=true --wait" enter

tmux a
# Exit with CTRL+B followed by & and cofirm with y
