# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

source /opt/ros/humble/setup.bash
echo "ROS2 Humble Active"
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/NeuROAM/install/setup.bash
echo "NeuROAM Workspace Active"

alias gst="git status"
alias ga="git add"
alias gp="git push"
alias gpull="git pull"
alias gd="git diff"
alias gc="git commit"
alias gb="git branch"
alias gpupdate="git add .; git commit -m 'small update'; git push;"
alias git-pull-neuroam="cd /home/neuroam/NeuROAM && git pull && git submodule update --init --recursive && cd -"

cp ~/NeuROAM/util/.bashrc ~/.bashrc
alias src="source ~/.bashrc"
alias src-update="cp ~/NeuROAM/util/.bashrc ~/.bashrc; source ~/.bashrc"

# Set ROS_DOMAIN_ID based on payload hostname
case "$(hostname)" in
  payload0) export ROS_DOMAIN_ID=0 ;;
  payload1) export ROS_DOMAIN_ID=1 ;;
  payload2) export ROS_DOMAIN_ID=2 ;;
  payload3) export ROS_DOMAIN_ID=3 ;;
  payload4) export ROS_DOMAIN_ID=4 ;;
  *) echo "⚠️  Unknown hostname '$(hostname)', ROS_DOMAIN_ID not set." ;;
esac

# Tmux
alias neuroam-tmux="tmux new -A -s neuroam"

# # set Zenoh params
# export ZENOH_CONFIG_OVERRIDE="transport/link/tx/queue/congestion_control/drop/wait_before_drop=1000000"

# ~/.bashrc  (or any setup script you source before running ROS 2)
ZENOH_ROUTER_CONFIG_URI="~/NeuROAM/util/zenoh_configs/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5"
ZENOH_SESSION_CONFIG_URI="~/NeuROAM/util/zenoh_configs/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

export ZENOH_CONFIG_OVERRIDE="transport/link/tx/queue/congestion_control/drop/wait_before_drop=1000000"
export ZENOH_FLOW_CONTROL=false
export RMW_ZENOH_FRAGMENT_SIZE=131072
export RMW_ZENOH_BATCH_SIZE=100000
export RMW_ZENOH_ROUTER_CHECK_ATTEMPTS=0

alias launch-experiments="ros2 launch ~/NeuROAM/launch/global_launch.py record_rosbag:=true"

# only run this if hostname is 'payload2'
if [ "$(hostname)" = "payload2" ]; then
    echo "Setting up network interface enP8p1s0 with static IP -- this should only be done on payload2"
    sudo ip addr add 169.254.1.2/16 dev enP8p1s0
fi

echo ""
echo "REMINDERS!!"
echo "    [UPDATE ENVIRONMENT] run the command 'src-update' once to make sure you have the most up-to-date .bashrc file."
echo "    [UPDATE MAIN REPO] make sure that ~/NeuROAM is on the 'main' branch and use 'git pull' to make sure code is up to date"
echo "    [UPDATE SUBMODULES] make sure that all submodules in NeuROAM are up to date 'cd ~/NeuROAM; git pull --recurse-submodules'"
echo "    [CHECK STORAGE] check if you have enough storage space on your system. If not, delete some files or move them to an external drive."
echo ""
df -h


#!!!!!! NOTE: MAKE SURE YOU CHANGE THE FILE IN ~/NeuROAM/util/.bashrc
#!!!!!! WE OVERWRITE THE FILE IN THE ROOT DIRECTORY
