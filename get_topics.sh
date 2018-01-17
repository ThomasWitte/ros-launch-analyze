#!/bin/bash

function path {
    package=$1
    node=$2
    
    if [[ -n $CMAKE_PREFIX_PATH ]]; then
      IFS=$'\n'
      catkin_package_libexec_dirs=(`catkin_find --without-underlays --libexec --share "$package" 2> /dev/null`)
      unset IFS
      debug "Looking in catkin libexec dirs: $catkin_package_libexec_dirs"
    fi
    pkgdir=`rospack find "$package"`
    debug "Looking in rospack dir: $pkgdir"
    if [[ ${#catkin_package_libexec_dirs[@]} -eq 0 && -z $pkgdir ]]; then
      exit 2
    fi
    if [[ ! $node == */* ]]; then
      # The -perm /mode usage is not available in find on the Mac
      #exepathlist=(`find $pkgdir -name $2 -type f -perm /u+x,g+x,o+x`)
      # -L: #3475
      if [[ `uname` == Darwin ]]; then
        _perm="+111"
      else
        _perm="/111"
      fi
      debug "Searching for $node with permissions $_perm"
      exepathlist="`find -L "${catkin_package_libexec_dirs[@]}" "$pkgdir" -name "$node" -type f  -perm "$_perm" ! -regex ".*$pkgdir\/build\/.*" | uniq`"
      IFS=$'\n'
      exepathlist=($exepathlist)
      unset IFS
      unset _perm
      if [[ ${#exepathlist[@]} == 0 ]]; then
        echo "[rosrun] Couldn't find executable named $node below $pkgdir"
        nonexepathlist=(`find -H "$pkgdir" -name "$node"`)
        if [[ ${#nonexepathlist[@]} != 0 ]]; then
          echo "[rosrun] Found the following, but they're either not files,"
          echo "[rosrun] or not executable:"
          for p in "${nonexepathlist[@]}"; do
            echo "[rosrun]   ${p}"
          done
        fi
        exit 3

      elif [[ ${#exepathlist[@]} -gt 1 ]]; then
        echo "[rosrun] You have chosen a non-unique executable, please pick one of the following:"
        select opt in "${exepathlist[@]}"; do
          exepath="$opt"
          break
        done
      else
        exepath="${exepathlist[0]}"
      fi
    else
      absname="$pkgdir/$node"
      debug "Path given. Looing for $absname"
      if [[ ! -f $absname || ! -x $absname ]]; then
        echo "[rosrun] Couldn't find executable named $absname"
        exit 3
      fi
      exepath="$absname"
    fi
}

# try to retrieve infomation on topics from the source
#strings "$1" | grep "\.cpp" | sort | uniq | xargs rosrun ros_launch_lint topic_inspection
#
#if [ -n "`file -b "$1" | grep Python`" ]
#then
#    rosrun ros_launch_lint topic_inspection "$1"
#fi

echo "LD_LIBRARY_PATH=" $LD_LIBRARY_PATH 1>&2

# execute the node and print topics
ROSCPP_PRELOAD=$(locate libroscpp_preload.so)
(timeout -s 'TERM' 4 roscore &> /dev/null )&

if [ -n "`file -b "$1" | grep Python`" ]
then
    path $1 $2
    shift
    shift
    C=''
    for i in "$@"; do 
        i="${i//\\/\\\\}"
        C="$C \"${i//\"/\\\"}\""
    done
    timeout 3 bash -c "rosrun ros_launch_lint rospy_preload.py $exepath 2>&1" | tee >(grep -v '<<' 1>&2) | grep '<<' | sort | uniq
else
    C=''
    for i in "$@"; do 
        i="${i//\\/\\\\}"
        C="$C \"${i//\"/\\\"}\""
    done
    timeout 3 bash -c "LD_PRELOAD=$ROSCPP_PRELOAD rosrun $C 2>&1" | tee >(grep -v '<<' 1>&2)  | grep '<<' | sort | uniq
fi
