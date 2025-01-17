#!/usr/bin/python3
# -*- coding: utf-8 -*-
# generated from catkin/cmake/template/script.py.in
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
<<<<<<< HEAD
python_script = '/home/projet_rob_mobile/src/pck_launch_all/src/mark_robot_on_map.py'
=======
python_script = '/home/catkin_ws/src/pck_launch_all/src/mark_robot_on_map.py'
>>>>>>> b157bc9fbefe0d5e0a2bb8ea58a4cca1ac17fbfd
with open(python_script, 'r') as fh:
    context = {
        '__builtins__': __builtins__,
        '__doc__': None,
        '__file__': python_script,
        '__name__': __name__,
        '__package__': None,
    }
    exec(compile(fh.read(), python_script, 'exec'), context)
