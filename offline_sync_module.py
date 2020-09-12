#!/usr/bin/env python
# -*- coding: utf-8 -*-

def sync_robot_state(robot_states, start_time, i_prev, current_time, dt):
    try:
        i = i_prev + 1
        time_robot = robot_states[i_prev].timestamp - start_time
        while ((current_time -
                (robot_states[i].timestamp - start_time)) >= dt):
            i += 1
            time_robot = robot_states[i].timestamp - start_time
        return i, time_robot
    except IndexError:
        print("Not enogh robot states!\n")
        raise Exception("Not enogh robot states!")
