#!/usr/bin/env python 


class global_var:
    display = '1'
    angle = '1'
    traffic = 'green'


def set_name(name):
    global_var.display = name


def set_angle(name):
    global_var.angle = name


def set_traffic(name):
    global_var.traffic = name


def get_name():
    return global_var.display


def get_angle():
    return global_var.angle


def get_traffic():
    return global_var.traffic
