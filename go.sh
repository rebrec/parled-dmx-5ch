#!/bin/sh

# Used to build the whole package and upload it using PickitII
make clean && make all && make write
