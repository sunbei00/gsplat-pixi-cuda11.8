#!/bin/bash
cd ~/code/gsplat
export PYTHONPATH=.  
pixi run -e gs $@
