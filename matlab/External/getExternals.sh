#!/bin/bash

# DERIVESTsuite:
# https://de.mathworks.com/matlabcentral/fileexchange/13490-adaptive-robust-numerical-differentiation
wget -O DERIVESTsuite.zip https://de.mathworks.com/matlabcentral/mlc-downloads/downloads/submissions/13490/versions/7/download/zip
unzip DERIVESTsuite.zip "DERIVESTsuite/*" -d "./"
unzip DERIVESTsuite.zip "license.txt" -d "./DERIVESTsuite"

# yaml matlab e.g. for interface to libRSF or to rpg_trajectory_evaluation 
git clone https://github.com/ewiger/yamlmatlab.git

