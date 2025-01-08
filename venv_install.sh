#!/bin/bash

# To make this script executable, run:
# chmod +x filename.sh

# To execute this script, run:
# ./filename.sh

# Create a virtual environment named coord_trans
python3 -m venv coord_trans

# Activate the virtual environment
source coord_trans/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

