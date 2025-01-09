#!/bin/bash

# Create a virtual environment named coord_trans
python3 -m venv coord_trans

# Activate the virtual environment
source coord_trans/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

