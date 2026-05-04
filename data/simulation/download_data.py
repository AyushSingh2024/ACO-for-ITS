"""
Download SUMO simulation data from Google Drive.
Usage: python data/simulation/download_data.py
"""
import os
import gdown

url = "https://drive.google.com/drive/folders/13Aykv4V9Rd1UEsP-ixmJH9NTULcAAC4W"
output_dir = os.path.dirname(os.path.abspath(__file__))
gdown.download_folder(url, output=output_dir, quiet=False)