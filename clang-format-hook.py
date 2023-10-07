import os
import subprocess
import sys

# Path to the clang-format executable
CLANG_FORMAT_PATH = '/usr/bin/clang-format'

# Function to format a file using clang-format
def format_file(filename):
  subprocess.run([CLANG_FORMAT_PATH, '-i', filename], check=True)

# Function to format all files in the Components directory of a Git repository
def format_components():
  # Get the list of files in the Components directory
  files = subprocess.check_output(['git', 'ls-files', 'Components']).decode().splitlines()

  # Format each file
  for filename in files:
    format_file(filename)

# Main function
def main():
  # Format the files in the Components directory
  format_components()
  subprocess.run(['git', 'add', '-u'], check=True)
  subprocess.run(['git', 'commit', '-m', 'Format code'], check=True)
  subprocess.run(['git', 'push'], check=True)

if __name__ == '__main__':
  main()