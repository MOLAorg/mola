#!/bin/env python3

import os


def list_directories(path):
    # List all directories under the given path
    directories = [d for d in os.listdir(
        path) if os.path.isdir(os.path.join(path, d))]
    return directories


def extract_plain_name(directory):
    # Extract the plain name without the full path
    return os.path.basename(directory)


def process_directories(path, items_to_remove, pattern_text):
    directories = list_directories(path)

    sorted_directories = sorted(directories, key=extract_plain_name)

    for plain_name in sorted_directories:

        # Check if the plain name passes the filter
        if plain_name not in items_to_remove:
            # Print the pattern text with the name
            output_text = pattern_text.format(name=plain_name)
            print(output_text)


path_to_search = '..'
items_to_remove = ['scripts', 'docs',
                   '.vscode', '.github', '.circleci', '.git']
pattern_text = "| {name} | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__{name}__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__{name}__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__{name}__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__{name}__ubuntu_jammy_amd64__binary/)| [![Build Status](https://build.ros2.org/job/Rbin_uN64__{name}__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__{name}__ubuntu_noble_amd64__binary/) |"

process_directories(path_to_search, items_to_remove, pattern_text)
