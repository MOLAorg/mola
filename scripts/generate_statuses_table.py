#!/bin/env python3

import os


script_dir = os.path.dirname(os.path.abspath(__file__))


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


path_to_search = os.path.join(script_dir, '..')
items_to_remove = ['scripts', 'docs', 'cla',
                   '.vscode', '.github', '.circleci', '.git', '.claude', '.cache']
pattern_text = \
    "| {name} | "\
    "[![Build Status](https://build.ros2.org/job/Hbin_uJ64__{name}__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__{name}__ubuntu_jammy_amd64__binary/) " \
    "<br> " \
    "[![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__{name}__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__{name}__ubuntu_jammy_arm64__binary/) " \
    " | " \
    "[![Build Status](https://build.ros2.org/job/Jbin_uN64__{name}__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__{name}__ubuntu_noble_amd64__binary/) " \
    "<br> " \
    "[![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__{name}__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__{name}__ubuntu_noble_arm64__binary/) " \
    " | " \
    "[![Build Status](https://build.ros2.org/job/Lbin_uR64__{name}__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Lbin_uR64__{name}__ubuntu_resolute_amd64__binary/) " \
    "<br> " \
    "[![Build Status](https://build.ros2.org/job/Lbin_armv8_uRv8__{name}__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Lbin_armv8_uRv8__{name}__ubuntu_resolute_arm64__binary/) " \
    " | " \
    "[![Build Status](https://build.ros2.org/job/Rbin_uR64__{name}__ubuntu_resolute_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uR64__{name}__ubuntu_resolute_amd64__binary/) " \
    "<br> " \
    "[![Build Status](https://build.ros2.org/job/Rbin_unv8_uRv8__{name}__ubuntu_resolute_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uRv8__{name}__ubuntu_resolute_arm64__binary/) " \
    " | "

process_directories(path_to_search, items_to_remove, pattern_text)
