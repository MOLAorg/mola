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


def process_directories(path, items_to_remove, make_row):
    directories = list_directories(path)

    sorted_directories = sorted(directories, key=extract_plain_name)

    for plain_name in sorted_directories:

        # Check if the plain name passes the filter
        if plain_name not in items_to_remove:
            # Print the pattern text with the name
            output_text = make_row(plain_name)
            print(output_text)


path_to_search = os.path.join(script_dir, '..')
items_to_remove = ['scripts', 'docs', 'cla', 'build', 'install', 'log',
                   '.vscode', '.github', '.circleci', '.git', '.claude', '.cache']

# (distro letter, ubuntu amd64 code, ubuntu arm64 code, ubuntu name,
#  RHEL (code, name) or None, Fedora (code, name) or None)
DISTROS = [
    ("H", "uJ64", "ujv8_uJv8", "jammy", None, None),
    ("J", "uN64", "unv8_uNv8", "noble", ("rhel_el964", "rhel_9"), None),
    ("K", "uN64", "unv8_uNv8", "noble", ("rhel_el964", "rhel_9"), None),
    ("L", "uR64", "armv8_uRv8", "resolute",
     ("rhel_el1064", "rhel_10"), ("fedora_fc4364", "fedora_43")),
    ("R", "uR64", "unv8_uRv8", "resolute",
     ("rhel_el1064", "rhel_10"), ("fedora_fc4464", "fedora_44")),
]


def badge(job):
    url = f"https://build.ros2.org/job/{job}/"
    return f"[![Build Status]({url}badge/icon)]({url})"


def make_cell(name, distro):
    letter, amd64, arm64, ubuntu, rhel, fedora = distro
    jobs = [
        f"{letter}bin_{amd64}__{name}__ubuntu_{ubuntu}_amd64__binary",
        f"{letter}bin_{arm64}__{name}__ubuntu_{ubuntu}_arm64__binary",
    ]
    if rhel:
        jobs.append(f"{letter}bin_{rhel[0]}__{name}__{rhel[1]}_x86_64__binary")
    if fedora:
        jobs.append(
            f"{letter}bin_{fedora[0]}__{name}__{fedora[1]}_x86_64__binary")
    return " <br> ".join(badge(j) for j in jobs)


def make_row(name):
    return f"| {name} | " + " | ".join(make_cell(name, d) for d in DISTROS) + " |"


process_directories(path_to_search, items_to_remove, make_row)
