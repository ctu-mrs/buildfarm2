#!/bin/python3

import json
import os
import xml.etree.ElementTree as ET
import re
import graphlib

def clean_xml(file_path):

    """ Reads an XML file, removes invalid comments, and returns a clean string. """
    with open(file_path, "r", encoding="utf-8") as f:
        xml_content = f.read()

    # Remove illegal comments (e.g., those containing "--" incorrectly)
    xml_content = re.sub(r'<!\s*--[^>]*--\s*>', '', xml_content, flags=re.DOTALL)
    
    return xml_content.strip()  # Ensure root remains on the first line

def find_packages(root_dir):

    repositories = {}

    ## | ----------- create a map of package->repository ---------- |

    # get list of repositories

    repository_list = []

    for root, dirs, files in os.walk(root_dir):

        for folder in dirs:
            repository_list.append(folder)

        break

    # create a map of package->repository

    package_repo_map = {}

    for repository in repository_list:

        for subdir, _, files in os.walk(root_dir + "/" + repository):

            if "package.xml" in files:

                if "COLCON_IGNORE" in files:
                    continue

                package_path = os.path.join(subdir, "package.xml")

                xml_content = clean_xml(package_path)

                root = ET.fromstring(xml_content)

                name_elem = root.find("name")

                package_name = name_elem.text.strip()

                package_repo_map[package_name] = repository

    # build the dependency map between the repositories

    for repository in repository_list:

        # print("walking over repository: {}".format(repository))

        pruned_dependencies = set()

        for subdir, _, files in os.walk(root_dir + "/" + repository):

            if "package.xml" in files:

                if "COLCON_IGNORE" in files:
                    continue

                package_path = os.path.join(subdir, "package.xml")

                xml_content = clean_xml(package_path)

                root = ET.fromstring(xml_content)

                name_elem = root.find("name")

                if name_elem is None:
                    continue

                package_name = name_elem.text.strip()

                dependencies = [dep.text.strip() for dep in root.findall("depend") if dep.text]
                dependencies = dependencies + [dep.text.strip() for dep in root.findall("build_depend") if dep.text]
                dependencies = dependencies + [dep.text.strip() for dep in root.findall("exec_depend") if dep.text]
                dependencies = dependencies + [dep.text.strip() for dep in root.findall("test_depend") if dep.text]
                dependencies = dependencies + [dep.text.strip() for dep in root.findall("run_depend") if dep.text]

                for dependency in dependencies:

                    # the dependency is not package within our repositories
                    if dependency not in package_repo_map:
                        # print("Dependency {} of the package {} is not within the package map".format(dependency, package_name))
                        continue

                    # the dependency is satisfied by a package from this repository
                    if package_repo_map[dependency] == repository:
                        # print("Dependency {} of the package {} from repository {} is located within the repository".format(dependency, package_name, repository))
                        continue

                    # print("Appending {}".format(package_repo_map[dependency]))

                    pruned_dependencies.add(package_repo_map[dependency])

        # print("pruned_dependencies for {}: {}".format(repository, pruned_dependencies))

        repositories[repository] = pruned_dependencies

    # print("repositories: {}".format(repositories))

    return repositories

def compute_levels(packages, ordered_list):

    """ Longest-path depth of each repo in the dependency DAG: 0 = no in-graph deps,
    N = depends (transitively) on something at depth N-1. ordered_list is a valid
    topological order, so every dependency of a repo is already resolved by the
    time that repo is reached. """

    depth = {}

    for repo in ordered_list:
        deps = packages.get(repo, set())
        depth[repo] = 0 if not deps else 1 + max(depth[dep] for dep in deps)

    return depth

def main(root_dir, grouped=False):

    packages = find_packages(root_dir)
    ts = graphlib.TopologicalSorter(packages)
    ordered_list = [*tuple(ts.static_order())]

    if not grouped:
        print(json.dumps(ordered_list))
        return

    depth = compute_levels(packages, ordered_list)

    output = {
        "group_1": [],
        "group_2": [],
        "group_3": [],
        "group_last": []
    }

    # anything deeper falls back to the serial tail (group_last)
    group_depth = {
        0: "group_1",
        1: "group_2",
        2: "group_3"
    }

    for repo in ordered_list:
        output[group_depth.get(depth[repo], "group_last")].append(repo)

    print(json.dumps(output))

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python build_order.py <root_directory> [grouped]")
    else:
        grouped = len(sys.argv) > 2 and sys.argv[2].lower() == "true"
        main(sys.argv[1], grouped)
