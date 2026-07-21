#!/bin/python3

import os
import xml.etree.ElementTree as ET
from collections import defaultdict, deque
import copy
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

import json
import yaml

def main(root_dir, yaml_file=None):

    packages = find_packages(root_dir)
    ts = graphlib.TopologicalSorter(packages)
    ordered_list = [*tuple(ts.static_order())]

    if not yaml_file:
        print(json.dumps(ordered_list))
        return

    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)

    output = {
        "group_1": [],
        "group_2": [],
        "group_3": [],
        "group_last": []
    }

    for repo in ordered_list:
        repo_config = config.get(repo, {})
        group_id = repo_config.get('group', None)
        
        if group_id == 1:
            output["group_1"].append(repo)
        elif group_id == 2:
            output["group_2"].append(repo)
        elif group_id == 3:
            output["group_3"].append(repo)
        else:
            output["group_last"].append(repo)

    print(json.dumps(output))

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python build_order.py <root_directory> [yaml_file]")
    else:
        root_dir = sys.argv[1]
        yaml_file = sys.argv[2] if len(sys.argv) > 2 else None
        main(root_dir, yaml_file)
