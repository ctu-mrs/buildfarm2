#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
import re
import json
import sys
import subprocess

def clean_xml(file_path):
    """ Reads an XML file, removes invalid comments, and returns a clean string. """
    with open(file_path, "r", encoding="utf-8") as f:
        xml_content = f.read()
    
    # Remove illegal comments (e.g., those containing "--" incorrectly)
    xml_content = re.sub(r'<!\s*--[^>]*--\s*>', '', xml_content, flags=re.DOTALL)
    
    return xml_content.strip()

def sanitize_git_remote(remote_url):
    """ Normalizes Git remote URLs into standard HTTPS format for Nix fetchGit. """
    if not remote_url:
        return None

    if remote_url.startswith("git@"):
        return remote_url.replace(":", "/", 1).replace("git@", "https://", 1)

    if remote_url.startswith("ssh://"):
        return remote_url.replace("ssh://git@", "https://", 1).replace("ssh://", "https://", 1)

    return remote_url

def get_git_info(path):
    """ Runs git commands to extract remote URL, branch, commit hash, and the true Git root. """
    try:
        remote_cmd = ["git", "-C", path, "config", "--get", "remote.origin.url"]
        raw_remote = subprocess.check_output(remote_cmd, stderr=subprocess.DEVNULL).decode('utf-8').strip()

        branch_cmd = ["git", "-C", path, "rev-parse", "--abbrev-ref", "HEAD"]
        branch = subprocess.check_output(branch_cmd, stderr=subprocess.DEVNULL).decode('utf-8').strip()

        # Extract the exact commit hash for pure reproducible builds
        rev_cmd = ["git", "-C", path, "rev-parse", "HEAD"]
        git_rev = subprocess.check_output(rev_cmd, stderr=subprocess.DEVNULL).decode('utf-8').strip()

        # Ask Git where the actual root of this specific repository is
        root_cmd = ["git", "-C", path, "rev-parse", "--show-toplevel"]
        git_root = subprocess.check_output(root_cmd, stderr=subprocess.DEVNULL).decode('utf-8').strip()

        clean_remote = sanitize_git_remote(raw_remote)

        return clean_remote, branch, git_rev, git_root
    except subprocess.CalledProcessError:
        return None, None, None, None

def generate_nix_json(root_dir):
    packages_data = {}

    for subdir, dirs, files in os.walk(root_dir):
        if "COLCON_IGNORE" in files:
            dirs[:] = [] 
            continue

        if "package.xml" in files:
            package_path = os.path.join(subdir, "package.xml")
            
            try:
                xml_content = clean_xml(package_path)           
                root = ET.fromstring(xml_content)
            except Exception as e:
                print(f"Warning: Failed to parse {package_path}: {e}", file=sys.stderr)
                continue

            name_elem = root.find("name")
            if name_elem is None:
                continue
            package_name = name_elem.text.strip()
            
            # Extract the package version
            version_elem = root.find("version")
            package_version = version_elem.text.strip() if version_elem is not None and version_elem.text else "unknown"

            # Extract Git Information
            git_remote, git_branch, git_rev, git_root = get_git_info(subdir)
            
            # Calculate path relative to the Git repository root to fix unpackPhase
            if git_root:
                rel_path = os.path.relpath(subdir, git_root)
            else:
                rel_path = os.path.relpath(subdir, root_dir)
                
            if rel_path == ".":
                rel_path = "" # If it's at the root of the repo, leave it blank
            
            # Extract dependencies strictly by their ROS types
            buildtool_depends = set()
            build_depends = set()
            build_export_depends = set()
            exec_depends = set()
            test_depends = set()

            for dep in root.findall("buildtool_depend"):
                if dep.text: buildtool_depends.add(dep.text.strip())

            # <depend> applies to build, export, and execution
            for dep in root.findall("depend"):
                if dep.text:
                    build_depends.add(dep.text.strip())
                    build_export_depends.add(dep.text.strip())
                    exec_depends.add(dep.text.strip())

            for dep in root.findall("build_depend"):
                if dep.text: build_depends.add(dep.text.strip())

            # Extract build_export_depend (and the inverted name to be safe)
            for dep_type in ["build_export_depend", "build_depend_export"]:
                for dep in root.findall(dep_type):
                    if dep.text: build_export_depends.add(dep.text.strip())

            for dep_type in ["exec_depend", "run_depend"]:
                for dep in root.findall(dep_type):
                    if dep.text: exec_depends.add(dep.text.strip())

            for dep in root.findall("test_depend"):
                if dep.text: test_depends.add(dep.text.strip())

            # Compile the JSON payload
            packages_data[package_name] = {
                "path": rel_path,
                "version": package_version,
                "git_remote": git_remote,
                "git_branch": git_branch,
                "git_rev": git_rev,
                "buildtool_depends": sorted(list(buildtool_depends)),
                "build_depends": sorted(list(build_depends)),
                "build_export_depends": sorted(list(build_export_depends)),
                "exec_depends": sorted(list(exec_depends)),
                "test_depends": sorted(list(test_depends))
            }

    return packages_data

def main(root_dir, output_file):
    packages_data = generate_nix_json(root_dir)
    
    # Ensure the target directory exists
    output_dir = os.path.dirname(os.path.abspath(output_file))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        
    with open(output_file, "w") as f:
        json.dump(packages_data, f, indent=4)
        
    print(f"✅ Successfully generated {output_file} containing {len(packages_data)} ROS packages.")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 extract_deps_json.py <root_directory> <output_json_path>")
        sys.exit(1)
    else:
        main(sys.argv[1], sys.argv[2])
