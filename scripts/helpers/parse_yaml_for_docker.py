#!/usr/bin/python3

import yaml
import sys
import json

def main():

    if len(sys.argv) < 2:
        return

    file_path = sys.argv[1]
    variant = 'stable'

    for i, v in enumerate(sys.argv):
        if v == '--variant':
            variant = sys.argv[i + 1]
            break

    with open(file_path, 'r') as file:

        try:
            data = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)

        packages = []
        for package in data:

            properties = data[package]
            docker = properties.get('generate_docker_image', False)

            # if all branches are identical the Docker output will be the same no matter which variant builds it so build it only on stable
            git_refs = properties.get('git_refs', {})
            is_single_version = len(set(git_refs.values())) == 1

            if not docker or is_single_version and variant != 'stable':
                continue

            if isinstance(docker, dict):
                for image_name, config in docker.items():
                    if isinstance(config, dict):
                        folder = config.get('folder', './docker')
                        base_image = config.get('base_image', 'ctumrs/ros_jazzy:latest')
                        tag = config.get('tag', '')
                    else:
                        folder = config
                        base_image = 'ctumrs/ros_jazzy:latest'
                        tag = ''

                    packages.append({
                        'repo': package,
                        'folder': folder,
                        'image': image_name,
                        'base_image': base_image,
                        'tag': tag
                    })
            elif docker == True:
                packages.append({
                    'repo': package,
                    'folder': './docker',
                    'image': package,
                    'base_image': 'ctumrs/ros_jazzy:latest'
                })

        print(json.dumps(packages))

if __name__ == '__main__':
    main()
