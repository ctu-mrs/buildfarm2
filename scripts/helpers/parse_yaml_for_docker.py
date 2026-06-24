#!/usr/bin/python3

import yaml
import sys
import json

def main():

    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        return

    with open(file_path, 'r') as file:

        try:
            data = yaml.safe_load(file)
        except yaml.YAMLError as exc:
            print(exc)

        packages = []
        for package in data:

            properties = data[package]
            docker = properties.get('generate_docker_image', False)

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
