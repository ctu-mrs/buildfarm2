#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
export REPO_PATH=$MY_PATH/..
export ARCH=$(dpkg-architecture -qDEB_HOST_ARCH)
export LOCATION=/tmp/git

python3 - << 'PYEOF'
import os, sys, subprocess, yaml
from concurrent.futures import ThreadPoolExecutor

REPO_PATH = os.environ.get("REPO_PATH", ".")
PUSH_TOKEN = os.environ.get("PUSH_TOKEN", "")
LOCATION = os.environ.get("LOCATION", "/tmp/git")
ARCH = os.environ.get("ARCH", "amd64")

def load_packages():
    pkgs = []
    for yml in ["mrs.yaml", "thirdparty.yaml", "nonbloom.yaml"]:
        path = os.path.join(REPO_PATH, yml)
        if not os.path.exists(path):
            continue
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        for name, props in data.items():
            if ARCH not in props.get("architecture", []):
                continue
            url = props.get("source", "")
            refs = props.get("git_refs", {})
            rel = refs.get("stable")
            test = refs.get("testing")
            if rel and test and rel != "none" and test != "none":
                pkgs.append({
                    "name": name,
                    "url": url,
                    "release_branch": rel,
                    "testing_branch": test,
                })
    return pkgs

def check_branch_status(pkg):
    url = pkg["url"]
    auth_url = url.replace("https://", f"https://{PUSH_TOKEN}@") if PUSH_TOKEN else url
    rel = pkg["release_branch"]
    test = pkg["testing_branch"]

    cmd = ["git", "ls-remote", "--heads", auth_url, rel, test]
    res = subprocess.run(cmd, capture_output=True, text=True)
    shas = {}
    for line in res.stdout.strip().split("\n"):
        if "\t" in line:
            sha, ref = line.split("\t")
            branch = ref.replace("refs/heads/", "")
            shas[branch] = sha

    rel_sha = shas.get(rel)
    test_sha = shas.get(test)
    needs_merge = (test_sha is not None) and (rel_sha != test_sha)
    return pkg, needs_merge, rel_sha, test_sha

packages = load_packages()
print(f"[fast_merge_push] Checking {len(packages)} repositories in parallel (arch: {ARCH})...")

with ThreadPoolExecutor(max_workers=16) as pool:
    results = list(pool.map(check_branch_status, packages))

to_merge = [r for r in results if r[1]]
already_synced = len(results) - len(to_merge)

print(f"[fast_merge_push] {already_synced}/{len(results)} repositories already up-to-date.")

if not to_merge:
    print("[fast_merge_push] All repositories are up-to-date! No merges or pushes needed.")
    sys.exit(0)

print(f"[fast_merge_push] {len(to_merge)} repositories need merging:")
for p, _, rel_sha, test_sha in to_merge:
    r_short = rel_sha[:7] if rel_sha else "none"
    t_short = test_sha[:7] if test_sha else "none"
    print(f"  • {p['name']}: {p['release_branch']} ({r_short}) <- {p['testing_branch']} ({t_short})")

os.makedirs(LOCATION, exist_ok=True)

for p, _, rel_sha, test_sha in to_merge:
    pkg = p["name"]
    raw_url = p["url"]
    auth_url = raw_url.replace("https://", f"https://{PUSH_TOKEN}@") if PUSH_TOKEN else raw_url
    rel = p["release_branch"]
    test = p["testing_branch"]
    pkg_dir = os.path.join(LOCATION, pkg)

    print(f"\n[fast_merge_push] Processing '{pkg}'...")
    if os.path.exists(pkg_dir):
        subprocess.run(["rm", "-rf", pkg_dir], check=True)

    if rel_sha is None:
        print(f"[fast_merge_push] Creating new branch '{rel}' from '{test}'...")
        subprocess.run(["git", "clone", auth_url, "--branch", test, pkg_dir], check=True)
        subprocess.run(["git", "-C", pkg_dir, "config", "user.name", "GitHub Action"], check=True)
        subprocess.run(["git", "-C", pkg_dir, "config", "user.email", "action@github.com"], check=True)
        subprocess.run(["git", "-C", pkg_dir, "checkout", "-b", rel], check=True)
        subprocess.run(["git", "-C", pkg_dir, "push", "-u", "origin", rel], check=True)
    else:
        print(f"[fast_merge_push] Cloning '{rel}'...")
        subprocess.run(["git", "clone", auth_url, "--branch", rel, pkg_dir], check=True)
        subprocess.run(["git", "-C", pkg_dir, "config", "user.name", "GitHub Action"], check=True)
        subprocess.run(["git", "-C", pkg_dir, "config", "user.email", "action@github.com"], check=True)
        print(f"[fast_merge_push] Merging 'origin/{test}' into '{rel}'...")
        subprocess.run(["git", "-C", pkg_dir, "merge", f"origin/{test}", "-m", f"Merge {test} into {rel}"], check=True)
        print(f"[fast_merge_push] Dry-run pushing '{rel}'...")
        subprocess.run(["git", "-C", pkg_dir, "push", "--dry-run"], check=True)
        print(f"[fast_merge_push] Pushing '{rel}'...")
        subprocess.run(["git", "-C", pkg_dir, "push", "origin", rel], check=True)

print("\n[fast_merge_push] All merges completed successfully!")
PYEOF
