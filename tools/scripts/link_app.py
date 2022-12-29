import argparse
import json
import os
import sys

import packmanapi
import urllib3


def find_omniverse_apps():
    http = urllib3.PoolManager()
    try:
        r = http.request("GET", "http://127.0.0.1:33480/components")
    except Exception as e:
        print(f"Failed retrieving apps from an Omniverse Launcher, maybe it is not installed?\nError: {e}")
        sys.exit(1)

    apps = {}
    for x in json.loads(r.data.decode("utf-8")):
        latest = x.get("installedVersions", {}).get("latest", "")
        if latest:
            for s in x.get("settings", []):
                if s.get("version", "") == latest:
                    root = s.get("launch", {}).get("root", "")
                    apps[x["slug"]] = (x["name"], root)
                    break
    return apps


def create_link(src, dst):
    print(f"Creating a link '{src}' -> '{dst}'")
    packmanapi.link(src, dst)


APP_PRIORITIES = ["code", "create", "view"]

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Create folder link to Kit App installed from Omniverse Launcher")
    parser.add_argument(
        "--path",
        help="Path to Kit App installed from Omniverse Launcher, e.g.: 'C:/Users/bob/AppData/Local/ov/pkg/create-2021.3.4'",
        required=False,
    )
    parser.add_argument(
        "--app", help="Name of Kit App installed from Omniverse Launcher, e.g.: 'code', 'create'", required=False
    )
    args = parser.parse_args()

    path = args.path
    if not path:
        print("Path is not specified, looking for Omniverse Apps...")
        apps = find_omniverse_apps()
        if len(apps) == 0:
            print(
                "Can't find any Omniverse Apps. Use Omniverse Launcher to install one. 'Code' is the recommended app for developers."
            )
            sys.exit(0)

        print("\nFound following Omniverse Apps:")
        for i, slug in enumerate(apps):
            name, root = apps[slug]
            print(f"{i}: {name} ({slug}) at: '{root}'")

        if args.app:
            selected_app = args.app.lower()
            if selected_app not in apps:
                choices = ", ".join(apps.keys())
                print(f"Passed app: '{selected_app}' is not found. Specify one of the following found Apps: {choices}")
                sys.exit(0)
        else:
            selected_app = next((x for x in APP_PRIORITIES if x in apps), None)
            if not selected_app:
                selected_app = next(iter(apps))

        print(f"\nSelected app: {selected_app}")
        _, path = apps[selected_app]

    if not os.path.exists(path):
        print(f"Provided path doesn't exist: {path}")
    else:
        SCRIPT_ROOT = os.path.dirname(os.path.realpath(__file__))
        create_link(f"{SCRIPT_ROOT}/../../app", path)
        print("Success!")
